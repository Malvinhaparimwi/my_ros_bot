#!/usr/bin/env python3
"""Train a tiny YOLOv8-style anchor-free detector for plant bounding boxes and export ONNX.

Dataset layout (YOLO format):
    dataset/
      images/
        frame_0001.jpg
      labels/
        frame_0001.txt   # one row per box: <class> <cx> <cy> <w> <h>  (all 0-1 normalised)

All values in label files must be normalised to [0, 1] relative to image size.
Class index 0 = plant (only class used here).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset


# ---------------------------------------------------------------------------
# Model – YOLOv8-style anchor-free detector
# ---------------------------------------------------------------------------

class ConvBnRelu(nn.Module):
    def __init__(self, in_ch, out_ch, k=3, s=1):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, k, stride=s, padding=k // 2, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.SiLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class TinyDetectorYOLOv8(nn.Module):
    """
    YOLOv8-style detector:
    - Backbone: 4 strided convs → input / 16 feature map
    - Two output heads:
      * Classification head: (num_classes,) per cell
      * Bbox regression head: (4,) per cell (direct dx, dy, dw, dh prediction)
    - At stride-16, a 160×96 input gives 10×6 = 60 cell grid.
    """

    def __init__(self, base_channels: int = 16, num_classes: int = 1):
        super().__init__()
        self.num_classes = num_classes
        b = base_channels
        
        self.backbone = nn.Sequential(
            ConvBnRelu(3,     b,   s=2),   # /2
            ConvBnRelu(b,     b*2, s=2),   # /4
            ConvBnRelu(b*2,   b*4, s=2),   # /8
            ConvBnRelu(b*4,   b*8, s=2),   # /16
            ConvBnRelu(b*8,   b*8),
        )
        
        # Classification head: (num_classes) per cell
        self.cls_head = nn.Sequential(
            ConvBnRelu(b*8, b*8, k=3),
            nn.Conv2d(b*8, num_classes, 1),
        )
        
        # Bbox regression head: (4) per cell [dx, dy, dw, dh]
        self.bbox_head = nn.Sequential(
            ConvBnRelu(b*8, b*8, k=3),
            nn.Conv2d(b*8, 4, 1),
        )

    def forward(self, x):
        feat = self.backbone(x)                    # (B, C, H/16, W/16)
        cls_logits = self.cls_head(feat)           # (B, num_classes, Gy, Gx)
        bbox_pred = self.bbox_head(feat)           # (B, 4, Gy, Gx)
        return cls_logits, bbox_pred


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

def parse_yolo_label(path: Path):
    """Return (N,5) array of [cls, cx, cy, w, h] or empty (0,5)."""
    if not path.exists():
        return np.zeros((0, 5), dtype=np.float32)
    rows = []
    for line in path.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) == 5:
            rows.append([float(p) for p in parts])
    return np.array(rows, dtype=np.float32) if rows else np.zeros((0, 5), dtype=np.float32)


class PlantDetectionDataset(Dataset):
    def __init__(self, root: str, width: int, height: int, stride: int = 16):
        self.root   = Path(root)
        self.width  = width
        self.height = height
        self.stride = stride
        self.gw     = width  // stride
        self.gh     = height // stride
        self.image_paths = sorted((self.root / "images").glob("*"))
        self.label_dir   = self.root / "labels"
        if not self.image_paths:
            raise RuntimeError(f"No images found in {self.root / 'images'}")

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        img_path   = self.image_paths[idx]
        label_path = self.label_dir / f"{img_path.stem}.txt"

        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Cannot read {img_path}")
        img = cv2.resize(img, (self.width, self.height))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        img = img.transpose(2, 0, 1) / 255.0          # (3, H, W)

        boxes = parse_yolo_label(label_path)          # (N, 5): [cls, cx, cy, w, h]
        
        # Build target grids
        cls_target = np.zeros((1, self.gh, self.gw), dtype=np.float32)
        bbox_target = np.zeros((4, self.gh, self.gw), dtype=np.float32)
        
        # Assign each box to its grid cell
        for box in boxes:
            cls, cx, cy, bw, bh = box
            
            # Grid cell coordinates
            gx = int(cx * self.gw)
            gy = int(cy * self.gh)
            gx = min(gx, self.gw - 1)
            gy = min(gy, self.gh - 1)
            
            # Objectness (presence of plant in this cell)
            cls_target[0, gy, gx] = 1.0
            
            # Bbox regression: direct offsets
            bbox_target[0, gy, gx] = cx * self.gw - gx      # dx (offset within cell, 0-1)
            bbox_target[1, gy, gx] = cy * self.gh - gy      # dy (offset within cell, 0-1)
            bbox_target[2, gy, gx] = bw                      # w (normalized)
            bbox_target[3, gy, gx] = bh                      # h (normalized)

        return torch.from_numpy(img), torch.from_numpy(cls_target), torch.from_numpy(bbox_target)


# ---------------------------------------------------------------------------
# Loss (YOLOv8-style)
# ---------------------------------------------------------------------------

class YOLOv8Loss(nn.Module):
    """
    YOLOv8-style loss combining classification and bbox regression.
    """
    def __init__(self, lambda_bbox: float = 7.5, lambda_cls: float = 0.5):
        super().__init__()
        self.lambda_bbox = lambda_bbox
        self.lambda_cls = lambda_cls
        self.bce = nn.BCEWithLogitsLoss(reduction="mean")

    def forward(self, cls_pred, bbox_pred, cls_target, bbox_target):
        """
        cls_pred:    (B, 1, Gy, Gx)  – logits
        bbox_pred:   (B, 4, Gy, Gx)  – predicted [dx, dy, dw, dh]
        cls_target:  (B, 1, Gy, Gx)  – {0, 1}
        bbox_target: (B, 4, Gy, Gx)  – ground truth [dx, dy, dw, dh]
        """
        # Classification loss (BCEWithLogits handles sigmoid internally)
        cls_loss = self.bce(cls_pred, cls_target)
        
        # Bbox loss (only for positive cells)
        obj_mask = cls_target > 0.5                         # (B, 1, Gy, Gx)
        bbox_loss = (obj_mask * (bbox_pred - bbox_target).pow(2)).sum() / (obj_mask.sum() + 1e-6)
        
        total_loss = cls_loss + self.lambda_bbox * bbox_loss
        return total_loss


# ---------------------------------------------------------------------------
# ONNX export
# ---------------------------------------------------------------------------

def export_onnx(model, output, width, height):
    try:
        import onnx  # noqa: F401
    except ModuleNotFoundError:
        print(
            "ONNX export needs the Python package 'onnx'. Install it with:\n"
            "  python3 -m pip install onnx\n\n"
            "Your PyTorch checkpoint is still saved, so after installing onnx "
            "you can export without retraining.",
            file=sys.stderr,
        )
        raise

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    model.eval().cpu()
    dummy = torch.zeros(1, 3, height, width)
    
    torch.onnx.export(
        model,
        dummy,
        str(output),
        input_names=["image"],
        output_names=["cls_logits", "bbox_pred"],
        opset_version=12,
        dynamic_axes={
            "image": {0: "batch"},
            "cls_logits": {0: "batch"},
            "bbox_pred": {0: "batch"},
        },
    )
    print(f"Exported ONNX → {output}")
    print("  cls_logits: (batch, 1, grid_h, grid_w)  – objectness logits")
    print("  bbox_pred:  (batch, 4, grid_h, grid_w)  – [dx, dy, dw, dh]")


# ---------------------------------------------------------------------------
# Checkpoint helpers
# ---------------------------------------------------------------------------

def save_checkpoint(model, args, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state":   model.cpu().state_dict(),
            "base_channels": args.base_channels,
            "num_classes":   args.num_classes,
            "width":         args.width,
            "height":        args.height,
        },
        path,
    )
    print(f"Saved checkpoint → {path}")


def load_checkpoint(path):
    ckpt = torch.load(path, map_location="cpu")
    model = TinyDetectorYOLOv8(
        base_channels=ckpt.get("base_channels", 16),
        num_classes=ckpt.get("num_classes", 1),
    )
    model.load_state_dict(ckpt["model_state"])
    return model, ckpt


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------

def train(args):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}\n")

    # ── export-only mode ──────────────────────────────────────────────────
    if args.export_checkpoint:
        model, ckpt = load_checkpoint(args.export_checkpoint)
        export_onnx(model, args.output, ckpt.get("width", args.width), ckpt.get("height", args.height))
        return

    # ── dataset ───────────────────────────────────────────────────────────
    dataset = PlantDetectionDataset(args.dataset, args.width, args.height)
    loader  = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, num_workers=2)
    print(f"Dataset: {len(dataset)} images  |  Grid: {dataset.gw}×{dataset.gh}\n")

    # ── model + optimiser ─────────────────────────────────────────────────
    model     = TinyDetectorYOLOv8(args.base_channels, args.num_classes).to(device)
    criterion = YOLOv8Loss(lambda_bbox=args.lambda_bbox, lambda_cls=args.lambda_cls)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    print(f"Model params: {sum(p.numel() for p in model.parameters()):,}\n")

    # ── loop ──────────────────────────────────────────────────────────────
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0.0
        
        for images, cls_targets, bbox_targets in loader:
            images      = images.to(device)
            cls_targets = cls_targets.to(device)
            bbox_targets = bbox_targets.to(device)
            
            optimizer.zero_grad(set_to_none=True)
            
            cls_pred, bbox_pred = model(images)
            loss = criterion(cls_pred, bbox_pred, cls_targets, bbox_targets)
            
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
            optimizer.step()
            
            total_loss += loss.item() * images.size(0)

        scheduler.step()
        avg_loss = total_loss / len(dataset)
        lr = scheduler.get_last_lr()[0]
        print(f"epoch {epoch + 1:03d}/{args.epochs}: loss={avg_loss:.4f}  lr={lr:.2e}")

    save_checkpoint(model, args, args.checkpoint)
    export_onnx(model, args.output, args.width, args.height)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Tiny YOLOv8-style plant detector")
    p.add_argument("--dataset",           help="Path to dataset root (images/ + labels/)")
    p.add_argument("--output",            default="models/tiny_plant_det.onnx")
    p.add_argument("--checkpoint",        default="models/tiny_plant_det.pt")
    p.add_argument("--export-checkpoint", help="Load a .pt checkpoint and export ONNX only")
    p.add_argument("--width",             type=int,   default=160)
    p.add_argument("--height",            type=int,   default=96)
    p.add_argument("--base-channels",     type=int,   default=16)
    p.add_argument("--num-classes",       type=int,   default=1)
    p.add_argument("--epochs",            type=int,   default=80)
    p.add_argument("--batch-size",        type=int,   default=16)
    p.add_argument("--lr",                type=float, default=1e-3)
    p.add_argument("--lambda-bbox",       type=float, default=7.5,
                   help="Weight for bbox loss vs cls loss")
    p.add_argument("--lambda-cls",        type=float, default=0.5,
                   help="Weight for classification loss")
    args = p.parse_args()
    if not args.export_checkpoint and not args.dataset:
        p.error("--dataset is required unless --export-checkpoint is used")
    return args


if __name__ == "__main__":
    train(parse_args())