#!/usr/bin/env python3
"""
Test the TinyDetector ONNX model on a single image.

Usage:
    python test_detector.py --model models/tiny_row_det.onnx --image frame_0001.jpg

Optional:
    --threshold 0.3        lower if you are missing detections
    --save result.jpg      save annotated image instead of / as well as showing it
    --no-show              skip the display window (useful on headless machines)
    --width 160            must match what you trained with (default 160)
    --height 96            must match what you trained with (default 96)
    --nms-iou 0.5          NMS IoU threshold (default 0.5)
    --y-cluster-dist 15    max Y distance to cluster detections (default 15)
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy import stats


# ── decode one grid output ────────────────────────────────────────────────────

def decode_detections(raw, grid_w, grid_h, roi_w, roi_h, threshold):
    """
    raw       : numpy array (4, grid_h, grid_w)  – YOLOv8 style from ONNX model
                Channel 0: cls_logits (objectness)
                Channels 1-4: bbox_pred [dx, dy, dw, dh]
    returns   : list of dicts with keys cx, cy, bw, bh, score  (all in ROI pixels)
    """
    raw = np.squeeze(raw)            # handles (1,4,Gy,Gx) or (4,Gy,Gx)

    cls_logits = raw[0]              # (grid_h, grid_w)
    dx, dy, dw, dh = raw[1], raw[2], raw[3], raw[4] if raw.shape[0] > 4 else (raw[1], raw[2], raw[3], raw[3])

    # Handle case where we only have 4 channels total
    if raw.shape[0] == 4:
        cls_logits = raw[0]          # (grid_h, grid_w)
        dx = raw[1]                  # (grid_h, grid_w)
        dy = raw[2]                  # (grid_h, grid_w)
        dw = raw[3]                  # (grid_h, grid_w)
        dh = raw[3]                  # reuse dw for dh (or assume square)

    scores = 1.0 / (1.0 + np.exp(-cls_logits))   # sigmoid

    detections = []
    gy_vals, gx_vals = np.where(scores >= threshold)
    for gy, gx in zip(gy_vals, gx_vals):
        score = float(scores[gy, gx])

        # Decode box from grid cell
        # dx, dy are offsets within cell (0-1)
        # dw, dh are normalized widths/heights
        cx_norm = (gx + dx[gy, gx]) / grid_w
        cy_norm = (gy + dy[gy, gx]) / grid_h
        bw_norm = float(dw[gy, gx])
        bh_norm = float(dh[gy, gx])

        detections.append({
            "cx":    cx_norm * roi_w,
            "cy":    cy_norm * roi_h,
            "bw":    bw_norm * roi_w,
            "bh":    bh_norm * roi_h,
            "score": score,
            "cell":  (int(gx), int(gy)),
        })

    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections


# ── NMS (Non-Maximum Suppression) ──────────────────────────────────────────────

def nms(detections, iou_threshold=0.5):
    """
    Apply Non-Maximum Suppression to remove overlapping boxes.
    """
    if not detections:
        return []

    # Sort by score (descending)
    sorted_dets = sorted(detections, key=lambda d: d["score"], reverse=True)
    keep = []

    while sorted_dets:
        current = sorted_dets.pop(0)
        keep.append(current)

        # Remove boxes with high IoU with current box
        remaining = []
        for det in sorted_dets:
            iou = compute_iou(current, det)
            if iou < iou_threshold:
                remaining.append(det)
        sorted_dets = remaining

    return keep


def compute_iou(box1, box2):
    """Compute IoU (Intersection over Union) between two boxes."""
    x1_min = box1["cx"] - box1["bw"] / 2
    y1_min = box1["cy"] - box1["bh"] / 2
    x1_max = box1["cx"] + box1["bw"] / 2
    y1_max = box1["cy"] + box1["bh"] / 2

    x2_min = box2["cx"] - box2["bw"] / 2
    y2_min = box2["cy"] - box2["bh"] / 2
    x2_max = box2["cx"] + box2["bw"] / 2
    y2_max = box2["cy"] + box2["bh"] / 2

    inter_x_min = max(x1_min, x2_min)
    inter_y_min = max(y1_min, y2_min)
    inter_x_max = min(x1_max, x2_max)
    inter_y_max = min(y1_max, y2_max)

    if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
        return 0.0

    inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
    box1_area = box1["bw"] * box1["bh"]
    box2_area = box2["bw"] * box2["bh"]
    union_area = box1_area + box2_area - inter_area

    return inter_area / union_area if union_area > 0 else 0.0


# ── cluster by Y position ──────────────────────────────────────────────────────

def cluster_by_y(detections, y_distance=15):
    """
    Cluster detections by Y position (same row).
    Returns list of clusters, each cluster is a list of detections.
    """
    if not detections:
        return []

    # Sort by Y position
    sorted_dets = sorted(detections, key=lambda d: d["cy"])
    clusters = []
    current_cluster = [sorted_dets[0]]

    for det in sorted_dets[1:]:
        if det["cy"] - current_cluster[-1]["cy"] <= y_distance:
            current_cluster.append(det)
        else:
            clusters.append(current_cluster)
            current_cluster = [det]

    clusters.append(current_cluster)
    return clusters


# ── cluster by X position ──────────────────────────────────────────────────────

def cluster_by_x(detections, x_distance=15):
    """
    Cluster detections by X position (same vertical column/row).
    Returns list of clusters, each cluster is a list of detections.
    """
    if not detections:
        return []

    # Sort by X position
    sorted_dets = sorted(detections, key=lambda d: d["cx"])
    clusters = []
    current_cluster = [sorted_dets[0]]

    for det in sorted_dets[1:]:
        if det["cx"] - current_cluster[-1]["cx"] <= x_distance:
            current_cluster.append(det)
        else:
            clusters.append(current_cluster)
            current_cluster = [det]

    clusters.append(current_cluster)
    return clusters


# ── fit line and extract row centre ────────────────────────────────────────────

def fit_line_to_cluster(cluster):
    """
    Fit a line to cluster detections (least-squares fit on Y positions).
    Returns row centre info: {x_mean, y_positions, slope, intercept, detections_count}
    """
    if not cluster:
        return None

    # Use Y positions to fit a vertical line (linear regression on Y vs X)
    x_coords = np.array([det["cx"] for det in cluster])
    y_coords = np.array([det["cy"] for det in cluster])

    # Fit line: x = slope * y + intercept
    coeffs = np.polyfit(y_coords, x_coords, 1)
    slope, intercept = coeffs[0], coeffs[1]

    x_mean = np.mean(x_coords)
    y_mean = np.mean(y_coords)

    return {
        "x_mean": x_mean,
        "y_mean": y_mean,
        "slope": slope,
        "intercept": intercept,
        "x_coords": x_coords,
        "y_coords": y_coords,
        "count": len(cluster),
        "detections": cluster,
    }


# ── draw ─────────────────────────────────────────────────────────────────────

def draw_detections(image, row_fits, detections_before_nms=None):
    """Draw fitted rows and raw detections on image."""
    out = image.copy()
    h, w = out.shape[:2]

    # Draw fitted lines for each row
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]

    for i, row_fit in enumerate(row_fits):
        color = colors[i % len(colors)]

        # Draw VERTICAL line (x = slope * y + intercept)
        x1 = int(row_fit["intercept"])
        y1 = 0
        x2 = int(row_fit["slope"] * h + row_fit["intercept"])
        y2 = h
        cv2.line(out, (x1, y1), (x2, y2), color, 2)

        # Draw detection boxes in this cluster
        for det in row_fit["detections"]:
            cx, cy = det["cx"], det["cy"]
            bw, bh = det["bw"], det["bh"]
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            cv2.circle(out, (int(cx), int(cy)), 3, color, -1)

        # Draw row label
        label = f"Row {i+1} ({row_fit['count']} det)  x={row_fit['x_mean']:.1f}"
        cv2.putText(out, label, (10, 25 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    summary = f"{len(row_fits)} rows detected"
    cv2.putText(out, summary, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 230, 0), 2, cv2.LINE_AA)

    return out


# ── grid heat-map (objectness scores across all cells) ───────────────────────

def make_heatmap(raw, grid_w, grid_h, display_w=320, display_h=192):
    raw      = np.squeeze(raw)
    logits   = raw[0]                                # (grid_h, grid_w)
    scores   = 1.0 / (1.0 + np.exp(-logits))
    heat     = (scores * 255).astype(np.uint8)
    heat     = cv2.resize(heat, (display_w, display_h), interpolation=cv2.INTER_NEAREST)
    heat_rgb = cv2.applyColorMap(heat, cv2.COLORMAP_JET)

    # annotate each cell with its score
    cell_w = display_w // grid_w
    cell_h = display_h // grid_h
    for gy in range(grid_h):
        for gx in range(grid_w):
            s   = float(scores[gy, gx])
            px  = gx * cell_w + 2
            py  = gy * cell_h + cell_h - 4
            cv2.putText(heat_rgb, f"{s:.2f}", (px, py),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)
    return heat_rgb


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Test TinyDetector ONNX on a single image with row fitting")
    ap.add_argument("--model",     required=True, help="Path to .onnx model")
    ap.add_argument("--image",     required=True, help="Path to test image")
    ap.add_argument("--threshold", type=float, default=0.5,
                    help="Objectness threshold (default 0.5)")
    ap.add_argument("--width",     type=int,   default=160, help="Model input width")
    ap.add_argument("--height",    type=int,   default=96,  help="Model input height")
    ap.add_argument("--nms-iou",   type=float, default=0.5,  help="NMS IoU threshold")
    ap.add_argument("--y-cluster-dist", type=int, default=15, help="Max Y distance to cluster detections")
    ap.add_argument("--save",      default="", help="Save annotated image to this path")
    ap.add_argument("--no-show",   action="store_true", help="Skip display window")
    args = ap.parse_args()

    # ── load model ───────────────────────────────────────────────────────
    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")
    net = cv2.dnn.readNetFromONNX(str(model_path))
    print(f"Loaded model : {model_path}")

    grid_w = args.width  // 16
    grid_h = args.height // 16
    print(f"Grid         : {grid_w}×{grid_h}  (stride 16,  {grid_w*grid_h} cells total)")

    # ── load image ───────────────────────────────────────────────────────
    img_path = Path(args.image)
    if not img_path.exists():
        raise FileNotFoundError(f"Image not found: {img_path}")
    frame = cv2.imread(str(img_path))
    if frame is None:
        raise RuntimeError(f"cv2.imread failed for {img_path}")
    print(f"Image size   : {frame.shape[1]}×{frame.shape[0]}")

    # ── pre-process (must match training) ────────────────────────────────
    resized = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_AREA)
    rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    blob    = cv2.dnn.blobFromImage(
        rgb,
        scalefactor=1.0 / 255.0,
        size=(args.width, args.height),
        mean=(0.0, 0.0, 0.0),
        swapRB=False,
        crop=False,
    )

    # ── inference ────────────────────────────────────────────────────────
    net.setInput(blob)
    outputs = net.forward()           # Two outputs: cls_logits, bbox_pred
    
    # Handle both single output and multiple outputs
    if isinstance(outputs, np.ndarray) and outputs.ndim == 4:
        # Single concatenated output: stack cls_logits and bbox_pred
        cls_logits = outputs[:, 0:1, :, :]   # (1, 1, 6, 10)
        bbox_pred = outputs[:, 1:5, :, :]    # (1, 4, 6, 10)
        raw = np.concatenate([cls_logits, bbox_pred], axis=1)  # (1, 5, 6, 10)
    else:
        raw = outputs[0] if isinstance(outputs, list) else outputs
    
    print(f"Raw output   : shape={raw.shape}  min={raw.min():.3f}  max={raw.max():.3f}")

    # ── PIPELINE: raw detections → NMS → cluster by X → fit line ─────────
    
    # Step 1: Decode raw detections
    detections_raw = decode_detections(
        raw, grid_w, grid_h,
        roi_w=frame.shape[1],
        roi_h=frame.shape[0],
        threshold=args.threshold,
    )
    print(f"\n{'─'*55}")
    print(f"Step 1 - Raw detections: {len(detections_raw)}")

    # Step 2: NMS
    detections_nms = nms(detections_raw, iou_threshold=args.nms_iou)
    print(f"Step 2 - After NMS (IoU={args.nms_iou}): {len(detections_nms)}")

    # Step 3: Cluster by X position (vertical columns)
    clusters = cluster_by_x(detections_nms, x_distance=args.y_cluster_dist)
    print(f"Step 3 - Clustered by X (dist={args.y_cluster_dist}): {len(clusters)} rows")

    # Step 4: Fit line to each cluster
    row_fits = []
    for i, cluster in enumerate(clusters):
        row_fit = fit_line_to_cluster(cluster)
        if row_fit:
            row_fits.append(row_fit)
            print(f"  Row {i+1}: {row_fit['count']} detections, "
                  f"x_mean={row_fit['x_mean']:.1f}, slope={row_fit['slope']:.3f}")

    print(f"Step 4 - Extracted {len(row_fits)} fitted rows")
    print(f"{'─'*55}\n")

    # ── visualise ────────────────────────────────────────────────────────
    annotated = draw_detections(frame, row_fits)
    heatmap   = make_heatmap(raw, grid_w, grid_h,
                              display_w=frame.shape[1],
                              display_h=max(80, frame.shape[0] // 3))

    combined = np.vstack([annotated, heatmap])

    if args.save:
        save_path = Path(args.save)
        cv2.imwrite(str(save_path), combined)
        print(f"Saved annotated image → {save_path}")

    if not args.no_show:
        cv2.imshow("TinyDetector row fitting  (press any key to quit)", combined)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()