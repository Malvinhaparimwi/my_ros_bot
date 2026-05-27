from ultralytics import YOLO
import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────
SCORE_THRESH = 0.51
MODEL_PATH   = "/home/eath/my_ros_bot/src/move_base/tools/yolov8n_best.pt"
IMG_PATH     = "/home/eath/front_camera/test/row_345.jpg"
OUT_PATH     = "/home/eath/my_ros_bot/src/move_base/inference_result.png"

# ── Load YOLOv8 model ─────────────────────────────────────────
model = YOLO(MODEL_PATH)
print(f"Model loaded: {MODEL_PATH}")

# ── Run inference ─────────────────────────────────────────────
results = model(IMG_PATH, conf=SCORE_THRESH, verbose=False)

# ── Draw with OpenCV ─────────────────────────────────────────
img_cv = cv2.imread(IMG_PATH)

for result in results:
    boxes = result.boxes
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]
        
        cv2.rectangle(img_cv, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label_text = f"{cls_name} {conf:.2f}"
        (tw, th), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img_cv, (x1, y1 - th - baseline - 4), (x1 + tw, y1), (0, 0, 255), -1)
        cv2.putText(img_cv, label_text, (x1, y1 - baseline - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

cv2.imshow("YOLOv8 Detection", img_cv)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite(OUT_PATH, img_cv)
print(f"Saved: {OUT_PATH}")