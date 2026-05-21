# Move Base Row Following

This package follows the middle crop row from `/camera/left/compressed` using
a tiny U-Net segmentation model exported to ONNX. The runtime uses OpenCV DNN,
so the Raspberry Pi does not need PyTorch installed.

The U-Net produces a plant-row mask, the row follower estimates the row center
from that mask, and the node publishes conservative steering commands on
`/onyx/cmd_vel`.

## 1. Collect Data

Run the robot camera and save frames while you drive manually:

```bash
ros2 run move_base collect_row_data --ros-args \
  -p image_topic:=/camera/left/compressed \
  -p output_dir:=row_data/raw \
  -p save_every_n_frames:=10
```

Collect straight rows, curves, shadows, bright sun, missing plants, and row-end
cases. Keep the robot speed slow while collecting so frames are not blurred.

## 2. Label Masks For U-Net

Create this dataset layout:

```text
row_data/dataset/
  images/
    row_000001.jpg
  masks/
    row_000001.png
```

For each image, make a same-name PNG mask where plant pixels are white and
background is black. You can label masks with tools like CVAT, Label Studio,
Roboflow, or any image editor.

## 3. Train Tiny U-Net

Train on your laptop or desktop, then copy the ONNX file to the Raspberry Pi:

```bash
python3 src/move_base/tools/train_tiny_unet.py \
  --dataset row_data/dataset \
  --output models/tiny_row_unet.onnx \
  --width 160 \
  --height 96 \
  --epochs 80
```

The robot runtime uses OpenCV DNN, so the Pi does not need PyTorch.

## 4. Tune Without Driving

Start the camera, then run the row follower in dry-run mode:

```bash
ros2 launch move_base row_follow.launch.py \
  dry_run:=true \
  display_debug:=true \
  segmentation_model_path:=models/tiny_row_unet.onnx
```

Watch `/move_base/row_debug/compressed` or the OpenCV debug window. The red
vertical line should sit on the middle row. The white line is the camera center.

## 5. Drive Autonomously

When the debug view is stable, run slowly:

```bash
ros2 launch move_base row_follow.launch.py \
  dry_run:=false \
  segmentation_model_path:=models/tiny_row_unet.onnx \
  max_linear_speed:=0.08 \
  steering_gain:=0.75
```

Increase `max_linear_speed` only after the robot can recover from small offsets.
Use a physical emergency stop or be ready to stop the motor controller.

## Useful Parameters

- `process_every_n_frames`: higher values reduce CPU load.
- `segmentation_model_path`: path to the exported `.onnx` U-Net.
- `model_threshold`: segmentation probability threshold.
- `confidence_stop_threshold`: raise it if false detections move the robot.
- `lost_timeout_sec`: how long to keep creeping after briefly losing the row.
- `max_angular_speed`: steering command clamp.
- `publish_debug`: publishes `/move_base/row_debug/compressed`.
