#!/usr/bin/env python3
"""ONNX anchor-free detector crop-row follower for ROS 2."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage


@dataclass
class RowEstimate:
    center_x: float
    confidence: float
    box_xywh: Tuple[float, float, float, float]   # pixel coords in ROI space


class RowFollower(Node):
    def __init__(self):
        super().__init__("row_follower")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_topic",              "/camera/left/compressed"),
                ("cmd_vel_topic",            "/onyx/cmd_vel"),
                ("debug_topic",              "/move_base/row_debug/compressed"),
                ("publish_debug",            True),
                ("display_debug",            True),
                ("dry_run",                  False),
                ("process_every_n_frames",   2),
                ("roi_top_ratio",            0.38),
                ("roi_bottom_ratio",         0.98),
                ("segmentation_model_path",  ""),   # kept same param name for launch compat
                ("model_input_width",        160),
                ("model_input_height",       96),
                ("model_threshold",          0.5),  # objectness threshold
                ("max_linear_speed",         0.12),
                ("search_linear_speed",      0.04),
                ("max_angular_speed",        0.9),
                ("steering_gain",            0.85),
                ("confidence_stop_threshold",0.08),
                ("lost_timeout_sec",         0.8),
                ("command_rate_hz",          10.0),
            ],
        )

        # ── read params ──────────────────────────────────────────────────
        self.image_topic             = self.get_parameter("image_topic").value
        self.cmd_vel_topic           = self.get_parameter("cmd_vel_topic").value
        self.debug_topic             = self.get_parameter("debug_topic").value
        self.publish_debug           = self.get_parameter("publish_debug").value
        self.display_debug           = self.get_parameter("display_debug").value
        self.dry_run                 = self.get_parameter("dry_run").value
        self.process_every_n_frames  = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.roi_top_ratio           = float(self.get_parameter("roi_top_ratio").value)
        self.roi_bottom_ratio        = float(self.get_parameter("roi_bottom_ratio").value)
        self.segmentation_model_path = self.get_parameter("segmentation_model_path").value
        self.model_input_width       = int(self.get_parameter("model_input_width").value)
        self.model_input_height      = int(self.get_parameter("model_input_height").value)
        self.model_threshold         = float(self.get_parameter("model_threshold").value)
        self.max_linear_speed        = float(self.get_parameter("max_linear_speed").value)
        self.search_linear_speed     = float(self.get_parameter("search_linear_speed").value)
        self.max_angular_speed       = float(self.get_parameter("max_angular_speed").value)
        self.steering_gain           = float(self.get_parameter("steering_gain").value)
        self.confidence_stop_threshold = float(self.get_parameter("confidence_stop_threshold").value)
        self.lost_timeout_sec        = float(self.get_parameter("lost_timeout_sec").value)

        # grid dimensions derived from model input size and stride=16
        self._stride   = 16
        self._grid_w   = self.model_input_width  // self._stride   # e.g. 10
        self._grid_h   = self.model_input_height // self._stride   # e.g. 6

        # ── ROS comms ────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, qos
        )
        self.cmd_pub  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = (
            self.create_publisher(CompressedImage, self.debug_topic, 10)
            if self.publish_debug else None
        )

        # ── state ────────────────────────────────────────────────────────
        self.frame_count    = 0
        self.latest_twist   = Twist()
        self.latest_debug   = None
        self.last_seen_time = None
        self.last_error     = 0.0

        if self.display_debug:
            cv2.namedWindow("Row follower debug", cv2.WINDOW_NORMAL)
            self.get_logger().info("OpenCV debug window enabled")

        self.model_net = self._load_model()

        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.command_timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(f"Following row from {self.image_topic}")
        self.get_logger().info(f"Publishing drive commands to {self.cmd_vel_topic}")
        if self.dry_run:
            self.get_logger().warning("dry_run=true: robot will not move")

    # ── model loading ────────────────────────────────────────────────────

    def _load_model(self):
        if not self.segmentation_model_path:
            raise RuntimeError("segmentation_model_path param is required")
        path = Path(self.segmentation_model_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"ONNX model not found: {path}")
        try:
            net = cv2.dnn.readNetFromONNX(str(path))
        except cv2.error as exc:
            raise RuntimeError(f"Failed to load ONNX model: {exc}")
        self.get_logger().info(f"Loaded ONNX detector: {path}")
        self.get_logger().info(
            f"Grid: {self._grid_w}×{self._grid_h}  stride={self._stride}  "
            f"threshold={self.model_threshold}"
        )
        return net

    # ── ROS callbacks ────────────────────────────────────────────────────

    def image_callback(self, msg: CompressedImage):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        frame = self._decode_frame(msg)
        if frame is None:
            return
        estimate, debug = self._estimate_row(frame)
        self.latest_debug = debug
        self.update_command(estimate, frame.shape[1])
        self._publish_debug_frame(msg.header.stamp)
        self._show_debug_frame()

    @staticmethod
    def _decode_frame(msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except cv2.error:
            return None

    # ── detection ────────────────────────────────────────────────────────

    def _estimate_row(
        self, frame: np.ndarray
    ) -> Tuple[Optional[RowEstimate], np.ndarray]:
        h, w = frame.shape[:2]
        y0 = int(np.clip(self.roi_top_ratio,    0.00, 0.95) * h)
        y1 = int(np.clip(self.roi_bottom_ratio, 0.05, 1.00) * h)
        if y1 <= y0:
            y0, y1 = int(0.4 * h), h

        roi = frame[y0:y1, :]
        detections = self._run_detector(roi)   # list of (cx_px, cy_px, bw_px, bh_px, score)

        debug = self._draw_debug(frame, y0, y1, detections, None)
        if not detections:
            return None, debug

        # pick detection whose centre is closest to the image horizontal mid
        mid = w / 2.0
        best = min(detections, key=lambda d: abs(d[0] - mid))
        cx_px, cy_px, bw_px, bh_px, score = best

        estimate = RowEstimate(
            center_x=cx_px,
            confidence=float(score),
            box_xywh=(cx_px, cy_px, bw_px, bh_px),
        )
        debug = self._draw_debug(frame, y0, y1, detections, estimate)
        return estimate, debug

    def _run_detector(self, roi: np.ndarray):
        """
        Run the TinyDetector ONNX model on the ROI.
        Returns list of (cx_px, cy_px, bw_px, bh_px, score) in ROI pixel coords.
        """
        roi_h, roi_w = roi.shape[:2]

        # pre-process — identical to training
        resized = cv2.resize(roi, (self.model_input_width, self.model_input_height),
                             interpolation=cv2.INTER_AREA)
        rgb  = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        blob = cv2.dnn.blobFromImage(
            rgb,
            scalefactor=1.0 / 255.0,
            size=(self.model_input_width, self.model_input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=False,
            crop=False,
        )
        self.model_net.setInput(blob)
        raw = self.model_net.forward()          # (1, 5, grid_h, grid_w)
        raw = np.squeeze(raw, axis=0)           # (5, grid_h, grid_w)

        obj_logits = raw[0]                     # (grid_h, grid_w)
        tx         = raw[1]
        ty         = raw[2]
        tw         = raw[3]
        th         = raw[4]

        scores = 1.0 / (1.0 + np.exp(-obj_logits))   # sigmoid

        detections = []
        gy_vals, gx_vals = np.where(scores >= self.model_threshold)
        for gy, gx in zip(gy_vals, gx_vals):
            score = float(scores[gy, gx])

            # decode normalised box centre
            cx_norm = (gx + 1.0 / (1.0 + np.exp(-tx[gy, gx]))) / self._grid_w
            cy_norm = (gy + 1.0 / (1.0 + np.exp(-ty[gy, gx]))) / self._grid_h
            bw_norm = float(tw[gy, gx])
            bh_norm = float(th[gy, gx])

            # convert to ROI pixel coords
            cx_px = cx_norm * roi_w
            cy_px = cy_norm * roi_h
            bw_px = bw_norm * roi_w
            bh_px = bh_norm * roi_h

            detections.append((cx_px, cy_px, bw_px, bh_px, score))

        return detections

    # ── steering (unchanged logic from original) ─────────────────────────

    def update_command(self, estimate: Optional[RowEstimate], width: int):
        now   = self.get_clock().now()
        twist = Twist()

        if estimate and estimate.confidence >= self.confidence_stop_threshold:
            self.last_seen_time = now
            error = (estimate.center_x - (width / 2.0)) / (width / 2.0)
            error = 0.65 * error + 0.35 * self.last_error
            self.last_error = error

            twist.linear.x  = self.max_linear_speed * max(0.35, 1.0 - abs(error))
            twist.angular.z = float(np.clip(
                -self.steering_gain * error,
                -self.max_angular_speed,
                self.max_angular_speed,
            ))
        elif self._recently_saw_row(now):
            twist.linear.x  = self.search_linear_speed
            twist.angular.z = float(np.clip(
                -self.steering_gain * self.last_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            ))
        else:
            self.last_error = 0.0

        self.latest_twist = twist

    def _recently_saw_row(self, now) -> bool:
        if self.last_seen_time is None:
            return False
        return (now - self.last_seen_time).nanoseconds / 1e9 <= self.lost_timeout_sec

    def publish_command(self):
        if not self.dry_run:
            self.cmd_pub.publish(self.latest_twist)

    # ── debug visuals ────────────────────────────────────────────────────

    def _draw_debug(self, frame, y0, y1, detections, estimate):
        debug = frame.copy()
        roi_w = frame.shape[1]

        # ROI boundary
        cv2.rectangle(debug, (0, y0), (roi_w - 1, y1 - 1), (255, 190, 0), 2)

        # all detection boxes (green)
        for cx_px, cy_px, bw_px, bh_px, score in detections:
            x1 = int(cx_px - bw_px / 2)
            y_top = int(y0 + cy_px - bh_px / 2)
            x2 = int(cx_px + bw_px / 2)
            y_bot = int(y0 + cy_px + bh_px / 2)
            cv2.rectangle(debug, (x1, y_top), (x2, y_bot), (0, 200, 0), 2)
            cv2.putText(debug, f"{score:.2f}", (x1, max(y_top - 4, y0 + 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1, cv2.LINE_AA)

        # image centre line
        mid = roi_w // 2
        cv2.line(debug, (mid, y0), (mid, y1), (255, 255, 255), 1)

        if estimate:
            cx = int(estimate.center_x)
            cy_px = int(y0 + estimate.box_xywh[1])
            # chosen box in red
            bw, bh = estimate.box_xywh[2], estimate.box_xywh[3]
            cv2.rectangle(debug,
                          (int(cx - bw / 2), int(cy_px - bh / 2)),
                          (int(cx + bw / 2), int(cy_px + bh / 2)),
                          (0, 0, 255), 2)
            cv2.line(debug, (cx, y0), (cx, y1), (0, 0, 255), 2)
            status = (f"det conf={estimate.confidence:.2f} "
                      f"err={self.last_error:.2f} "
                      f"boxes={len(detections)}")
            color = (0, 0, 255)
        else:
            status = f"det no row  boxes={len(detections)}"
            color  = (0, 180, 255)

        cv2.putText(debug, status, (12, max(28, y0 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        return debug

    def _publish_debug_frame(self, stamp):
        if self.debug_pub is None or self.latest_debug is None:
            return
        ok, enc = cv2.imencode(".jpg", self.latest_debug, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data   = enc.tobytes()
        self.debug_pub.publish(msg)

    def _show_debug_frame(self):
        if not self.display_debug or self.latest_debug is None:
            return
        cv2.imshow("Row follower debug", self.latest_debug)
        cv2.waitKey(1)

    def stop_robot(self):
        self.latest_twist = Twist()
        if not self.dry_run:
            self.cmd_pub.publish(self.latest_twist)


def main(args=None):
    rclpy.init(args=args)
    node = RowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()