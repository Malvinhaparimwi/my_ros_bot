#!/usr/bin/env python3
from ultralytics import YOLO
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


@dataclass
class RowEstimate:
    center_x: float
    confidence: float
    box_xyxy: Tuple[float, float, float, float]
    cls_name: str = "plant"


# ── Full mission plan ─────────────────────────────────────────────────────────
#
#  row_index=0  follow row 1
#               end → advance 75 cm → turn RIGHT → advance 20 cm → turn RIGHT → row_index=1
#
#  row_index=1  follow row 2
#               end → advance 75 cm → turn LEFT  → advance 40 cm → turn LEFT  → row_index=2
#
#  row_index=2  follow row 3
#               end → advance 75 cm → DONE
#
# State machine labels (task_state):
#   following
#   advance_after_end          (first advance after a row ends, 75 cm for all rows)
#   reset_turn_heading         (wait + reset IMU before first inter-row turn)
#   turn_right / turn_left     (first inter-row turn)
#   advance_between_turns      (short advance between the two turns)
#   reset_turn_heading_second  (wait + reset IMU before second inter-row turn)
#   turn_right_second / turn_left_second  (second inter-row turn)
#   final_advance              (75 cm advance at end of last row before done)
#   done


class RowFollower(Node):
    def __init__(self):
        super().__init__("row_follower")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_topic",              "/camera/left/compressed"),
                ("cmd_vel_topic",            "/onyx/cmd_vel"),
                ("debug_topic",              "/move_base/row_debug/compressed"),
                ("enable_topic",             "/move_base/enable"),
                ("status_topic",             "/move_base/status"),
                ("row_end_topic",            "/move_base/row_end"),
                ("target_x_offset_ratio",    -0.01),
                ("publish_debug",            True),
                ("display_debug",            True),
                ("dry_run",                  False),
                ("process_every_n_frames",   1),
                ("roi_top_ratio",            0.38),
                ("roi_bottom_ratio",         0.98),
                ("detector_model_path",      ""),
                ("model_input_width",        1280),
                ("model_input_height",       736),
                ("model_threshold",          0.3),
                ("max_linear_speed",         0.055),
                ("search_linear_speed",      0.04),
                ("max_angular_speed",        0.06),
                ("steering_gain",            0.10),
                ("steering_deadband",        0.10),
                ("steering_smoothing",       0.06),
                ("left_turn_scale",          0.60),
                ("forward_right_bias",       0.010),
                ("imu_euler_topic",          "/imu/euler"),
                ("imu_turn_delta_topic",     "/imu/angle_swept"),
                ("imu_reset_service",        "/reset_imu"),
                ("imu_yaw_rate_gain",        0.08),
                ("imu_timeout_sec",          0.25),
                ("confidence_stop_threshold",0.08),
                ("lost_timeout_sec",         0.8),
                ("command_rate_hz",          10.0),
                ("jump_threshold",           0.25),
                ("realign_exit_threshold",   0.12),
                # Minimum angular speed enforced during realign so the robot
                # always physically moves — prevents stall when gain * error
                # is too small to overcome static friction.
                ("realign_min_angular_speed", 0.03),
                ("cmd_vel_timeout_sec",       0.50),
                ("end_sequence_enabled",     True),
                ("end_confirm_sec",          0.25),
                # Row-end advances
                ("end_advance_distance_m",          0.75),   # advance after each row end
                ("end_between_turns_distance_m",    0.17),   # right-turn corridor (row 1→2)
                ("end_between_turns_distance_left_m", 0.50), # left-turn corridor  (row 2→3)
                ("end_sequence_forward_speed",      0.1),
                # Wheel/advance calibration
                ("advance_wheel_diameter_m",            0.11),
                ("advance_seconds_per_wheel_rev",       12.0),
                ("advance_calibration_command_speed",   0.10),
                # IMU heading reset
                ("end_heading_reset_wait_sec",  3.0),
                # Turn parameters
                ("end_turn_speed",               0.07),
                ("end_right_turn_yaw_delta_deg", -80.0),
                ("end_left_turn_yaw_delta_deg",   85.0),
                ("end_turn_tolerance_deg",        2.0),
                ("end_turn_timeout_sec",          8.0),
                # How many degrees before the target to begin slowing (ramp-down zone).
                # Eliminates overshoot caused by the robot still spinning when
                # _end_turn_complete() first returns True.
                ("end_turn_slow_zone_deg",        8.0),
                # Reduced turn speed used inside the slow zone.
                ("end_turn_slow_speed",           0.035),
                # Separate (longer) freshness window for turn-delta readings so that
                # a brief processing hiccup does not flip us to the less-accurate
                # absolute-yaw fallback mid-turn.
                ("turn_imu_timeout_sec",          1.0),
            ],
        )

        self.image_topic             = self.get_parameter("image_topic").value
        self.cmd_vel_topic           = self.get_parameter("cmd_vel_topic").value
        self.debug_topic             = self.get_parameter("debug_topic").value
        self.enable_topic            = self.get_parameter("enable_topic").value
        self.status_topic            = self.get_parameter("status_topic").value
        self.row_end_topic           = self.get_parameter("row_end_topic").value
        self.publish_debug           = self.get_parameter("publish_debug").value
        self.display_debug           = self.get_parameter("display_debug").value
        self.dry_run                 = self.get_parameter("dry_run").value
        self.process_every_n_frames  = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.roi_top_ratio           = float(self.get_parameter("roi_top_ratio").value)
        self.roi_bottom_ratio        = float(self.get_parameter("roi_bottom_ratio").value)
        self.target_x_offset_ratio   = float(self.get_parameter("target_x_offset_ratio").value)
        self.detector_model_path     = self.get_parameter("detector_model_path").value
        self.model_input_width       = int(self.get_parameter("model_input_width").value)
        self.model_input_height      = int(self.get_parameter("model_input_height").value)
        self.model_threshold         = float(self.get_parameter("model_threshold").value)
        self.max_linear_speed        = float(self.get_parameter("max_linear_speed").value)
        self.search_linear_speed     = float(self.get_parameter("search_linear_speed").value)
        self.max_angular_speed       = float(self.get_parameter("max_angular_speed").value)
        self.steering_gain           = float(self.get_parameter("steering_gain").value)
        self.steering_deadband       = float(self.get_parameter("steering_deadband").value)
        self.steering_smoothing      = float(self.get_parameter("steering_smoothing").value)
        self.left_turn_scale         = float(self.get_parameter("left_turn_scale").value)
        self.forward_right_bias      = float(self.get_parameter("forward_right_bias").value)
        self.imu_euler_topic         = self.get_parameter("imu_euler_topic").value
        self.imu_turn_delta_topic    = self.get_parameter("imu_turn_delta_topic").value
        self.imu_reset_service       = self.get_parameter("imu_reset_service").value
        self.imu_yaw_rate_gain       = float(self.get_parameter("imu_yaw_rate_gain").value)
        self.imu_timeout_sec         = float(self.get_parameter("imu_timeout_sec").value)
        self.confidence_stop_threshold = float(self.get_parameter("confidence_stop_threshold").value)
        self.lost_timeout_sec          = float(self.get_parameter("lost_timeout_sec").value)
        self.jump_threshold            = float(self.get_parameter("jump_threshold").value)
        self.realign_exit_threshold    = float(self.get_parameter("realign_exit_threshold").value)
        self.realign_min_angular_speed = float(self.get_parameter("realign_min_angular_speed").value)
        self.cmd_vel_timeout_sec       = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.end_sequence_enabled      = bool(self.get_parameter("end_sequence_enabled").value)
        self.end_confirm_sec           = float(self.get_parameter("end_confirm_sec").value)
        self.end_advance_distance_m    = float(self.get_parameter("end_advance_distance_m").value)
        self.end_between_turns_distance_m = float(
            self.get_parameter("end_between_turns_distance_m").value
        )
        self.end_between_turns_distance_left_m = float(
            self.get_parameter("end_between_turns_distance_left_m").value
        )
        self.end_sequence_forward_speed = float(
            self.get_parameter("end_sequence_forward_speed").value
        )
        self.advance_wheel_diameter_m = float(
            self.get_parameter("advance_wheel_diameter_m").value
        )
        self.advance_seconds_per_wheel_rev = float(
            self.get_parameter("advance_seconds_per_wheel_rev").value
        )
        self.advance_calibration_command_speed = float(
            self.get_parameter("advance_calibration_command_speed").value
        )
        self.end_heading_reset_wait_sec = float(
            self.get_parameter("end_heading_reset_wait_sec").value
        )
        self.end_turn_speed               = float(self.get_parameter("end_turn_speed").value)
        self.end_right_turn_yaw_delta_deg = float(
            self.get_parameter("end_right_turn_yaw_delta_deg").value
        )
        self.end_left_turn_yaw_delta_deg  = float(
            self.get_parameter("end_left_turn_yaw_delta_deg").value
        )
        self.end_turn_tolerance_deg  = float(self.get_parameter("end_turn_tolerance_deg").value)
        self.end_turn_timeout_sec    = float(self.get_parameter("end_turn_timeout_sec").value)
        self.end_turn_slow_zone_deg  = float(self.get_parameter("end_turn_slow_zone_deg").value)
        self.end_turn_slow_speed     = float(self.get_parameter("end_turn_slow_speed").value)
        self.turn_imu_timeout_sec    = float(self.get_parameter("turn_imu_timeout_sec").value)

        self.class_names = ['plant']

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, qos
        )
        self.imu_subscription = self.create_subscription(
            Vector3Stamped, self.imu_euler_topic, self.imu_callback, 10
        )
        self.imu_turn_delta_subscription = self.create_subscription(
            Float32, self.imu_turn_delta_topic, self.imu_turn_delta_callback, 10
        )
        self.imu_reset_client = self.create_client(Trigger, self.imu_reset_service)
        self.enable_subscription = self.create_subscription(
            Bool, self.enable_topic, self.enable_callback, 10
        )
        self.cmd_pub   = self.create_publisher(Twist,  self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic,  10)
        self.row_end_pub = self.create_publisher(Bool,  self.row_end_topic, 10)
        self.debug_pub = (
            self.create_publisher(CompressedImage, self.debug_topic, 10)
            if self.publish_debug else None
        )

        self.frame_count    = 0
        self.latest_twist   = Twist()
        self.latest_debug   = None
        self.last_seen_time = None
        self.enabled        = True
        self.current_status = "lost"
        self.last_error     = 0.0
        self.realigning     = False
        self.realign_target_cx  = None
        self.last_imu_yaw_deg   = None
        self.last_imu_time      = None
        self.last_imu_turn_delta_deg  = None
        self.last_imu_turn_delta_time = None
        self.imu_yaw_rate = 0.0

        self.last_cx_px   = None
        self.twist_stamp  = None
        self.task_state   = "following"
        self.task_state_started = self.get_clock().now()
        self.row_lost_since = None
        self.turn_start_yaw_deg = None

        # Which row we are currently following (0-indexed).
        # 0 = first row, 1 = second row, 2 = third (last) row.
        self.row_index = 0

        # turn_count within the current inter-row transition (1 = first turn, 2 = second turn)
        self.turn_count = 0

        self.end_sequence_armed  = True
        self.imu_reset_requested = False
        self.imu_reset_completed = False
        self.imu_reset_future    = None
        self.imu_reset_warned    = False

        if self.display_debug:
            cv2.namedWindow("Row follower debug", cv2.WINDOW_NORMAL)
            self.get_logger().info("OpenCV debug window enabled")

        self.onnx_sess = self._load_model()

        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.command_timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(f"Following row from {self.image_topic}")
        self.get_logger().info(f"Publishing drive commands to {self.cmd_vel_topic}")
        self.get_logger().info(
            f"Move_base control topics: enable={self.enable_topic}, "
            f"status={self.status_topic}, row_end={self.row_end_topic}"
        )
        self.get_logger().info(
            f"IMU topics: euler={self.imu_euler_topic}, "
            f"turn_delta={self.imu_turn_delta_topic}, reset={self.imu_reset_service}"
        )
        self.get_logger().info(
            f"Steering limits: linear={self.max_linear_speed:.3f}, "
            f"angular={self.max_angular_speed:.3f}, gain={self.steering_gain:.3f}, "
            f"left_turn_scale={self.left_turn_scale:.2f}, "
            f"target_x_offset_ratio={self.target_x_offset_ratio:.3f}"
        )
        self.get_logger().info(
            "Mission: row0 → [adv75→R→adv20→R] → row1 → "
            "[adv75→L→adv40→L] → row2 → adv75 → DONE"
        )

    # ── model loading ─────────────────────────────────────────────────────────

    def _load_model(self):
        if not self.detector_model_path:
            raise RuntimeError("detector_model_path param is required")
        path = Path(self.detector_model_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Model not found: {path}")
        try:
            model = YOLO(str(path))
        except Exception as exc:
            raise RuntimeError(f"Failed to load YOLOv8 model: {exc}")
        self.get_logger().info(f"Loaded YOLOv8 detector: {path}")
        self.get_logger().info(f"Threshold={self.model_threshold}")
        return model

    # ── ROS callbacks ─────────────────────────────────────────────────────────

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

    def enable_callback(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.latest_twist = Twist()
            if not self.dry_run:
                self.cmd_pub.publish(self._to_microcontroller_twist(self.latest_twist))

    def imu_callback(self, msg: Vector3Stamped):
        now = self.get_clock().now()
        yaw_deg = float(msg.vector.z)
        if self.last_imu_yaw_deg is not None and self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            if dt > 0.0:
                yaw_delta = self._wrap_degrees(yaw_deg - self.last_imu_yaw_deg)
                self.imu_yaw_rate = np.deg2rad(yaw_delta) / dt
        self.last_imu_yaw_deg = yaw_deg
        self.last_imu_time    = now

    def imu_turn_delta_callback(self, msg: Float32):
        self.last_imu_turn_delta_deg  = float(msg.data)
        self.last_imu_turn_delta_time = self.get_clock().now()

    @staticmethod
    def _decode_frame(msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except cv2.error:
            return None

    @staticmethod
    def _wrap_degrees(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    # ── detection ─────────────────────────────────────────────────────────────

    def _estimate_row(
        self, frame: np.ndarray
    ) -> Tuple[Optional[RowEstimate], np.ndarray]:
        h, w = frame.shape[:2]
        y0 = int(np.clip(self.roi_top_ratio,    0.00, 0.95) * h)
        y1 = int(np.clip(self.roi_bottom_ratio, 0.05, 1.00) * h)
        if y1 <= y0:
            y0, y1 = int(0.4 * h), h

        roi        = frame[y0:y1, :]
        detections = self._run_detector(roi)

        debug = self._draw_debug(frame, y0, y1, detections, None)
        if not detections:
            return None, debug

        roi_h = roi.shape[0]
        roi_w = roi.shape[1]
        mid_x = self._target_x(roi_w)

        def _selection_score(det):
            x1, y1_d, x2, y2_d, score, _ = det
            cx          = (x1 + x2) / 2.0
            horiz_dist  = abs(cx - mid_x) / (roi_w / 2.0)
            vert_penalty = (roi_h - y2_d) / roi_h
            return horiz_dist + 0.7 * vert_penalty

        def _stable_score(det, target_cx):
            x1, y1_d, x2, y2_d, score, _ = det
            cx     = (x1 + x2) / 2.0
            x_dist = abs(cx - target_cx) / (roi_w / 2.0)
            vert_penalty = (roi_h - y2_d) / roi_h
            return x_dist + 0.7 * vert_penalty

        middle_best = min(detections, key=_selection_score)
        best        = middle_best

        if self.realigning and self.realign_target_cx is not None:
            best = min(detections, key=lambda det: _stable_score(det, self.realign_target_cx))
        elif self.last_cx_px is not None:
            stable_best = min(detections, key=lambda det: _stable_score(det, self.last_cx_px))
            middle_cx   = (middle_best[0] + middle_best[2]) / 2.0
            stable_cx   = (stable_best[0] + stable_best[2]) / 2.0
            middle_jump = abs(middle_cx - self.last_cx_px) / (roi_w / 2.0)
            stable_jump = abs(stable_cx - self.last_cx_px) / (roi_w / 2.0)
            if middle_jump > self.jump_threshold and stable_jump <= self.jump_threshold:
                best = stable_best

        x1, y1_b, x2, y2_b, score, cls_name = best
        cx_px = (x1 + x2) / 2.0

        estimate = RowEstimate(
            center_x  = cx_px,
            confidence = float(score),
            box_xyxy  = (x1, y1_b, x2, y2_b),
            cls_name  = cls_name,
        )
        debug = self._draw_debug(frame, y0, y1, detections, estimate)
        return estimate, debug

    def _run_detector(self, roi: np.ndarray):
        results = self.onnx_sess(roi, conf=self.model_threshold, verbose=False)
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf     = float(box.conf[0])
                cls_id   = int(box.cls[0])
                cls_name = self.onnx_sess.names[cls_id]
                detections.append((x1, y1, x2, y2, conf, cls_name))
        return detections

    # ── command update ────────────────────────────────────────────────────────

    def update_command(self, estimate: Optional[RowEstimate], width: int):
        now   = self.get_clock().now()
        twist = Twist()

        if self.task_state != "following":
            self._publish_status()
            return

        if estimate and estimate.confidence >= self.confidence_stop_threshold:
            self.last_seen_time    = now
            self.end_sequence_armed = True
            self.current_status    = "tracking"
            self.row_lost_since    = None

            half_w    = width / 2.0
            target_x  = self._target_x(width)
            detected_cx = estimate.center_x

            if self.last_cx_px is not None:
                jump_norm = abs(detected_cx - self.last_cx_px) / half_w
                if jump_norm > self.jump_threshold and not self.realigning:
                    self.realigning        = True
                    self.realign_target_cx = self.last_cx_px
                    self.get_logger().warning(
                        f"Green box jump rejected: jump={jump_norm:.2f}, "
                        f"target_cx={self.realign_target_cx:.1f}"
                    )

            if self.realigning:
                target_cx     = self.realign_target_cx or self.last_cx_px or half_w
                detected_error = (detected_cx - target_cx) / half_w
                if abs(detected_error) <= self.realign_exit_threshold:
                    self.realigning        = False
                    self.realign_target_cx = None
                    self.last_cx_px        = detected_cx
                    self.current_status    = "tracking"
                else:
                    self.current_status = "realigning"
                    self.last_error     = detected_error
                    raw_angular = float(np.clip(
                        -self.steering_gain * detected_error,
                        -self.max_angular_speed,
                        self.max_angular_speed,
                    ))
                    # Enforce minimum so the robot physically moves.
                    # gain * small_error can be far below the motor's stall threshold.
                    if raw_angular != 0.0:
                        min_speed = self.realign_min_angular_speed
                        if abs(raw_angular) < min_speed:
                            raw_angular = math.copysign(min_speed, raw_angular)
                    twist.linear.x  = 0.0
                    twist.angular.z = self._apply_turn_asymmetry(raw_angular)
                    self.latest_twist = twist
                    self.twist_stamp  = self.get_clock().now()
                    self._publish_status()
                    return
            else:
                self.last_cx_px = detected_cx

            raw_error = (estimate.center_x - target_x) / half_w
            error     = 0.25 * raw_error + 0.75 * self.last_error
            if abs(error) < self.steering_deadband:
                error = 0.0
            self.last_error = error

            speed_scale     = max(0.35, 1.0 - abs(error))
            twist.linear.x  = self.max_linear_speed * speed_scale
            target_angular  = float(np.clip(
                -self.steering_gain * error,
                -self.max_angular_speed,
                self.max_angular_speed,
            ))
            target_angular  = self._apply_imu_yaw_damping(target_angular, now)
            target_angular  = self._apply_turn_asymmetry(target_angular)
            target_angular  = self._apply_forward_right_bias(target_angular, twist.linear.x)
            smoothing       = float(np.clip(self.steering_smoothing, 0.0, 1.0))
            twist.angular.z = (
                smoothing * target_angular
                + (1.0 - smoothing) * self.latest_twist.angular.z
            )

        elif self._recently_saw_row(now):
            self.current_status = "lost_recent"
            self._mark_row_lost(now)
            self.realigning        = False
            self.realign_target_cx = None
            twist.linear.x  = self.search_linear_speed
            twist.angular.z = self._apply_forward_right_bias(0.0, twist.linear.x)
        else:
            self.current_status    = "lost"
            self._mark_row_lost(now)
            self.last_cx_px        = None
            self.last_error        = 0.0
            self.realigning        = False
            self.realign_target_cx = None

        if self._should_start_end_sequence(now):
            self.end_sequence_armed = False
            self.turn_count         = 0
            self._start_task_state("advance_after_end")
            twist = Twist()
            self.get_logger().info(
                f"Row {self.row_index} end confirmed; "
                f"advancing {self.end_advance_distance_m:.2f} m."
            )

        self.latest_twist = twist
        self.twist_stamp  = self.get_clock().now()
        self._publish_status()

    # ── steering helpers ──────────────────────────────────────────────────────

    def _apply_imu_yaw_damping(self, target_angular: float, now) -> float:
        if self.last_imu_time is None:
            return target_angular
        age_sec = (now - self.last_imu_time).nanoseconds / 1e9
        if age_sec > self.imu_timeout_sec:
            return target_angular
        damped = target_angular - self.imu_yaw_rate_gain * self.imu_yaw_rate
        return float(np.clip(damped, -self.max_angular_speed, self.max_angular_speed))

    def _recently_saw_row(self, now) -> bool:
        if self.last_seen_time is None:
            return False
        return (now - self.last_seen_time).nanoseconds / 1e9 <= self.lost_timeout_sec

    def _target_x(self, width: int) -> float:
        return (width / 2.0) + (self.target_x_offset_ratio * width)

    def _apply_turn_asymmetry(self, angular_z: float) -> float:
        if angular_z > 0.0:
            return angular_z * self.left_turn_scale
        return angular_z

    def _apply_forward_right_bias(self, angular_z: float, linear_x: float) -> float:
        if linear_x <= 0.0 or self.forward_right_bias <= 0.0:
            return angular_z
        biased = angular_z - self.forward_right_bias
        return float(np.clip(biased, -self.max_angular_speed, self.max_angular_speed))

    def _mark_row_lost(self, now):
        if not self.end_sequence_enabled or self.row_lost_since is not None:
            return
        self.row_lost_since = now

    def _should_start_end_sequence(self, now) -> bool:
        if (
            not self.end_sequence_enabled
            or not self.end_sequence_armed
            or self.task_state != "following"
        ):
            return False
        if self.row_lost_since is None:
            return False
        lost_age = (now - self.row_lost_since).nanoseconds / 1e9
        return lost_age >= self.end_confirm_sec

    # ── mission routing helpers ───────────────────────────────────────────────

    def _doing_right_turns(self) -> bool:
        """After row 0 we do right turns."""
        return self.row_index == 0

    def _doing_left_turns(self) -> bool:
        """After row 1 we do left turns."""
        return self.row_index == 1

    def _is_last_row(self) -> bool:
        """Row 2 is the last row — just advance then done."""
        return self.row_index == 2

    def _first_turn_state(self) -> str:
        return "turn_right" if self._doing_right_turns() else "turn_left"

    def _second_turn_state(self) -> str:
        return "turn_right_second" if self._doing_right_turns() else "turn_left_second"

    def _between_turns_distance(self) -> float:
        if self._doing_right_turns():
            return self.end_between_turns_distance_m          # 20 cm
        return self.end_between_turns_distance_left_m          # 40 cm

    def _target_yaw_delta_deg(self) -> float:
        if self._doing_right_turns():
            return self.end_right_turn_yaw_delta_deg           # negative (e.g. -39)
        return self.end_left_turn_yaw_delta_deg                # positive (e.g. +39)

    def _turn_angular_z(self) -> float:
        speed = abs(self.end_turn_speed)
        return -speed if self._doing_right_turns() else speed  # right = negative

    # ── task state management ─────────────────────────────────────────────────

    def _start_task_state(self, state: str):
        self.task_state         = state
        self.task_state_started = self.get_clock().now()
        self.twist_stamp        = self.task_state_started
        if state.startswith("reset_turn_heading"):
            self.imu_reset_requested = False
            self.imu_reset_completed = False
            self.imu_reset_future    = None
            self.imu_reset_warned    = False

    def _task_state_elapsed_sec(self) -> float:
        if self.task_state_started is None:
            self.task_state_started = self.get_clock().now()
            return 0.0
        return (self.get_clock().now() - self.task_state_started).nanoseconds / 1e9

    # ── end-sequence state machine ────────────────────────────────────────────

    def _update_end_sequence_command(self) -> bool:
        """
        Called every timer tick when task_state != 'following'.
        Returns True if it handled this tick (caller should not apply row-follow logic).
        """
        if self.task_state == "following":
            return False

        twist   = Twist()
        elapsed = self._task_state_elapsed_sec()

        # ── 1. advance_after_end (75 cm for every row) ───────────────────────
        if self.task_state == "advance_after_end":
            distance_m = self._advance_speed_mps() * elapsed
            self.current_status = "advance_after_end"
            if distance_m >= self.end_advance_distance_m:
                self.latest_twist = Twist()
                self.twist_stamp  = self.get_clock().now()
                if self._is_last_row():
                    # Row 2 done — mission complete
                    self._start_task_state("done")
                    self.get_logger().info(
                        f"Final advance complete ({distance_m:.2f} m); mission DONE."
                    )
                else:
                    self.turn_start_yaw_deg = None
                    self._start_task_state("reset_turn_heading")
                    self.get_logger().info(
                        f"Advance complete ({distance_m:.2f} m); resetting heading."
                    )
                return True
            twist.linear.x  = self.end_sequence_forward_speed
            twist.angular.z = self._apply_forward_right_bias(0.0, twist.linear.x)
            self.latest_twist = twist
            self.twist_stamp  = self.get_clock().now()
            return True

        # ── 2. reset_turn_heading / reset_turn_heading_second ────────────────
        if self.task_state in ("reset_turn_heading", "reset_turn_heading_second"):
            self.current_status = "reset_turn_heading"
            self.latest_twist   = Twist()
            self.twist_stamp    = self.get_clock().now()
            self._request_imu_heading_reset()
            self._update_imu_heading_reset()
            imu_ready         = self.imu_reset_completed and self._turn_reference_is_fresh()
            reset_timeout_sec = max(1.0, self.end_heading_reset_wait_sec + 0.75)
            reset_timed_out   = elapsed >= reset_timeout_sec
            if elapsed >= self.end_heading_reset_wait_sec and (imu_ready or reset_timed_out):
                self.turn_start_yaw_deg = (
                    self.last_imu_yaw_deg if self._imu_heading_is_fresh() else None
                )
                next_turn = (
                    self._second_turn_state()
                    if self.task_state == "reset_turn_heading_second"
                    else self._first_turn_state()
                )
                self._start_task_state(next_turn)
                if imu_ready:
                    self.get_logger().info(
                        f"Turn reference ready at yaw_delta="
                        f"{self._current_turn_delta_deg():.1f} deg; "
                        f"executing {next_turn}."
                    )
                else:
                    self.get_logger().warning(
                        "Timed out waiting for zeroed IMU; turning with timeout fallback."
                    )
            return True

        # ── 3. turn_right / turn_left / turn_right_second / turn_left_second ─
        if self.task_state in (
            "turn_right", "turn_left", "turn_right_second", "turn_left_second"
        ):
            self.current_status   = self.task_state
            heading_available     = self._turn_heading_available()
            timeout_without_heading = (
                not heading_available and elapsed >= self.end_turn_timeout_sec
            )
            if self._end_turn_complete() or timeout_without_heading:
                self.latest_twist = Twist()
                self.twist_stamp  = self.get_clock().now()
                turn_delta        = self._current_turn_delta_deg()
                self.turn_count  += 1
                if self.turn_count == 1:
                    # First turn done → advance between turns
                    self._start_task_state("advance_between_turns")
                    self.get_logger().info(
                        f"First turn complete at yaw_delta={turn_delta:.1f} deg; "
                        f"advancing {self._between_turns_distance():.2f} m."
                    )
                else:
                    # Second turn done → start next row
                    self.row_index += 1
                    self._resume_row_following()
                    self.get_logger().info(
                        f"Second turn complete at yaw_delta={turn_delta:.1f} deg; "
                        f"now following row {self.row_index}."
                    )
                return True

            # ── Slow-zone ramp-down to prevent overshoot ──────────────────
            # Switch to a reduced speed when within end_turn_slow_zone_deg of
            # the target so the robot is already moving slowly when the stop
            # condition fires, dramatically reducing overshoot.
            yaw_delta  = self._current_turn_delta_deg()
            target     = self._target_yaw_delta_deg()
            remaining  = abs(self._wrap_degrees(yaw_delta - target))
            in_slow_zone = heading_available and remaining <= self.end_turn_slow_zone_deg
            base_speed = self.end_turn_slow_speed if in_slow_zone else self.end_turn_speed
            twist.angular.z   = (-abs(base_speed) if self._doing_right_turns()
                                 else abs(base_speed))
            self.latest_twist = twist
            self.twist_stamp  = self.get_clock().now()
            return True

        # ── 4. advance_between_turns ──────────────────────────────────────────
        if self.task_state == "advance_between_turns":
            distance_m = self._advance_speed_mps() * elapsed
            self.current_status = "advance_between_turns"
            target_dist = self._between_turns_distance()
            if distance_m >= target_dist:
                self.turn_start_yaw_deg = None
                self._start_task_state("reset_turn_heading_second")
                self.latest_twist = Twist()
                self.twist_stamp  = self.get_clock().now()
                self.get_logger().info(
                    f"Between-turns advance complete ({distance_m:.2f} m); "
                    "resetting heading for second turn."
                )
                return True
            twist.linear.x  = self.end_sequence_forward_speed
            twist.angular.z = self._apply_forward_right_bias(0.0, twist.linear.x)
            self.latest_twist = twist
            self.twist_stamp  = self.get_clock().now()
            return True

        # ── 5. done ───────────────────────────────────────────────────────────
        if self.task_state == "done":
            self.current_status = "done"
            self.latest_twist   = Twist()
            self.twist_stamp    = self.get_clock().now()
            return True

        return False

    def _resume_row_following(self):
        self.task_state         = "following"
        self.task_state_started = self.get_clock().now()
        self.current_status     = "lost"
        self.latest_twist       = Twist()
        self.twist_stamp        = self.get_clock().now()
        self.row_lost_since     = None
        self.last_seen_time     = None
        self.last_cx_px         = None
        self.last_error         = 0.0
        self.realigning         = False
        self.realign_target_cx  = None
        self.turn_start_yaw_deg = None
        self.turn_count         = 0
        self.end_sequence_armed = False

    # ── IMU reset ─────────────────────────────────────────────────────────────

    def _request_imu_heading_reset(self):
        if self.imu_reset_requested:
            return
        self.imu_reset_requested = True
        if not self.imu_reset_client.service_is_ready():
            if not self.imu_reset_client.wait_for_service(timeout_sec=0.0):
                if not self.imu_reset_warned:
                    self.get_logger().warning(
                        f"IMU reset service {self.imu_reset_service} not available; "
                        "using current heading as turn zero."
                    )
                    self.imu_reset_warned    = True
                self.imu_reset_completed = True
                return
        self.get_logger().info("Resetting IMU heading to zero before turn.")
        self.imu_reset_future = self.imu_reset_client.call_async(Trigger.Request())

    def _update_imu_heading_reset(self):
        if self.imu_reset_completed or self.imu_reset_future is None:
            return
        if not self.imu_reset_future.done():
            return
        try:
            response = self.imu_reset_future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"IMU reset service call failed: {exc}; using current heading as turn zero."
            )
            self.imu_reset_completed = True
            self.imu_reset_future    = None
            return

        if response.success:
            self.last_imu_yaw_deg         = None
            self.last_imu_time            = None
            self.last_imu_turn_delta_deg  = None
            self.last_imu_turn_delta_time = None
            self.imu_yaw_rate             = 0.0
            self.get_logger().info("IMU heading reset complete; waiting for turn delta.")
        else:
            self.get_logger().warning(
                f"IMU reset rejected: {response.message}; using current heading."
            )
        self.imu_reset_completed = True
        self.imu_reset_future    = None

    # ── advance speed helper ──────────────────────────────────────────────────

    def _advance_speed_mps(self) -> float:
        seconds_per_rev      = self.advance_seconds_per_wheel_rev
        wheel_diameter_m     = self.advance_wheel_diameter_m
        calibration_command  = self.advance_calibration_command_speed
        if seconds_per_rev <= 0.0:
            return abs(self.end_sequence_forward_speed)
        speed_scale = (
            abs(self.end_sequence_forward_speed) / calibration_command
            if calibration_command > 0.0 else 1.0
        )
        wheel_circumference_m = np.pi * wheel_diameter_m
        calibrated_speed_mps  = wheel_circumference_m / seconds_per_rev
        return calibrated_speed_mps * speed_scale

    # ── turn completion helpers ───────────────────────────────────────────────

    def _end_turn_complete(self) -> bool:
        if not self._turn_heading_available():
            return False
        yaw_delta = self._current_turn_delta_deg()
        target    = self._target_yaw_delta_deg()
        error     = self._wrap_degrees(yaw_delta - target)
        return abs(error) <= self.end_turn_tolerance_deg

    def _turn_heading_available(self) -> bool:
        return self._turn_delta_is_fresh() or (
            self.turn_start_yaw_deg is not None and self.last_imu_yaw_deg is not None
        )

    def _turn_delta_is_fresh(self) -> bool:
        if self.last_imu_turn_delta_time is None or self.last_imu_turn_delta_deg is None:
            return False
        age_sec = (
            self.get_clock().now() - self.last_imu_turn_delta_time
        ).nanoseconds / 1e9
        # Use the longer turn_imu_timeout_sec (default 1.0 s) here — imu_timeout_sec
        # (0.25 s) is intentionally tight for damping during row-following but causes
        # spurious fallbacks to the less-accurate absolute-yaw path mid-turn.
        return age_sec <= self.turn_imu_timeout_sec

    def _turn_reference_is_fresh(self) -> bool:
        return self._turn_delta_is_fresh() or self._imu_heading_is_fresh()

    def _imu_heading_is_fresh(self) -> bool:
        if self.last_imu_time is None or self.last_imu_yaw_deg is None:
            return False
        age_sec = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
        return age_sec <= self.imu_timeout_sec

    def _current_turn_delta_deg(self) -> float:
        if self._turn_delta_is_fresh():
            return self.last_imu_turn_delta_deg
        if self.turn_start_yaw_deg is None or self.last_imu_yaw_deg is None:
            return 0.0
        return self._wrap_degrees(self.last_imu_yaw_deg - self.turn_start_yaw_deg)

    # ── publish command timer ─────────────────────────────────────────────────

    def publish_command(self):
        self._publish_status()
        if self.dry_run or not self.enabled:
            return

        if self._update_end_sequence_command():
            self._publish_status()
            self.cmd_pub.publish(self._to_microcontroller_twist(self.latest_twist))
            return

        if self.twist_stamp is not None:
            age_sec = (self.get_clock().now() - self.twist_stamp).nanoseconds / 1e9
            if age_sec > self.cmd_vel_timeout_sec:
                self.cmd_pub.publish(self._to_microcontroller_twist(Twist()))
                return

        self.cmd_pub.publish(self._to_microcontroller_twist(self.latest_twist))

    def _publish_status(self):
        msg      = String()
        msg.data = self.current_status
        self.status_pub.publish(msg)
        row_end_msg      = Bool()
        row_end_msg.data = self.current_status in ("lost_recent", "lost")
        self.row_end_pub.publish(row_end_msg)

    @staticmethod
    def _to_microcontroller_twist(twist: Twist) -> Twist:
        converted            = Twist()
        converted.linear.x   = -twist.angular.z
        converted.angular.z  = -twist.linear.x
        return converted

    # ── debug visuals ─────────────────────────────────────────────────────────

    def _draw_debug(self, frame, y0, y1, detections, estimate):
        debug = frame.copy()
        roi_w = frame.shape[1]
        h     = frame.shape[0]

        mid = int(self._target_x(roi_w))
        cv2.line(debug, (mid, 0), (mid, h), (0, 0, 0), 7)
        cv2.line(debug, (mid, 0), (mid, h), (255, 255, 255), 4)
        cv2.putText(debug, "TARGET", (min(mid + 8, roi_w - 95), 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(debug, "TARGET", (min(mid + 8, roi_w - 95), 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

        for x1, y1_det, x2, y2_det, conf, cls_name in detections:
            fy1   = y0 + y1_det
            fy2   = y0 + y2_det
            color = (0, 0, 255)
            cv2.rectangle(debug, (x1, fy1), (x2, fy2), color, 2)
            label_text = f"{cls_name} {conf:.2f}"
            (tw, th), baseline = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
            )
            cv2.rectangle(debug, (x1, fy1 - th - baseline - 4), (x1 + tw, fy1), color, -1)
            cv2.putText(debug, label_text, (x1, fy1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        if estimate:
            x1, fy1_e, x2, fy2_e = estimate.box_xyxy
            fy1_e = y0 + int(fy1_e)
            fy2_e = y0 + int(fy2_e)
            cv2.rectangle(debug, (int(x1), fy1_e), (int(x2), fy2_e), (0, 255, 0), 3)
            centroid_x = int((float(x1) + float(x2)) / 2.0)
            centroid_y = int((fy1_e + fy2_e) / 2.0)
            cv2.circle(debug, (centroid_x, centroid_y), 6,  (0, 255, 0), -1, cv2.LINE_AA)
            cv2.circle(debug, (centroid_x, centroid_y), 10, (0, 255, 0),  2, cv2.LINE_AA)

            if self.realigning and self.realign_target_cx is not None:
                target_x = int(self.realign_target_cx)
                cv2.line(debug, (target_x, 0), (target_x, h), (0, 200, 255), 2)
                status = (f"REALIGN target_x={target_x} "
                          f"centroid=({centroid_x},{centroid_y}) "
                          f"err={self.last_error:.2f} "
                          f"boxes={len(detections)}")
                color = (0, 200, 255)
            else:
                status = (f"det conf={estimate.confidence:.2f} "
                          f"centroid=({centroid_x},{centroid_y}) "
                          f"err={self.last_error:.2f} "
                          f"boxes={len(detections)}")
                color = (0, 255, 0)
        else:
            status = f"det no row  boxes={len(detections)}"
            color  = (0, 180, 255)

        cv2.putText(debug, status, (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

        if self.twist_stamp is not None:
            age_ms = (self.get_clock().now() - self.twist_stamp).nanoseconds / 1e6
            stale  = age_ms > self.cmd_vel_timeout_sec * 1000
            age_color  = (0, 0, 255) if stale else (200, 200, 200)
            vel_status = (
                f"lin={self.latest_twist.linear.x:.3f}  "
                f"ang={self.latest_twist.angular.z:.3f}  "
                f"yaw_rate={self.imu_yaw_rate:.2f}  "
                f"age={age_ms:.0f}ms{'  STALE→ZERO' if stale else ''}"
            )
            cv2.putText(debug, vel_status, (12, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, age_color, 1, cv2.LINE_AA)

        task_status = self._debug_task_status()
        cv2.putText(debug, task_status, (12, 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(debug, task_status, (12, 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)

        return debug

    def _debug_task_status(self) -> str:
        elapsed   = self._task_state_elapsed_sec()
        direction = "RIGHT" if self._doing_right_turns() else "LEFT"
        if self.task_state == "advance_after_end":
            distance_m = self._advance_speed_mps() * elapsed
            return (
                f"row={self.row_index} task=advance "
                f"dist={distance_m:.3f}/{self.end_advance_distance_m:.3f}m"
            )
        if self.task_state == "advance_between_turns":
            distance_m  = self._advance_speed_mps() * elapsed
            target_dist = self._between_turns_distance()
            return (
                f"row={self.row_index} task=advance_between({direction}) "
                f"dist={distance_m:.3f}/{target_dist:.3f}m"
            )
        if self.task_state in (
            "turn_right", "turn_left", "turn_right_second", "turn_left_second"
        ):
            turn_delta = self._current_turn_delta_deg()
            target     = self._target_yaw_delta_deg()
            return (
                f"row={self.row_index} task={self.task_state} "
                f"yaw={turn_delta:.1f}/{target:.1f}deg"
            )
        return f"row={self.row_index} task={self.task_state}"

    def _publish_debug_frame(self, stamp):
        if self.debug_pub is None or self.latest_debug is None:
            return
        ok, enc = cv2.imencode(".jpg", self.latest_debug, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg            = CompressedImage()
        msg.header.stamp = stamp
        msg.format     = "jpeg"
        msg.data       = enc.tobytes()
        self.debug_pub.publish(msg)

    def _show_debug_frame(self):
        if not self.display_debug or self.latest_debug is None:
            return
        cv2.imshow("Row follower debug", self.latest_debug)
        cv2.waitKey(1)

    def stop_robot(self):
        self.latest_twist = Twist()
        if not self.dry_run:
            self.cmd_pub.publish(self._to_microcontroller_twist(self.latest_twist))


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