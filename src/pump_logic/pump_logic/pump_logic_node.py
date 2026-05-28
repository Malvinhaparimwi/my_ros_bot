#!/usr/bin/env python3
import math
import sys
from pathlib import Path
from typing import Optional, Tuple, Dict, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO


class CentroidTracker:
    """
    A lightweight, pure-numpy centroid tracker for tracking plants across frames.
    Assigns stable IDs and manages occlusions/disappearances gracefully.
    """
    def __init__(self, max_disappeared: int = 10, max_distance: float = 100.0):
        self.next_id = 0
        self.objects: Dict[int, np.ndarray] = {}  # id -> centroid (cx, cy)
        self.disappeared: Dict[int, int] = {}    # id -> disappeared frame count
        self.sprayed: Dict[int, bool] = {}       # id -> whether it has been sprayed
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, centroid: np.ndarray) -> int:
        object_id = self.next_id
        self.objects[object_id] = centroid
        self.disappeared[object_id] = 0
        self.sprayed[object_id] = False
        self.next_id += 1
        return object_id

    def deregister(self, object_id: int):
        if object_id in self.objects:
            del self.objects[object_id]
        if object_id in self.disappeared:
            del self.disappeared[object_id]
        if object_id in self.sprayed:
            del self.sprayed[object_id]

    def update(self, rects: List[Tuple[int, int, int, int]]) -> Dict[int, np.ndarray]:
        """
        Updates tracked objects based on newly detected bounding box rectangles.
        """
        if len(rects) == 0:
            # Increment disappeared counter for all current tracks
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            return self.objects

        # Compute centroids for all input detections
        input_centroids = np.zeros((len(rects), 2), dtype="int")
        for i, (x1, y1, x2, y2) in enumerate(rects):
            cx = int((x1 + x2) / 2.0)
            cy = int((y1 + y2) / 2.0)
            input_centroids[i] = (cx, cy)

        # If we have no active tracks, register all input centroids
        if len(self.objects) == 0:
            for i in range(len(input_centroids)):
                self.register(input_centroids[i])
        else:
            object_ids = list(self.objects.keys())
            object_centroids = np.array(list(self.objects.values()))

            # Compute Euclidean distance using pure numpy matrix operations
            # Shape: (num_existing_objects, num_input_detections)
            D = np.linalg.norm(object_centroids[:, np.newaxis] - input_centroids, axis=2)

            # Match existing objects to input detections
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            used_rows = set()
            used_cols = set()

            for row, col in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                if D[row, col] > self.max_distance:
                    continue

                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0

                used_rows.add(row)
                used_cols.add(col)

            unused_rows = set(range(0, D.shape[0])).difference(used_rows)
            unused_cols = set(range(0, D.shape[1])).difference(used_cols)

            # If there are more/equal active tracks than detections, some might have disappeared
            if D.shape[0] >= D.shape[1]:
                for row in unused_rows:
                    object_id = object_ids[row]
                    self.disappeared[object_id] += 1
                    if self.disappeared[object_id] > self.max_disappeared:
                        self.deregister(object_id)
            # If there are more detections than active tracks, register them as new
            else:
                for col in unused_cols:
                    self.register(input_centroids[col])

        return self.objects


class PumpLogicNode(Node):
    """
    ROS 2 node that runs plant detection, tracks them, and manages pump control commands.
    """
    def __init__(self):
        super().__init__('pump_logic')

        # Find absolute path of the default model in the same package folder
        default_model_path = str(Path(__file__).resolve().parent / 'side_yolov8n_best.onnx')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/right/compressed'), # USB side camera
                ('pump_topic', '/pump/controller'),
                ('debug_topic', '/pump/debug/compressed'),
                ('model_path', default_model_path),
                ('model_threshold', 0.50),
                ('max_disappeared', 10),     # frames allowed to disappear
                ('max_distance', 120.0),     # max pixel movement allowed per frame
                ('trigger_mode', 'immediate'), # 'immediate' or 'cross_line'
                ('trigger_x_ratio', 0.50),    # center line of camera image
                ('direction_of_motion', 'either'), # 'left_to_right', 'right_to_left', or 'either'
                ('spray_duration', 4.0),     # seconds to spray a plant
                ('publish_debug', True),
                ('display_debug', False),
                ('process_every_n_frames', 4),
                ('model_input_size', 320),
                ('device', 'cpu'),
                ('trigger_delay', 0.0),
            ]
        )

        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').value
        self.pump_topic = self.get_parameter('pump_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.model_threshold = float(self.get_parameter('model_threshold').value)
        self.max_disappeared = int(self.get_parameter('max_disappeared').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        self.trigger_mode = self.get_parameter('trigger_mode').value
        self.trigger_x_ratio = float(self.get_parameter('trigger_x_ratio').value)
        self.direction_of_motion = self.get_parameter('direction_of_motion').value
        self.spray_duration = float(self.get_parameter('spray_duration').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.display_debug = bool(self.get_parameter('display_debug').value)
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.model_input_size = int(self.get_parameter('model_input_size').value)
        self.device = self.get_parameter('device').value
        self.trigger_delay = float(self.get_parameter('trigger_delay').value)

        # Initialize core components
        self.tracker = CentroidTracker(
            max_disappeared=self.max_disappeared,
            max_distance=self.max_distance
        )
        self.detector = self._load_detector()

        # State tracking variables
        self.frame_count = 0
        self.spray_until: Optional[rclpy.time.Time] = None
        self.spray_triggered_at: Dict[int, rclpy.time.Time] = {}
        self.pump_active = False
        self.pending_triggers = []

        # QoS for compressed camera feed
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions & Publishers
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        self.pump_pub = self.create_publisher(String, self.pump_topic, 10)
        self.debug_pub = (
            self.create_publisher(CompressedImage, self.debug_topic, 10)
            if self.publish_debug else None
        )

        # Pump watchdog timer (runs at 10 Hz to shut off pump when expired)
        self.pump_timer = self.create_timer(0.10, self.check_pump_watchdog)

        self.get_logger().info('===============================================')
        self.get_logger().info('Pump Logic Node initialized successfully')
        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')
        self.get_logger().info(f'Publishing commands to: {self.pump_topic}')
        self.get_logger().info(f'Detector model: {self.model_path}')
        self.get_logger().info(f'Trigger mode: {self.trigger_mode} (spray duration: {self.spray_duration}s, delay: {self.trigger_delay}s)')
        self.get_logger().info(f'Optimizations: process every {self.process_every_n_frames} frame(s), '
                               f'YOLO size {self.model_input_size}, device {self.device}')
        self.get_logger().info('===============================================')

    def _load_detector(self) -> YOLO:
        path = Path(self.model_path).expanduser()
        if not path.exists():
            self.get_logger().error(f'YOLO model file not found: {path}')
            raise FileNotFoundError(f'Model not found: {path}')
        try:
            model = YOLO(str(path), task='detect')
            self.get_logger().info(f'YOLOv8 Model loaded successfully from: {path}')
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise RuntimeError(e)

    def image_callback(self, msg: CompressedImage):
        """
        Receives side camera frames, runs YOLO, tracks plants, and triggers spray events.
        """
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        frame = self._decode_frame(msg)
        if frame is None:
            return

        h, w = frame.shape[:2]

        # Run YOLO detection
        results = self.detector(
            frame,
            conf=self.model_threshold,
            verbose=False,
            imgsz=self.model_input_size,
            device=self.device
        )
        rects = []
        for result in results:
            for box in result.boxes:
                # Store bounding box rectangles
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                rects.append((x1, y1, x2, y2))

        # Update centroid tracker
        active_objects = self.tracker.update(rects)

        # Process each tracked plant and check for trigger conditions
        trigger_x = int(self.trigger_x_ratio * w)
        now = self.get_clock().now()

        for object_id, centroid in active_objects.items():
            cx, cy = centroid

            # Check if plant has already been sprayed
            if self.tracker.sprayed[object_id]:
                continue

            should_trigger = False

            if self.trigger_mode == 'immediate':
                # Trigger as soon as the plant is detected
                should_trigger = True
            elif self.trigger_mode == 'cross_line':
                # Trigger when the plant crosses the trigger_x line depending on direction
                if self.direction_of_motion == 'left_to_right' and cx >= trigger_x:
                    should_trigger = True
                elif self.direction_of_motion == 'right_to_left' and cx <= trigger_x:
                    should_trigger = True
                elif self.direction_of_motion == 'either':
                    should_trigger = True

            if should_trigger:
                self.tracker.sprayed[object_id] = True
                if self.trigger_delay > 0.0:
                    trigger_time = now + Duration(seconds=self.trigger_delay)
                    self.pending_triggers.append((trigger_time, object_id))
                    self.get_logger().info(f"Scheduled spray trigger for plant ID {object_id} in {self.trigger_delay}s...")
                else:
                    self.trigger_spray(now, object_id)

        # Publish visual debug frame
        if self.publish_debug and self.debug_pub:
            self.publish_debug_frame(frame, rects, active_objects, trigger_x)

    def trigger_spray(self, now: rclpy.time.Time, object_id: int):
        """
        Starts or extends a spray cycle.
        """
        duration = Duration(seconds=self.spray_duration)
        target_end_time = now + duration

        self.spray_triggered_at[object_id] = now

        if self.spray_until is None:
            self.spray_until = target_end_time
            self.set_pump(True)
            self.get_logger().info(f'Sprayer ON! Spraying for {self.spray_duration}s...')
        else:
            # Extend spraying duration if a new plant is detected
            if target_end_time > self.spray_until:
                self.spray_until = target_end_time
                self.get_logger().info('Sprayer extended! Added new plant to cycle.')

    def check_pump_watchdog(self):
        """
        Timer callback that stops the pump once the spray duration has expired.
        It also processes and activates any delayed/pending spray triggers.
        """
        now = self.get_clock().now()

        # Process pending triggers
        if len(self.pending_triggers) > 0:
            still_pending = []
            for trigger_time, object_id in self.pending_triggers:
                if now >= trigger_time:
                    self.trigger_spray(now, object_id)
                else:
                    still_pending.append((trigger_time, object_id))
            self.pending_triggers = still_pending

        # Watchdog to stop the pump when expired
        if self.spray_until is not None and now >= self.spray_until:
            self.set_pump(False)
            self.spray_until = None
            self.get_logger().info('Sprayer OFF. Cycle complete.')

    def set_pump(self, active: bool):
        """
        Sends standard String commands to the pump controller topic.
        """
        self.pump_active = active
        msg = String()
        msg.data = 'on' if active else 'off'
        self.pump_pub.publish(msg)

    def publish_debug_frame(self, frame: np.ndarray, rects: list, active_objects: dict, trigger_x: int):
        """
        Overlays tracking markers, trigger line, and statuses, then publishes the debug stream.
        """
        h, w = frame.shape[:2]
        debug_frame = frame.copy()

        # Draw trigger line if in cross_line mode
        if self.trigger_mode == 'cross_line':
            cv2.line(debug_frame, (trigger_x, 0), (trigger_x, h), (0, 0, 255), 2)
            cv2.putText(
                debug_frame, 'TRIGGER LINE', (trigger_x + 8, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )

        # Draw bounding boxes
        now = self.get_clock().now()
        for x1, y1, x2, y2 in rects:
            cx = int((x1 + x2) / 2.0)
            cy = int((y1 + y2) / 2.0)

            # Match bounding box to the closest active tracked object
            matched_id = None
            min_dist = float('inf')
            for object_id, centroid in active_objects.items():
                dist = math.hypot(cx - centroid[0], cy - centroid[1])
                if dist < min_dist and dist < 60.0:
                    min_dist = dist
                    matched_id = object_id

            # Draw red if matched plant is currently being sprayed
            is_being_sprayed = False
            if matched_id is not None and matched_id in self.spray_triggered_at:
                elapsed = (now - self.spray_triggered_at[matched_id]).nanoseconds / 1e9
                is_being_sprayed = elapsed < self.spray_duration and self.pump_active

            color = (0, 0, 255) if is_being_sprayed else (0, 255, 0)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)

        # Draw tracking dots and text
        for object_id, centroid in active_objects.items():
            cx, cy = centroid
            is_sprayed = self.tracker.sprayed[object_id]

            # Choose color: Green = active track, Blue = spraying / sprayed
            color = (255, 0, 0) if is_sprayed else (0, 255, 0)
            status_text = 'SPRAYED' if is_sprayed else 'TRACKING'

            cv2.circle(debug_frame, (cx, cy), 6, color, -1)
            cv2.putText(
                debug_frame, f'ID {object_id}: {status_text}', (cx - 30, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

        # Draw overall Spraying Status box at the top center
        status_bg_color = (0, 165, 255) if self.pump_active else (50, 50, 50)
        status_txt_color = (255, 255, 255)
        status_label = 'SPRAYING: ACTIVE' if self.pump_active else 'SPRAYING: READY'

        cv2.rectangle(debug_frame, (w // 2 - 140, 10), (w // 2 + 140, 45), status_bg_color, -1)
        cv2.putText(
            debug_frame, status_label, (w // 2 - 110, 33),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_txt_color, 2
        )

        # Encode and publish
        success, encoded_img = cv2.imencode('.jpg', debug_frame)
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = encoded_img.tobytes()
            self.debug_pub.publish(msg)

        # Show cv2 window if locally enabled
        if self.display_debug:
            cv2.imshow('Pump Logic Debug', debug_frame)
            cv2.waitKey(1)

    @staticmethod
    def _decode_frame(msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except cv2.error:
            return None

    def destroy_node(self):
        # Shut off the pump on node shutdown for safety
        self.set_pump(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PumpLogicNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Pump Logic node failed: {e}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
