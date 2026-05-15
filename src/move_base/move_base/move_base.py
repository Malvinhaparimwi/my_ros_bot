#!/usr/bin/env python3
"""
ROS 2 Jazzy node: subscribes to /camera/left/compressed and displays the feed.
Dependencies: rclpy, sensor_msgs, opencv-python, numpy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__("camera_left_viewer")

        self.declare_parameter("topic", "/camera/left/compressed")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        # Best-effort QoS matches most camera drivers; change to RELIABLE if needed
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            qos,
        )

        self.window_name = "Left Camera"
        self.latest_frame = None
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Display timer at ~30 fps
        self.timer = self.create_timer(1.0 / 30.0, self.display_frame)

        self.get_logger().info(f"Subscribed to: {topic}")
        self.get_logger().info("Press 'q' or Escape in the image window to quit.")

    def image_callback(self, msg: CompressedImage):
        """Decode CompressedImage bytes directly — no cv_bridge needed."""
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.latest_frame = frame
            else:
                self.get_logger().warn(
                    "imdecode returned None – check image encoding.",
                    throttle_duration_sec=5,
                )
        except Exception as e:
            self.get_logger().error(
                f"Failed to decode image: {e}",
                throttle_duration_sec=5,
            )

    def display_frame(self):
        """Timer callback: render latest frame and check for quit key."""
        if self.latest_frame is not None:
            cv2.imshow(self.window_name, self.latest_frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # 'q' or Escape
            self.get_logger().info("Quit requested – shutting down.")
            cv2.destroyAllWindows()
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()