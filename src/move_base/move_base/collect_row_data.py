#!/usr/bin/env python3
"""Save frames from a compressed camera topic for row-following tuning."""

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class RowDataCollector(Node):
    def __init__(self):
        super().__init__("row_data_collector")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_topic", "/camera/left/compressed"),
                ("output_dir", "front_camera/raw"),
                ("save_every_n_frames", 10),
                ("display", True),
            ],
        )

        self.image_topic = self.get_parameter("image_topic").value
        self.output_dir = Path(
            self.get_parameter("output_dir").value
        ).expanduser()
        self.save_every_n_frames = max(
            1,
            int(self.get_parameter("save_every_n_frames").value),
        )
        self.display = bool(self.get_parameter("display").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos,
        )

        self.frame_count = 0
        self.saved_count = 0
        self.get_logger().info(f"Collecting frames from {self.image_topic}")
        self.get_logger().info(
            f"Saving JPEGs to {self.output_dir.resolve()}"
        )

    def image_callback(self, msg: CompressedImage):
        self.frame_count += 1
        frame = self.decode_frame(msg)
        if frame is None:
            return

        if self.frame_count % self.save_every_n_frames == 0:
            stamp = (
                msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec
            )
            filename = (
                self.output_dir
                / f"row_{stamp}_{self.saved_count:06d}.jpg"
            )
            cv2.imwrite(str(filename), frame)
            self.saved_count += 1
            self.get_logger().info(
                f"saved {filename}",
                throttle_duration_sec=2.0,
            )

        if self.display:
            cv2.imshow("Row data collection", frame)
            cv2.waitKey(1)

    @staticmethod
    def decode_frame(msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except cv2.error:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RowDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
