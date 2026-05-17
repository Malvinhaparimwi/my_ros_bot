#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2


class StereoCameraNode(Node):

    def __init__(self):
        super().__init__('stereo_camera_node')

        # Publishers
        self.pub_left = self.create_publisher(
            CompressedImage,
            '/camera/left/compressed',
            10
        )

        self.pi_pipeline = (
            "libcamerasrc ! "
            "video/x-raw,width=1280,height=720,framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        self.cam_pi = None
        self.open_camera()

        # Timer (20 FPS)
        self.timer = self.create_timer(
            1/20,
            self.timer_callback
        )

        self.get_logger().info('Camera node ready')

    def open_camera(self):
        self.get_logger().info('Opening Pi Camera...')

        camera = cv2.VideoCapture(self.pi_pipeline, cv2.CAP_GSTREAMER)

        if not camera.isOpened():
            self.get_logger().warn('Pi Camera unavailable; will retry')
            camera.release()
            return False

        self.cam_pi = camera
        self.get_logger().info('Pi Camera opened OK')
        self.get_logger().info('Warming up camera...')

        for _ in range(10):
            self.cam_pi.read()

        return True

    def make_compressed_msg(self, frame, stamp, quality=80):

        ret, buf = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, quality]
        )

        if not ret:
            return None

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = 'jpeg'
        msg.data = buf.tobytes()

        return msg

    def timer_callback(self):

        if not self.cam_pi:
            self.open_camera()
            return

        ret, frame = self.cam_pi.read()

        if not ret:
            self.get_logger().warn('Pi Camera frame drop')
            self.cam_pi.release()
            self.cam_pi = None
            return

        stamp = self.get_clock().now().to_msg()

        left_msg = self.make_compressed_msg(
            frame,
            stamp
        )

        if left_msg:
            self.pub_left.publish(left_msg)

    def destroy_node(self):

        self.get_logger().info('Shutting down camera...')

        if self.cam_pi:
            self.cam_pi.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = StereoCameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
