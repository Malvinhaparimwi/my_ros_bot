#!/usr/bin/env python3

import os
import select
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


JPEG_START = b'\xff\xd8'
JPEG_END = b'\xff\xd9'
MAX_BUFFER_BYTES = 4 * 1024 * 1024


class StereoCameraNode(Node):

    def __init__(self):
        super().__init__('stereo_camera_node')

        # Publishers
        self.pub_left = self.create_publisher(
            CompressedImage,
            '/camera/left/compressed',
            10
        )

        self.camera_proc = None
        self.jpeg_buffer = bytearray()
        self.open_camera()

        # Timer (20 FPS)
        self.timer = self.create_timer(
            1/20,
            self.timer_callback
        )

        self.get_logger().info('Camera node ready')

    def open_camera(self):
        if self.camera_proc and self.camera_proc.poll() is None:
            return True

        self.get_logger().info('Opening Pi Camera with rpicam-vid...')

        cmd = [
            'rpicam-vid',
            '--codec', 'mjpeg',
            '--width', '1280',
            '--height', '720',
            '--framerate', '20',
            '--timeout', '0',
            '--nopreview',
            '--output', '-',
        ]

        try:
            self.camera_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except FileNotFoundError:
            self.get_logger().error('rpicam-vid not found')
            self.camera_proc = None
            return False

        self.jpeg_buffer.clear()
        self.get_logger().info('Pi Camera process started')

        return True

    def close_camera(self):
        if not self.camera_proc:
            return

        if self.camera_proc.poll() is None:
            self.camera_proc.terminate()
            try:
                self.camera_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.camera_proc.kill()

        self.camera_proc = None
        self.jpeg_buffer.clear()

    def read_latest_jpeg(self):
        if not self.camera_proc or self.camera_proc.poll() is not None:
            self.close_camera()
            self.open_camera()
            return None

        stdout = self.camera_proc.stdout
        if not stdout:
            return None

        while True:
            ready, _, _ = select.select([stdout], [], [], 0)
            if not ready:
                break

            chunk = os.read(stdout.fileno(), 65536)
            if not chunk:
                break

            self.jpeg_buffer.extend(chunk)

            if len(self.jpeg_buffer) > MAX_BUFFER_BYTES:
                del self.jpeg_buffer[:-MAX_BUFFER_BYTES]

        latest_frame = None

        while True:
            start = self.jpeg_buffer.find(JPEG_START)
            if start < 0:
                if len(self.jpeg_buffer) > 1:
                    del self.jpeg_buffer[:-1]
                break

            if start > 0:
                del self.jpeg_buffer[:start]

            end = self.jpeg_buffer.find(JPEG_END, len(JPEG_START))
            if end < 0:
                break

            frame_end = end + len(JPEG_END)
            latest_frame = bytes(self.jpeg_buffer[:frame_end])
            del self.jpeg_buffer[:frame_end]

        return latest_frame

    def make_compressed_msg(self, jpeg_data, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = 'jpeg'
        msg.data = jpeg_data

        return msg

    def timer_callback(self):
        jpeg_data = self.read_latest_jpeg()
        if not jpeg_data:
            return

        stamp = self.get_clock().now().to_msg()
        left_msg = self.make_compressed_msg(jpeg_data, stamp)

        if left_msg:
            self.pub_left.publish(left_msg)

    def destroy_node(self):

        self.get_logger().info('Shutting down camera...')
        self.close_camera()

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
