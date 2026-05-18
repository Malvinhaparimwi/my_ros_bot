#!/usr/bin/env python3

import os
import select
import subprocess

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


JPEG_START = b'\xff\xd8'
JPEG_END = b'\xff\xd9'
MAX_BUFFER_BYTES = 4 * 1024 * 1024

# Change this index if your USB webcam is not /dev/video0
USB_CAMERA_INDEX = 1


class StereoCameraNode(Node):

    def __init__(self):
        super().__init__('stereo_camera_node')

        # Publishers
        self.pub_left = self.create_publisher(
            CompressedImage,
            '/camera/left/compressed',
            10
        )
        self.pub_right = self.create_publisher(
            CompressedImage,
            '/camera/right/compressed',
            10
        )

        # Pi Camera (left)
        self.camera_proc = None
        self.jpeg_buffer = bytearray()
        self.open_camera()

        # USB Webcam (right)
        self.usb_cap = None
        self.open_usb_camera()

        # Timer (20 FPS)
        self.timer = self.create_timer(
            1 / 20,
            self.timer_callback
        )

        self.get_logger().info('Stereo camera node ready')

    # ------------------------------------------------------------------
    # Pi Camera (left) — rpicam-vid via subprocess
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # USB Webcam (right) — OpenCV VideoCapture
    # ------------------------------------------------------------------

    def open_usb_camera(self):
        if self.usb_cap and self.usb_cap.isOpened():
            return True

        self.get_logger().info(
            f'Opening USB webcam at index {USB_CAMERA_INDEX}...'
        )

        cap = cv2.VideoCapture(USB_CAMERA_INDEX, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(
                f'Failed to open USB webcam at index {USB_CAMERA_INDEX}. '
                'Check that the device is connected and try a different index.'
            )
            self.usb_cap = None
            return False

        # Match resolution/FPS to the Pi camera where possible
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 20)

        self.usb_cap = cap
        self.get_logger().info('USB webcam opened successfully')
        return True

    def close_usb_camera(self):
        if self.usb_cap:
            self.usb_cap.release()
            self.usb_cap = None

    def read_usb_jpeg(self):
        if not self.usb_cap or not self.usb_cap.isOpened():
            self.close_usb_camera()
            self.open_usb_camera()
            return None

        ret, frame = self.usb_cap.read()
        if not ret or frame is None:
            self.get_logger().warning('USB webcam read failed, will retry')
            self.close_usb_camera()
            return None

        # Encode BGR frame to JPEG in memory
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return None

        return buf.tobytes()

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def make_compressed_msg(self, jpeg_data, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = 'jpeg'
        msg.data = jpeg_data
        return msg

    # ------------------------------------------------------------------
    # Timer callback — runs at 20 Hz
    # ------------------------------------------------------------------

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        # Left — Pi Camera
        jpeg_left = self.read_latest_jpeg()
        if jpeg_left:
            self.pub_left.publish(self.make_compressed_msg(jpeg_left, stamp))

        # Right — USB Webcam
        jpeg_right = self.read_usb_jpeg()
        if jpeg_right:
            self.pub_right.publish(self.make_compressed_msg(jpeg_right, stamp))

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Shutting down cameras...')
        self.close_camera()
        self.close_usb_camera()
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
