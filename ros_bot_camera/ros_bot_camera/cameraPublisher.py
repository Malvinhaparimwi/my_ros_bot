#!/usr/bin/env python3
from gpiozero import Motor, PWMOutputDevice
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.left_motor = Motor(forward=17, backward=27, pwm=True)
        self.right_motor = Motor(forward=23, backward=24, pwm=True)
        self.enableA = PWMOutputDevice(13)
        self.enableB = PWMOutputDevice(12)

        # Declare parameters for flexibility
        self.declare_parameter("device_id", 0)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("width", 820)
        self.declare_parameter("height", 640)

        # Load parameter values
        self.device_id = self.get_parameter("device_id").value
        self.frame_rate = self.get_parameter("frame_rate").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value

        # Initialize OpenCV camera
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera device {self.device_id}")
            rclpy.shutdown()
            return

        # ROS image publisher
        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        self.bridge = CvBridge()

        # Timer for publishing frames
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)
        self.get_logger().info(f"Publishing raw camera frames on 'camera/image_raw' at {self.frame_rate} FPS")

        self.cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_cb, 10)

    def cmd_cb(self, msg):
        forward = msg.linear.x
        turn = msg.angular.z

        self.enableA.value = abs(0.4)
        self.enableB.value = abs(0.2)

        self.left_motor.forward(1.0)
        self.left_motor.backward(1.0)
        self.right_motor.forward(1.0)
        self.right_motor.backward(1.0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        # Resize the frame (optional)
        frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_CUBIC)

        # Convert OpenCV image (BGR) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
