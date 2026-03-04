#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

data = serial.Serial("/dev/ttyACM1", 9600)

class Controller(Node):
    def __init__(self):
        super().__init__("Controller")

        self.subscription = self.create_subscription(
            String,
            "controller",
            self.controller_cb,
            10
        )

        self.get_logger().info("Initialized")

    def controller_cb(self, msg):
        myCmd = msg.data
        myCmd = myCmd + "\r"
        data.write(myCmd.encode())


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()