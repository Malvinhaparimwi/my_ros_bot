#!/usr/bin/env python3
import json
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist

data = serial.Serial("/dev/serial/by-id/usb-Arduino_Nano_33_BLE_3E1994D9FA7CF7D9-if00", 9600)

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.cmd_dictinary = {
            "linear_x": 0.0,
            "angular_z": 0.0,
        }

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, qos_profile=10)
        self.get_logger().info("Controller Ready")

        
    def cmd_vel_cb(self, msg):
        # Update the dictionary with current values
        self.cmd_dictinary["linear_x"] = msg.linear.x
        self.cmd_dictinary["angular_z"] = msg.angular.z

        # Serialize dictionary to JSON string and send with newline
        json_msg = json.dumps(self.cmd_dictinary) + "\n"
        data.write(json_msg.encode("ascii"))
        # print(json_msg)



def main(args=None): 
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()