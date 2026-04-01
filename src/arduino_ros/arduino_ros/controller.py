#!/usr/bin/env python3
import json
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

# Increased timeout to prevent blocking
ser = serial.Serial("/dev/serial/by-id/usb-Arduino_Nano_33_BLE_3E1994D9FA7CF7D9-if00", 9600, timeout=0.1)

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.cmd_dictinary = {"linear_x": 0.0, "angular_z": 0.0}

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'onyx/cmd_vel', self.cmd_vel_cb, 10)
        self.imu_publisher = self.create_publisher(Imu, "onyx/imu", 10)

        # Timer to read IMU data
        self.timer = self.create_timer(0.01, self.read_imu_data)
        self.get_logger().info("Controller Ready")

    def cmd_vel_cb(self, msg):
        self.cmd_dictinary["linear_x"] = msg.linear.x
        self.cmd_dictinary["angular_z"] = msg.angular.z
        json_msg = json.dumps(self.cmd_dictinary) + "\n"
        ser.write(json_msg.encode("ascii"))

    def read_imu_data(self):
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line and line.startswith('{'): # Ensure it looks like JSON
                    imu_data = json.loads(line)

                    # FIX: Check imu_data, not 'data' (the serial object)
                    if "ax" in imu_data:
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"

                        # Use imu_data dictionary
                        imu_msg.linear_acceleration.x = imu_data['ax'] * 9.80665
                        imu_msg.linear_acceleration.y = imu_data['ay'] * 9.80665
                        imu_msg.linear_acceleration.z = imu_data['az'] * 9.80665

                        imu_msg.angular_velocity.x = imu_data['gx'] * (3.14159 / 180.0)
                        imu_msg.angular_velocity.y = imu_data['gy'] * (3.14159 / 180.0)
                        imu_msg.angular_velocity.z = imu_data['gz'] * (3.14159 / 180.0)

                        self.imu_publisher.publish(imu_msg)
            except Exception as e:
                # Catching parsing errors from partial serial strings
                pass

def main(args=None): 
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()