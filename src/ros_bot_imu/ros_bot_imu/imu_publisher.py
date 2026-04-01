#!/usr/bin/env python3
import json
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Increased timeout to prevent blocking
imu_ser = serial.Serial("/dev/serial/by-id/usb-Arduino_Nano_33_BLE_717C68010C91B35B-if00", 115200, timeout=0.1)

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.imu_publisher = self.create_publisher(Imu, "onyx/imu", 10)

        # Timer to read IMU data
        self.timer = self.create_timer(0.01, self.read_imu_data)
        self.get_logger().info("Imu Ready")


    def read_imu_data(self):
        if imu_ser.in_waiting > 0:
            try:
                line = imu_ser.readline().decode('utf-8').strip()
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
    node = ImuNode()
    rclpy.spin(node)
    imu_ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
