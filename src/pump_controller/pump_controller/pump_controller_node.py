import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pigpio


PUMP_PIN = 14


class PumpControllerNode(Node):

    def __init__(self):
        super().__init__('pump_controller')

        self.pi = pigpio.pi()
        self.pi.set_mode(PUMP_PIN, pigpio.OUTPUT)
        self.pi.write(PUMP_PIN, 1)

        self.subscription = self.create_subscription(
            String,
            'pump_control',
            self.listener_callback,
            10
        )

        self.get_logger().info('Pump controller ready. Listening on /pump_control')

    def listener_callback(self, msg):
        command = msg.data.strip().lower()

        if command == 'on':
            self.pi.write(PUMP_PIN, 0)
            self.get_logger().info('Pump ON')

        elif command == 'off':
            self.pi.write(PUMP_PIN, 1)
            self.get_logger().info('Pump OFF')

        else:
            self.get_logger().warn(f'Unknown command: "{msg.data}" — use "on" or "off"')

    def destroy_node(self):
        self.pi.write(PUMP_PIN, 1)
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PumpControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
