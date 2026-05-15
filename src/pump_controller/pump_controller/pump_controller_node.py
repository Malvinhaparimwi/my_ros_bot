import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


PUMP_PIN = 17


class PumpControllerNode(Node):

    def __init__(self):
        super().__init__('pump_controller')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_PIN, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = relay off

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
            GPIO.output(PUMP_PIN, GPIO.LOW)
            self.get_logger().info('Pump ON')

        elif command == 'off':
            GPIO.output(PUMP_PIN, GPIO.HIGH)
            self.get_logger().info('Pump OFF')

        else:
            self.get_logger().warn(f'Unknown command: "{msg.data}" — use "on" or "off"')

    def destroy_node(self):
        GPIO.output(PUMP_PIN, GPIO.HIGH)  # turn off pump on shutdown
        GPIO.cleanup()
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