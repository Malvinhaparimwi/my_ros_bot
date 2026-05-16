import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pigpio


PUMP_PIN = 14
PUMP_TOPIC = '/pump/controller'
LEGACY_PUMP_TOPIC = 'pump_control'
RELAY_ON = 0
RELAY_OFF = 1


class PumpControllerNode(Node):

    def __init__(self):
        super().__init__('pump_controller')

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                'Could not connect to pigpiod. Start it with: sudo systemctl start pigpiod'
            )

        self.pi.set_mode(PUMP_PIN, pigpio.OUTPUT)
        self.pi.write(PUMP_PIN, RELAY_OFF)  # active-low relay: HIGH = off

        self.subscription = self.create_subscription(
            String,
            PUMP_TOPIC,
            self.listener_callback,
            10
        )
        self.legacy_subscription = self.create_subscription(
            String,
            LEGACY_PUMP_TOPIC,
            self.listener_callback,
            10
        )

        self.get_logger().info(
            f'Pump controller ready on {PUMP_TOPIC} using pigpiod GPIO {PUMP_PIN}'
        )

    def listener_callback(self, msg):
        command = msg.data.strip().lower()

        if command == 'on':
            self.pi.write(PUMP_PIN, RELAY_ON)
            self.get_logger().info('Pump ON')

        elif command == 'off':
            self.pi.write(PUMP_PIN, RELAY_OFF)
            self.get_logger().info('Pump OFF')

        else:
            self.get_logger().warn(f'Unknown command: "{msg.data}" - use "on" or "off"')

    def destroy_node(self):
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.write(PUMP_PIN, RELAY_OFF)  # turn off pump on shutdown
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PumpControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'pump_controller failed: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
