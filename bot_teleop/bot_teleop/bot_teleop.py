#!/usr/bin/env python3
import os
import sys
import rclpy
import select
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Choosing which platform we are working on, 'posix' or 'new technology(nt)
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


class Controller(Node):
    def __init__(self, start_parameter_services=False):
        super().__init__(
            'Controller',
            start_parameter_services=start_parameter_services
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_lin_vel', 0.22),
                ('max_ang_vel', 12.84),
                ('lin_vel_step', 0.05),
                ('ang_vel_step', 0.1),
                ('lin_vel_step_large', 0.25),
                ('ang_vel_step_large', 0.5),
            ])
        
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.lin_vel_step = self.get_parameter('lin_vel_step').value
        self.ang_vel_step = self.get_parameter('ang_vel_step').value
        self.lin_vel_step_large = self.get_parameter('lin_vel_step_large').value
        self.ang_vel_step_large = self.get_parameter('ang_vel_step_large').value

        print('Max linear velocity {:.3f}\t Max angular velocity {:.3f}'.format(
            round(self.max_lin_vel, 3),
            round(self.max_ang_vel, 3))
        )

        self.tty_attr = None if os.name == 'nt' else termios.tcgetattr(sys.stdin)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        ui = f"""
==============================
    SIMPLE TELEOPERATION
==============================
        ( i )

  ( j ) ( k ) ( l )

        ( m )

=============================="""
        print(ui)

    def get_key(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [])

        key = sys.stdin.read(1) if rlist else ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_attr)
        return key

    @staticmethod
    def constrain(input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def check_linear_limit_velocity(self, velocity):
        return self.constrain(velocity, -self.max_lin_vel, self.max_lin_vel)

    def check_angular_limit_velocity(self, velocity):
        return self.constrain(velocity, -self.max_ang_vel, self.max_ang_vel)

    def perform(self):
        key = self.get_key()
        if key == 'i':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity + self.lin_vel_step)
        elif key == 'I':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity + self.lin_vel_step_large)
        elif key == 'm':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity - self.lin_vel_step)
        elif key == 'M':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity - self.lin_vel_step_large)
        elif key == 'j':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity + self.ang_vel_step)
        elif key == 'J':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity + self.ang_vel_step_large)
        elif key == 'l':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity - self.ang_vel_step)
        elif key == 'L':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity - self.ang_vel_step_large)
        elif key == 'K' or key == 'k':
            self.angular_velocity = 0.0
            self.linear_velocity = 0.0
        elif (key == '\x03'):
            print('Stopping the robot and exiting on CTRL-C press')

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.publisher_.publish(twist)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_attr)

            return False

        twist = Twist()

        twist.linear.x = self.linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_velocity

        self.publisher_.publish(twist)

        return True



def main(args=None): 
    rclpy.init(args=args)
    node = Controller(start_parameter_services=False)

    while(node.perform()):
        rclpy.spin_once(node, timeout_sec=0.001)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main