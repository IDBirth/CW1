#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UserInput(Node):
    def __init__(self):
        super().__init__('user_input')
        self.declare_parameter('linear', 1.0)
        self.declare_parameter('angular', 0.0)

        if sys.stdin.isatty():
            try:
                lin = float(input('Linear speed (m/s) [1.0]: ') or '1.0')
                ang = float(input('Angular speed (rad/s) [0.0]: ') or '0.0')
            except Exception:
                lin, ang = 1.0, 0.0
        else:
            lin = float(self.get_parameter('linear').value)
            ang = float(self.get_parameter('angular').value)

        self.linear = lin
        self.angular = ang

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'user_input running: linear={lin:.2f}, angular={ang:.2f}')

    def tick(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UserInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
