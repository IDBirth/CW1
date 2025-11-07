#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StraightMove(Node):
    def __init__(self):
        super().__init__('straight_move')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def tick(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StraightMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
