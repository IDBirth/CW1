#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircularMove(Node):
    def __init__(self):
        super().__init__('circular_move')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 1.0)  # linear speed
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        r = float(self.get_parameter('radius').value)
        v = float(self.get_parameter('speed').value)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = v / max(r, 1e-3)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircularMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
