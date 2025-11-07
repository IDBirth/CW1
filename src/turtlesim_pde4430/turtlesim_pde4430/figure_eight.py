#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class FigureEight(Node):
    def __init__(self):
        super().__init__('figure_eight')
        self.declare_parameter('cx', 5.5)
        self.declare_parameter('cy', 5.5)
        self.declare_parameter('a', 3.0)
        self.declare_parameter('b', 3.0)
        self.declare_parameter('omega', 0.6)
        self.declare_parameter('k_pos', 1.2)
        self.declare_parameter('k_ang', 4.0)
        self.declare_parameter('v_max', 2.0)

        self.dt = 0.05
        self.t  = 0.0
        self.pose = None

        self.sub  = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.tick)

    def on_pose(self, msg: Pose):
        self.pose = msg

    def desired(self, t):
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        a  = float(self.get_parameter('a').value)
        b  = float(self.get_parameter('b').value)
        w  = float(self.get_parameter('omega').value)

        x_d = cx + a * math.sin(t)
        y_d = cy + b * math.sin(t) * math.cos(t)
        xd  = a * math.cos(t) * w
        yd  = b * (math.cos(t)**2 - math.sin(t)**2) * w
        return x_d, y_d, xd, yd

    def tick(self):
        if self.pose is None:
            return
        k_pos = float(self.get_parameter('k_pos').value)
        k_ang = float(self.get_parameter('k_ang').value)
        v_max = float(self.get_parameter('v_max').value)

        x_d, y_d, xd, yd = self.desired(self.t)
        ex = self.pose.x - x_d
        ey = self.pose.y - y_d
        vx_cmd = xd - k_pos * ex
        vy_cmd = yd - k_pos * ey

        phi = math.atan2(vy_cmd, vx_cmd)
        v   = math.hypot(vx_cmd, vy_cmd)
        heading_err = wrap(phi - self.pose.theta)

        msg = Twist()
        msg.linear.x  = min(v_max, max(0.0, v))
        msg.angular.z = k_ang * heading_err
        self.pub.publish(msg)

        self.t += self.dt * float(self.get_parameter('omega').value)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
