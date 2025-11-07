#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        self.declare_parameter('x', 8.0)
        self.declare_parameter('y', 8.0)
        self.declare_parameter('k_lin', 1.0)
        self.declare_parameter('k_ang', 4.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 2.0)
        self.declare_parameter('tolerance', 0.12)

        self.goal = (
            float(self.get_parameter('x').value),
            float(self.get_parameter('y').value),
        )

        self.pose = None
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'goto_goal -> target {self.goal}')

    def on_pose(self, msg: Pose):
        self.pose = msg

    def tick(self):
        if self.pose is None:
            return
        xg, yg = self.goal
        dx = xg - self.pose.x
        dy = yg - self.pose.y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        err = wrap(heading - self.pose.theta)

        k_lin = float(self.get_parameter('k_lin').value)
        k_ang = float(self.get_parameter('k_ang').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        tol   = float(self.get_parameter('tolerance').value)

        cmd = Twist()
        if dist < tol:
            self.pub.publish(cmd)  # zero
            return

        cmd.linear.x  = max(0.0, min(v_max, k_lin * dist))
        cmd.angular.z = max(-w_max, min(w_max, k_ang * err))
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GotoGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
