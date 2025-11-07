#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def wrap(a):  # (-pi, pi]
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class GotoGoalAdv(Node):
    """
    Go-to-goal with:
      - P control on distance + heading
      - Smooth slowdown near goal (exponential)
      - Optional final heading alignment
      - Optional multi-waypoint list (semicolon-separated)
    """
    def __init__(self):
        super().__init__('goto_goal_adv')

        # --- Parameters (override with --ros-args -p key:=val) ---
        self.declare_parameter('x', 8.0)
        self.declare_parameter('y', 8.0)
        self.declare_parameter('final_theta', float('nan'))  # rad; NaN -> skip final heading align
        self.declare_parameter('k_lin', 1.2)
        self.declare_parameter('k_ang', 5.0)
        self.declare_parameter('v_max', 2.4)
        self.declare_parameter('w_max', 3.0)
        self.declare_parameter('tolerance', 0.10)           # meters
        self.declare_parameter('angle_tolerance', 0.05)     # rad
        self.declare_parameter('slowdown_radius', 1.0)      # start slowing when within this distance
        self.declare_parameter('waypoints', '')             # "x1,y1;x2,y2;..."
        self.declare_parameter('loop_waypoints', True)

        # read params
        self.goal_x = float(self.get_parameter('x').value)
        self.goal_y = float(self.get_parameter('y').value)
        self.final_theta = float(self.get_parameter('final_theta').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.ang_tol = float(self.get_parameter('angle_tolerance').value)
        self.slow_radius = float(self.get_parameter('slowdown_radius').value)
        self.loop = bool(self.get_parameter('loop_waypoints').value)

        # parse waypoints if provided
        raw = str(self.get_parameter('waypoints').value).strip()
        self.waypoints = []
        if raw:
            try:
                for pair in raw.split(';'):
                    xs, ys = pair.split(',')
                    self.waypoints.append((float(xs), float(ys)))
            except Exception as e:
                self.get_logger().warn(f'Failed to parse waypoints "{raw}": {e}')
        if self.waypoints:
            self.way_idx = 0
            self.goal_x, self.goal_y = self.waypoints[self.way_idx]
        else:
            self.way_idx = -1  # single goal

        # QoS: turtlesim pose is Best Effort
        qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, qos)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.tick)
        self.phase = 'GO'   # GO -> ALIGN (if final_theta provided) -> NEXT/DONE

    def on_pose(self, msg: Pose):
        self.pose = msg

    def set_goal(self, x, y):
        self.goal_x, self.goal_y = float(x), float(y)
        self.phase = 'GO'

    def advance_waypoint(self):
        if self.way_idx < 0:
            self.phase = 'DONE'; return
        self.way_idx += 1
        if self.way_idx >= len(self.waypoints):
            if self.loop:
                self.way_idx = 0
            else:
                self.phase = 'DONE'; return
        self.set_goal(*self.waypoints[self.way_idx])

    def tick(self):
        if self.pose is None:
            # gentle nudge so you know we're alive
            self.pub.publish(Twist()); 
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = math.hypot(dx, dy)
        head_des = math.atan2(dy, dx)
        head_err = wrap(head_des - self.pose.theta)

        # --- Phase: GO (drive towards goal) ---
        if self.phase == 'GO':
            if dist < self.tol:
                # Reached position; optionally align heading
                if math.isnan(self.final_theta):
                    if self.way_idx >= 0:  # multiple waypoints
                        self.advance_waypoint()
                    else:
                        self.phase = 'DONE'
                else:
                    self.phase = 'ALIGN'
                self.pub.publish(Twist()); 
                return

            # Smooth slowdown inside slow_radius: v = v_max * (1 - exp(-dist/slow_radius))
            v_des = self.v_max * (1.0 - math.exp(-max(0.0, dist) / max(1e-3, self.slow_radius)))
            v_cmd = min(self.v_max, max(0.0, self.k_lin * dist, v_des))

            # Heading control
            w_cmd = self.k_ang * head_err
            # limit angular speed
            w_cmd = max(-self.w_max, min(self.w_max, w_cmd))

            # If we are very misaligned, slow forward speed a bit to turn
            v_cmd *= max(0.2, 1.0 - min(1.0, abs(head_err) / 1.2))

            msg = Twist()
            msg.linear.x = v_cmd
            msg.angular.z = w_cmd
            self.pub.publish(msg)
            return

        # --- Phase: ALIGN (final heading) ---
        if self.phase == 'ALIGN':
            err = wrap(self.final_theta - self.pose.theta)
            if abs(err) < self.ang_tol:
                if self.way_idx >= 0:
                    self.advance_waypoint()
                else:
                    self.phase = 'DONE'
                self.pub.publish(Twist()); 
                return
            msg = Twist()
            msg.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * err))
            self.pub.publish(msg)
            return

        # --- DONE ---
        if self.phase == 'DONE':
            self.pub.publish(Twist())
            return

def main(args=None):
    rclpy.init(args=args)
    node = GotoGoalAdv()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
