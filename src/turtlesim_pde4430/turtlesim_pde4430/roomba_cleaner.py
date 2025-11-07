#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

WORLD = 11.088

def wrap(a): return (a + math.pi) % (2.0 * math.pi) - math.pi

class RoombaCleaner(Node):
    def __init__(self):
        super().__init__('roomba_cleaner')

        # Params
        self.declare_parameter('margin', 0.7)
        self.declare_parameter('lane_spacing', 0.5)
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('turn_speed', 2.0)
        self.declare_parameter('pen_r', 255)
        self.declare_parameter('pen_g', 0)
        self.declare_parameter('pen_b', 0)
        self.declare_parameter('pen_width', 5)
        self.declare_parameter('teleport_start', False)  # default to False to avoid any service wait

        self.margin       = float(self.get_parameter('margin').value)
        self.lane_spacing = float(self.get_parameter('lane_spacing').value)
        self.speed        = float(self.get_parameter('speed').value)
        self.turn_speed   = float(self.get_parameter('turn_speed').value)
        self.pen_rgbw     = (
            int(self.get_parameter('pen_r').value),
            int(self.get_parameter('pen_g').value),
            int(self.get_parameter('pen_b').value),
            int(self.get_parameter('pen_width').value),
        )
        self.teleport_start = bool(self.get_parameter('teleport_start').value)

        # QoS for turtlesim Pose (Best Effort)
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose = None
        self.sub  = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, qos)
        self.pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Non-blocking service clients (we won’t wait — we’ll try once later)
        self.set_pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.tele_cli    = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # FSM
        self.dir_right = True
        self.target_theta = 0.0
        self.mode = 'init'
        self.current_lane_y = None

        self.timer = self.create_timer(0.05, self.tick)
        self._once_tried_services = False
        self._heartbeat_last = 0

    def on_pose(self, msg: Pose):
        self.pose = msg

    # --- helper actions (non-blocking) ---
    def try_set_pen(self):
        if not self.set_pen_cli.service_is_ready():
            return
        r,g,b,w = self.pen_rgbw
        req = SetPen.Request(); req.r=r; req.g=g; req.b=b; req.width=w; req.off=0
        self.set_pen_cli.call_async(req)

    def try_teleport(self, x, y, theta):
        if not self.tele_cli.service_is_ready():
            return
        req = TeleportAbsolute.Request(); req.x=float(x); req.y=float(y); req.theta=float(theta)
        self.tele_cli.call_async(req)

    def near_right_wall(self): return self.pose.x > (WORLD - self.margin)
    def near_left_wall (self): return self.pose.x <  self.margin
    def near_top       (self): return self.pose.y > (WORLD - self.margin)
    def at_heading(self, theta, tol=0.03): return abs(wrap(theta - self.pose.theta)) < tol

    def face_heading(self, theta):
        err = wrap(theta - self.pose.theta)
        cmd = Twist(); cmd.angular.z = self.turn_speed if err > 0 else -self.turn_speed
        self.pub.publish(cmd)

    def forward(self, v=None):
        cmd = Twist(); cmd.linear.x = self.speed if v is None else v
        self.pub.publish(cmd)

    def tick(self):
        # Heartbeat (debug): proves the timer is firing
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._heartbeat_last > 1_000_000_000:
            self.get_logger().info(f'mode={self.mode}')
            self._heartbeat_last = now_ns

        # Publish something even if we don't have pose yet — you'll see immediate motion.
        if self.pose is None:
            cmd = Twist(); cmd.linear.x = 1.2; cmd.angular.z = 0.2
            self.pub.publish(cmd)
            return

        # One-time, non-blocking setup
        if not self._once_tried_services:
            self.try_set_pen()
            if self.teleport_start:
                self.try_teleport(self.margin, self.margin, 0.0)
            self._once_tried_services = True

        # --- FSM ---
        if self.mode == 'init':
            # use current y as starting lane
            self.current_lane_y = self.pose.y if not self.teleport_start else self.margin
            self.dir_right = True
            self.mode = 'align'
            return

        if self.mode == 'align':
            self.target_theta = 0.0 if self.dir_right else math.pi
            if not self.at_heading(self.target_theta):
                self.face_heading(self.target_theta); return
            self.mode = 'sweep'; return

        if self.mode == 'sweep':
            if self.dir_right and self.near_right_wall():
                self.mode = 'shift_up_prep'; return
            if (not self.dir_right) and self.near_left_wall():
                self.mode = 'shift_up_prep'; return
            self.forward(); return

        if self.mode == 'shift_up_prep':
            # face +y and climb one lane
            if not self.at_heading(math.pi/2):
                self.face_heading(math.pi/2); return
            target_y = self.current_lane_y + self.lane_spacing
            if self.near_top() or target_y > (WORLD - self.margin):
                self.mode = 'done'; return
            if self.pose.y < target_y:
                self.forward(); return
            self.current_lane_y = target_y
            self.dir_right = not self.dir_right
            self.mode = 'align'; return

        if self.mode == 'done':
            self.pub.publish(Twist()); return

def main(args=None):
    rclpy.init(args=args)
    node = RoombaCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
