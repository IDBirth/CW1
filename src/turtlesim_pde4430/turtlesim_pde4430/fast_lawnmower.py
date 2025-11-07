#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

WORLD = 11.088
def wrap(a): return (a + math.pi) % (2.0*math.pi) - math.pi

class FastLawnmower(Node):
    """
    High-throughput coverage of a rectangular area:
      - straight lanes at 0°
      - Dubins-like constant-radius U-turns
      - lane spacing derived from pen width (no gaps)
    """
    def __init__(self):
        super().__init__('fast_lawnmower')

        # Params (tunable via --ros-args -p key:=val)
        self.declare_parameter('margin', 0.7)            # wall margin
        self.declare_parameter('pen_r', 255)
        self.declare_parameter('pen_g', 0)
        self.declare_parameter('pen_b', 0)
        self.declare_parameter('pen_width', 5)           # turtlesim pen pixels
        self.declare_parameter('scale_px_to_m', 0.015)   # ≈ meters per pixel
        self.declare_parameter('v_lane', 3.2)            # m/s straight
        self.declare_parameter('w_turn', 3.8)            # rad/s during arc
        self.declare_parameter('teleport_start', True)
        self.declare_parameter('draw_shifts', False)     # pen off during lane shifts = faster

        # read params
        self.margin     = float(self.get_parameter('margin').value)
        self.pen_rgbw   = (int(self.get_parameter('pen_r').value),
                           int(self.get_parameter('pen_g').value),
                           int(self.get_parameter('pen_b').value),
                           int(self.get_parameter('pen_width').value))
        self.scale      = float(self.get_parameter('scale_px_to_m').value)
        self.v_lane     = float(self.get_parameter('v_lane').value)
        self.w_turn     = float(self.get_parameter('w_turn').value)
        self.tele_start = bool(self.get_parameter('teleport_start').value)
        self.draw_shifts= bool(self.get_parameter('draw_shifts').value)

        # lane spacing from pen width (avoid gaps/overdraw)
        pen_px = self.pen_rgbw[3]
        self.lane_spacing = max(0.15, pen_px * self.scale)

        # QoS (turtlesim pose is Best Effort)
        qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.sub  = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, qos)
        self.pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # async services
        self.set_pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.tele_cli    = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # FSM
        self.mode = 'init'
        self.dir_right = True
        self.y_lane = None
        self.turn_dir = +1
        self.arc_accum = 0.0

        # geometry
        self.x_left  = self.margin
        self.x_right = WORLD - self.margin
        self.y_top   = WORLD - self.margin

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.tick)

    def on_pose(self, p: Pose): self.pose = p

    # helpers
    def set_pen(self, on=True):
        if not self.set_pen_cli.service_is_ready(): return
        r,g,b,w = self.pen_rgbw
        req = SetPen.Request(); req.r,req.g,req.b,req.width = r,g,b,w; req.off = 0 if on else 1
        self.set_pen_cli.call_async(req)

    def teleport(self, x, y, th):
        if not self.tele_cli.service_is_ready(): return
        req = TeleportAbsolute.Request(); req.x,req.y,req.theta = float(x),float(y),float(th)
        self.tele_cli.call_async(req)

    def send(self, v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.pub.publish(m)

    # main loop
    def tick(self):
        if self.pose is None:
            # bootstrap movement until first pose
            self.send(1.0, 0.2); return

        if self.mode == 'init':
            self.set_pen(True)
            if self.tele_start:
                self.teleport(self.x_left, self.margin, 0.0)  # start bottom-left, face +x
                self.y_lane = self.margin
            else:
                self.y_lane = self.pose.y
            self.dir_right = True
            self.mode = 'lane'; return

        if self.mode == 'lane':
            if self.dir_right:
                if self.pose.x >= self.x_right:
                    self.mode = 'u_turn_prep'; self.turn_dir = +1; self.arc_accum = 0.0
                else:
                    self.send(self.v_lane, 0.0)
            else:
                if self.pose.x <= self.x_left:
                    self.mode = 'u_turn_prep'; self.turn_dir = -1; self.arc_accum = 0.0
                else:
                    self.send(self.v_lane, 0.0)
            return

        if self.mode == 'u_turn_prep':
            if not self.draw_shifts: self.set_pen(False)  # lift pen during shift+turn (faster)
            self.mode = 'u_turn_arc'; return

        if self.mode == 'u_turn_arc':
            # constant-radius arc: v = R*w ; pick R from lane spacing
            R = 0.9 * self.lane_spacing
            v = abs(self.w_turn) * R
            w = self.turn_dir * abs(self.w_turn)
            self.send(v, w)
            self.arc_accum += abs(w) * self.dt
            if self.arc_accum >= math.pi:   # ~180° heading change
                self.mode = 'lane_shift'
            return

        if self.mode == 'lane_shift':
            target_y = self.y_lane + self.lane_spacing
            if target_y >= self.y_top:
                self.mode = 'done'; self.set_pen(True); self.send(0.0, 0.0); return
            # face +y
            err = wrap(math.pi/2 - self.pose.theta)
            if abs(err) > 0.03:
                self.send(0.0, 4.0 if err > 0 else -4.0); return
            # climb to next lane height
            if self.pose.y < target_y:
                self.send(self.v_lane, 0.0); return
            # at next lane
            self.y_lane = target_y
            self.dir_right = not self.dir_right
            self.set_pen(True)
            goal = 0.0 if self.dir_right else math.pi
            err = wrap(goal - self.pose.theta)
            if abs(err) > 0.03:
                self.send(0.0, 4.0 if err > 0 else -4.0); return
            self.mode = 'lane'; return

        if self.mode == 'done':
            self.send(0.0, 0.0); return

def main(args=None):
    rclpy.init(args=args)
    node = FastLawnmower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
