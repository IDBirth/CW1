#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute

def wrap(a):
    return (a + math.pi) % (2.0*math.pi) - math.pi

class SquarePatrol(Node):
    def __init__(self):
        super().__init__('square_patrol')

        # parameters
        self.declare_parameter('edge_len', 3.0)     # meters
        self.declare_parameter('speed', 2.0)        # m/s
        self.declare_parameter('turn_speed', 2.0)   # rad/s
        self.declare_parameter('corner_margin', 1.1)
        self.edge_len = float(self.get_parameter('edge_len').value)
        self.speed    = float(self.get_parameter('speed').value)
        self.wz       = float(self.get_parameter('turn_speed').value)
        self.margin   = float(self.get_parameter('corner_margin').value)

        # services
        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn...')
        # teleport for turtle1
        self.t1_tel = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.t1_tel.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute...')

        world = 11.088
        m = self.margin
        # four corners: (x, y, theta). Theta aligns with first leg direction.
        self.corners = [
            (m,       m,       0.0),         # bottom-left, face +x
            (world-m, m,       math.pi/2),   # bottom-right, face +y
            (m,       world-m, 0.0),         # top-left, face +x
            (world-m, world-m, math.pi/2),   # top-right, face +y
        ]

        # place turtles: teleport turtle1, spawn t2..t4
        self.turtles = ['turtle1']
        self._teleport('turtle1', *self.corners[0])
        for i, (x,y,th) in enumerate(self.corners[1:], start=2):
            req = Spawn.Request(); req.x=x; req.y=y; req.theta=th; req.name=f'turtle{i}'
            try:
                resp = self.spawn_cli.call(req)
                name = resp.name if resp and resp.name else f'turtle{i}'
                self.turtles.append(name)
                self.get_logger().info(f"Spawned {name} at ({x:.2f},{y:.2f})")
            except Exception as e:
                self.get_logger().error(f"Spawn {i} failed: {e}")

        # pubs, subs, and per-turtle FSM state
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT  # turtlesim Pose is Best Effort

        self.pub = {}
        self.pose = {}
        self.state = {}  # name -> {'mode': 'fwd'|'turn', 'start_x', 'start_y', 'target_theta'}
        for name in self.turtles:
            self.pub[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'/{name}/pose', self._mk_pose_cb(name), qos)
            # initialize state once first pose arrives
            self.state[name] = {'mode': 'fwd', 'start_x': None, 'start_y': None, 'target_theta': None}

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def _teleport(self, turtle, x, y, theta):
        # turtle1 has its own teleport service; others also expose similar service per name.
        # We will try the specific teleport service for each turtle when needed.
        service_name = f'/{turtle}/teleport_absolute'
        cli = self.create_client(TeleportAbsolute, service_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name}...')
        req = TeleportAbsolute.Request(); req.x = x; req.y = y; req.theta = theta
        try:
            cli.call(req)
        except Exception as e:
            self.get_logger().error(f"Teleport {turtle} failed: {e}")

    def _mk_pose_cb(self, name):
        def cb(msg: Pose):
            self.pose[name] = msg
        return cb

    def tick(self):
        for name in self.turtles:
            p = self.pose.get(name)
            if p is None:
                # move gently until first pose arrives
                self.pub[name].publish(Twist())
                continue

            st = self.state[name]
            # initialize leg start on first pose
            if st['start_x'] is None:
                st['start_x'], st['start_y'] = p.x, p.y
                st['target_theta'] = None
                st['mode'] = 'fwd'

            cmd = Twist()
            if st['mode'] == 'fwd':
                # go straight until traveled >= edge_len
                dx = p.x - st['start_x']
                dy = p.y - st['start_y']
                if math.hypot(dx, dy) >= self.edge_len:
                    st['mode'] = 'turn'
                    st['target_theta'] = wrap(p.theta + math.pi/2)   # left turn 90°
                else:
                    cmd.linear.x = self.speed
                    cmd.angular.z = 0.0

            if st['mode'] == 'turn':
                err = wrap(st['target_theta'] - p.theta)
                if abs(err) < 0.04:
                    # next leg
                    st['mode'] = 'fwd'
                    st['start_x'], st['start_y'] = p.x, p.y
                    cmd.angular.z = 0.0
                else:
                    cmd.angular.z = self.wz if err > 0.0 else -self.wz
                    cmd.linear.x  = 0.0

            self.pub[name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePatrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
