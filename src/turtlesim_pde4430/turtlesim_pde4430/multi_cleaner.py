#!/usr/bin/env python3
import rclpy, random
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

WORLD = 11.088   # turtlesim approx world size
MARGIN = 0.8     # wall proximity threshold

class MultiCleaner(Node):
    def __init__(self):
        super().__init__('multi_cleaner')
        self.declare_parameter('spawn_count', 3)  # + turtle1 = 4 total
        self.declare_parameter('seed', 42)
        random.seed(int(self.get_parameter('seed').value))

        # Wait for /spawn
        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Spawn up to 3 more turtles (turtle2..turtle4)
        starts = [(3.0, 3.0, 0.0), (8.0, 3.0, 0.0), (3.0, 8.0, 0.0), (8.0, 8.0, 0.0)]
        want = int(self.get_parameter('spawn_count').value)
        self.turtles = ['turtle1']
        for i in range(min(want, 3)):
            x, y, th = starts[i]
            req = Spawn.Request(); req.x = x; req.y = y; req.theta = th; req.name = f'turtle{i+2}'
            try:
                resp = self.spawn_cli.call(req)
                self.turtles.append(resp.name if resp.name else f'turtle{i+2}')
                self.get_logger().info(f"Spawned {self.turtles[-1]} at ({x:.1f},{y:.1f})")
            except Exception as e:
                self.get_logger().error(f"Spawn failed: {e}")

        # Publishers, pose subscribers (Best Effort QoS), and per-turtle state
        self.pubs, self.pose, self.state = {}, {}, {}
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT   # <<< important for turtlesim pose

        for idx, name in enumerate(self.turtles):
            self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Pose, f'/{name}/pose', self._mk_pose_cb(name), qos)
            # start with a brief staggered turn so headings differ
            self.state[name] = {'mode': 'turn', 'until': self._now_ns() + int((0.4 + 0.3*idx)*1e9),
                                'turn_sign': 1 if (idx % 2 == 0) else -1}

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def _mk_pose_cb(self, name):
        def cb(msg: Pose):
            self.pose[name] = msg
        return cb

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _near_wall(self, p: Pose):
        return (p.x < MARGIN or p.x > WORLD - MARGIN or p.y < MARGIN or p.y > WORLD - MARGIN)

    def tick(self):
        for name in self.turtles:
            pub = self.pubs[name]
            st  = self.state[name]
            p   = self.pose.get(name)
            m   = Twist()

            # If we still haven't received a pose, just move forward gently so they start moving
            if p is None:
                m.linear.x = 1.5
                m.angular.z = 0.2
                pub.publish(m)
                continue

            # Wall bounce logic
            if self._near_wall(p) and st['mode'] != 'turn':
                st['mode']  = 'turn'
                st['until'] = self._now_ns() + int(random.uniform(0.6, 1.0) * 1e9)
                st['turn_sign'] = random.choice([-1, 1])

            if st['mode'] == 'turn':
                m.angular.z = st['turn_sign'] * random.uniform(1.4, 2.2)
                if self._now_ns() > st['until']:
                    st['mode'] = 'fwd'
            else:
                # forward cruise with slight per-turtle angular bias to decorrelate paths
                idx = self.turtles.index(name)
                m.linear.x  = 2.0
                m.angular.z = 0.2 + 0.15 * idx

            pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
