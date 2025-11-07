#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, Spawn, Kill
from turtlesim.msg import Pose

class FullMapCleaner(Node):
    def __init__(self):
        super().__init__('full_map_cleaner')

        # Publishers for turtle1..turtle4
        self.pubs = [self.create_publisher(Twist, f'/turtle{i}/cmd_vel', 10) for i in range(1, 5)]

        # Best-Effort QoS (turtlesim Pose is Best Effort)
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Pose buffers and subscribers
        self.poses = [None] * 4
        self.subs = [
            self.create_subscription(
                Pose, f'/turtle{i}/pose',
                (lambda idx: (lambda msg: self._set_pose(idx, msg)))(i-1),
                qos
            )
            for i in range(1, 5)
        ]

        # Per-turtle simple state machines
        self.states = [{"moving": True, "row": 0, "complete": False} for _ in range(4)]

        # Quadrant limits (your original layout)
        self.limits = [
            {"x_min": 5.5, "x_max": 11.0, "y_min": 5.5, "y_max": 11.0, "inc": 0.7},  # Q1
            {"x_min": 0.0, "x_max": 5.5,  "y_min": 5.5, "y_max": 11.0, "inc": 0.7},  # Q2
            {"x_min": 5.5, "x_max": 11.0, "y_min": 0.0, "y_max": 5.5,  "inc": 0.7},  # Q3
            {"x_min": 0.0, "x_max": 5.5,  "y_min": 0.0, "y_max": 5.5,  "inc": 0.7},  # Q4
        ]

        # Max rows/cols per your even/odd rule
        self.max_rows = [
            int((l["y_max"] - l["y_min"]) / l["inc"]) if i % 2 == 0
            else int((l["x_max"] - l["x_min"]) / l["inc"])
            for i, l in enumerate(self.limits)
        ]

        # Shared service clients
        self.kill_cli  = self.create_client(Kill,  '/kill')
        self.spawn_cli = self.create_client(Spawn, '/spawn')

        # Control loop
        self.timer = self.create_timer(0.1, self.loop)

        # One-shot init timer & flag
        self._init_timer = self.create_timer(0.2, self.spawn_and_position)
        self._started_logged = False

        self.get_logger().info("FULL MAP CLEANER: bringing up…")

    # --- helpers ---
    def _set_pose(self, idx, msg): self.poses[idx] = msg

    def _wait(self, cli, name):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name}…')

    def kill(self, name):
        self._wait(self.kill_cli, '/kill')
        req = Kill.Request(); req.name = name
        self.kill_cli.call_async(req)

    def spawn(self, name, x, y, theta):
        self._wait(self.spawn_cli, '/spawn')
        req = Spawn.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        req.name = name
        self.spawn_cli.call_async(req)

    def teleport(self, name, x, y, theta):
        cli = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
        self._wait(cli, f'/{name}/teleport_absolute')
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        cli.call_async(req)

    # --- initial placement (called once) ---
    def spawn_and_position(self):
        # cancel this timer so it runs once
        try: self._init_timer.cancel()
        except Exception: pass

        # standardize turtles
        self.kill('turtle1')
        self.spawn('turtle1', 5.5, 5.5, 0.0)
        self.spawn('turtle2', 5.5, 5.5, math.pi/2)
        self.spawn('turtle3', 5.5, 5.5, 0.0)
        self.spawn('turtle4', 5.5, 5.5, math.pi/2)

        # move each to its quadrant start
        self.teleport('turtle1', self.limits[0]["x_min"], self.limits[0]["y_max"], 0.0)           # Q1
        self.teleport('turtle2', 5.5,                       self.limits[1]["y_min"], math.pi/2)   # Q2 edge
        self.teleport('turtle3', self.limits[2]["x_max"],   self.limits[2]["y_min"], math.pi)     # Q3 edge
        self.teleport('turtle4', 0.0,                       self.limits[3]["y_min"], math.pi/2)   # Q4 edge

        if not self._started_logged:
            self.get_logger().info("FULL MAP CLEANER STARTED!")
            self._started_logged = True

    # --- control loop ---
    def loop(self):
        if any(p is None for p in self.poses):
            return

        for i in range(4):
            if not self.states[i]["complete"]:
                self.control_turtle(i)

        if all(s["complete"] for s in self.states):
            self.get_logger().info("🎉 ALL QUADRANTS COMPLETELY CLEANED! 🎉")
            self.timer.cancel()

    def control_turtle(self, i):
        p = self.poses[i]; s = self.states[i]; l = self.limits[i]
        msg = Twist()

        if i % 2 == 0:
            # even: sweep horizontally across x, stepping rows in y
            target_x = l["x_max"] if s["moving"] else l["x_min"]
            target_y = l["y_max"] - (s["row"] * l["inc"])
            if (p.x >= l["x_max"] - 0.2 and s["moving"]) or (p.x <= l["x_min"] + 0.2 and not s["moving"]):
                if s["row"] >= self.max_rows[i]:
                    self.pubs[i].publish(Twist()); s["complete"] = True
                    self.get_logger().info(f"✅ Turtle{i+1}: Quadrant COMPLETE!")
                    return
                s["row"] += 1; s["moving"] = not s["moving"]
        else:
            # odd: sweep vertically across y, stepping columns in x
            target_x = l["x_min"] + (s["row"] * l["inc"])
            target_y = l["y_max"] if s["moving"] else l["y_min"]
            if (p.y >= l["y_max"] - 0.2 and s["moving"]) or (p.y <= l["y_min"] + 0.2 and not s["moving"]):
                if s["row"] >= self.max_rows[i]:
                    self.pubs[i].publish(Twist()); s["complete"] = True
                    self.get_logger().info(f"✅ Turtle{i+1}: Quadrant COMPLETE!")
                    return
                s["row"] += 1; s["moving"] = not s["moving"]

        # P-control towards target
        angle_err = self.angle_err(p.x, p.y, p.theta, target_x, target_y)
        dist_err  = math.hypot(target_x - p.x, target_y - p.y)
        msg.linear.x  = min(2.6, dist_err * 1.8)
        msg.angular.z = 4.2 * angle_err
        self.pubs[i].publish(msg)

    @staticmethod
    def angle_err(x, y, theta, tx, ty):
        a = math.atan2(ty - y, tx - x) - theta
        while a > math.pi:  a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = FullMapCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
