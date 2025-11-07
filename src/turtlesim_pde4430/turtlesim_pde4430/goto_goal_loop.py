#!/usr/bin/env python3
import math, threading, queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def wrap(a):  # -> (-pi, pi]
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class GotoGoalLoop(Node):
    """
    Interactive go-to-goal loop:
      - prompts for (x, y)
      - drives to goal with P control + slowdown
      - when reached, prompts again (infinite loop)
    """
    def __init__(self):
        super().__init__('goto_goal_loop')

        # Tunables (can override with --ros-args -p key:=val)
        self.declare_parameter('k_lin', 1.2)
        self.declare_parameter('k_ang', 5.0)
        self.declare_parameter('v_max', 2.4)
        self.declare_parameter('w_max', 3.2)
        self.declare_parameter('tolerance', 0.12)
        self.declare_parameter('slowdown_radius', 1.0)

        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.tol   = float(self.get_parameter('tolerance').value)
        self.slowR = float(self.get_parameter('slowdown_radius').value)

        # Pose subscriber (turtlesim publishes Best Effort)
        qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, qos)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Goal state
        self.goal = None                # (x, y) or None
        self.goal_q: "queue.Queue[tuple[float,float]]" = queue.Queue()
        self.ready_for_next = threading.Event()
        self.ready_for_next.set()       # ask immediately at start

        # Control timer
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)

        # Background prompt thread (non-blocking for ROS)
        self.thread = threading.Thread(target=self._prompt_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("goto_goal_loop ready. I'll ask you for (x,y) goals in the terminal.")

    # --------------- IO thread ---------------
    def _prompt_loop(self):
        while rclpy.ok():
            # wait until controller signals it's ready for a new goal
            self.ready_for_next.wait()
            try:
                xs = input("Enter goal x (0..11): ").strip()
                ys = input("Enter goal y (0..11): ").strip()
                x = float(xs); y = float(ys)
                self.goal_q.put((x, y))
                self.ready_for_next.clear()  # controller will re-set when goal is reached
            except Exception:
                print("Invalid input. Example: 8.5  then  9.0")

    # --------------- ROS callbacks ---------------
    def on_pose(self, msg: Pose):
        self.pose = msg

    def tick(self):
        # fetch next goal if none
        if self.goal is None and not self.goal_q.empty():
            self.goal = self.goal_q.get()
            self.get_logger().info(f"New goal: {self.goal}")

        if self.pose is None:
            # publish idle until we have pose
            self.pub.publish(Twist()); 
            return

        if self.goal is None:
            # waiting for user; hold position
            self.pub.publish(Twist())
            return

        gx, gy = self.goal
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < self.tol:
            # reached! stop and ask for next
            self.pub.publish(Twist())
            self.get_logger().info(f"Reached {self.goal}.")
            self.goal = None
            self.ready_for_next.set()
            return

        # Heading to goal
        head_des = math.atan2(dy, dx)
        head_err = wrap(head_des - self.pose.theta)

        # Smooth slowdown near goal
        v_des = self.v_max * (1.0 - math.exp(-max(0.0, dist) / max(1e-3, self.slowR)))
        v_cmd = min(self.v_max, max(0.0, self.k_lin * dist, v_des))

        # Turn rate
        w_cmd = max(-self.w_max, min(self.w_max, self.k_ang * head_err))

        # Reduce forward speed when heading error is large
        v_cmd *= max(0.25, 1.0 - min(1.0, abs(head_err) / 1.2))

        msg = Twist(); msg.linear.x = v_cmd; msg.angular.z = w_cmd
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GotoGoalLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
