#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DoubleSweepRoomba(Node):
    def __init__(self):
        super().__init__('double_sweep_roomba')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # QoS: turtlesim pose is Best Effort
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, qos)

        # faster loop (20 Hz)
        self.timer = self.create_timer(0.05, self.loop)
        self.pose = None

        # --- original FSM, but tighter/faster defaults ---
        self.phase = "HORIZONTAL_SWEEP"
        self.state = "GO_TO_START"

        # tighter spacing
        self.sweep_inc = 0.5

        # recompute rows/cols from spacing (map ~1..10)
        self.max_rows = int((10.0 - 1.0) / self.sweep_inc)  # 18 for 0.5
        self.max_cols = int((10.0 - 1.0) / self.sweep_inc)

        self.row = 0
        self.col = 0

        self.h_start = (1.0, 10.0)
        self.v_start = (1.0, 10.0)
        self.target_x = 1.0
        self.target_y = 10.0
        self.right = True
        self.down = True

        # speed tuning
        self.k_lin = 2.0    # was ~1.5 equivalent
        self.k_ang = 5.0    # stronger turn-in
        self.v_max = 3.2    # was 2.0
        self.w_max = 4.5

        # tighter “wall reached” thresholds to reduce dead-zone
        self.x_right_thresh = 9.95
        self.x_left_thresh  = 1.05
        self.y_bottom_thresh = 1.05
        self.y_top_thresh    = 9.95

        self.get_logger().info(
            f"Double Sweep Roomba (FAST): inc={self.sweep_inc}, rows={self.max_rows}, cols={self.max_cols}, v_max={self.v_max}"
        )

    def pose_cb(self, msg):
        self.pose = msg

    def loop(self):
        if not self.pose:
            return
        x, y, th = self.pose.x, self.pose.y, self.pose.theta
        msg = Twist()

        if self.phase == "HORIZONTAL_SWEEP":
            self.horizontal(x, y)
        elif self.phase == "VERTICAL_SWEEP":
            self.vertical(x, y)
        elif self.phase == "COMPLETE":
            self.pub.publish(msg)
            return

        angle_err = self.angle_err(x, y, th)
        dist_err = math.hypot(self.target_x - x, self.target_y - y)

        # faster but safe: cap and reduce forward speed when turning a lot
        v = min(self.v_max, dist_err * self.k_lin)
        v *= max(0.35, 1.0 - min(1.0, abs(angle_err) / 1.2))  # slow if large turn needed
        w = max(-self.w_max, min(self.w_max, self.k_ang * angle_err))

        msg.linear.x = max(0.0, v)
        msg.angular.z = w
        self.pub.publish(msg)

    def horizontal(self, x, y):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.h_start
            if self.reached(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Horizontal sweep: Starting from top-left")
        elif self.state == "SWEEPING":
            self.target_y = self.h_start[1] - (self.row * self.sweep_inc)
            if self.right:
                self.target_x = 10.0
                if x >= self.x_right_thresh:
                    if self.row >= self.max_rows:
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.row += 1
                        self.right = False
                        self.get_logger().info(f"Horizontal: Row {self.row} - Moving left")
            else:
                self.target_x = 1.0
                if x <= self.x_left_thresh:
                    if self.row >= self.max_rows:
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.row += 1
                        self.right = True
                        self.get_logger().info(f"Horizontal: Row {self.row} - Moving right")

    def vertical(self, x, y):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.v_start
            if self.reached(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Vertical sweep: Starting from top-left")
        elif self.state == "SWEEPING":
            self.target_x = self.v_start[0] + (self.col * self.sweep_inc)
            if self.down:
                self.target_y = 1.0
                if y <= self.y_bottom_thresh:
                    if self.col >= self.max_cols:
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.col += 1
                        self.down = False
                        self.get_logger().info(f"Vertical: Column {self.col} - Moving up")
            else:
                self.target_y = 10.0
                if y >= self.y_top_thresh:
                    if self.col >= self.max_cols:
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.col += 1
                        self.down = True
                        self.get_logger().info(f"Vertical: Column {self.col} - Moving down")

    def reached(self, x, y):
        return math.hypot(self.target_x - x, self.target_y - y) < 0.2

    def angle_err(self, x, y, th):
        target = math.atan2(self.target_y - y, self.target_x - x)
        return self.normalize(target - th)

    def normalize(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DoubleSweepRoomba()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
