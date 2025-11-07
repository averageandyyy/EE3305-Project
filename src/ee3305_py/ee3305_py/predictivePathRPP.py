from math import atan2, cos, hypot, sin
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def curvature_from_three_points(p1, p2, p3):
    """
    Compute curvature of circle passing through three 2D points.
    curvature = 1 / R where R is circumradius.
    Uses formula: curvature = 4 * area / (a*b*c)
    If points are collinear (area ~ 0), returns 0.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    a = hypot(x2 - x1, y2 - y1)
    b = hypot(x3 - x2, y3 - y2)
    c = hypot(x1 - x3, y1 - y3)

    # triangle area by shoelace
    area = abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))) * 0.5
    if area <= 1e-9 or a <= 1e-9 or b <= 1e-9 or c <= 1e-9:
        return 0.0

    curvature = (4.0 * area) / (a * b * c)
    # curvature might have sign info depending on orientation; compute signed curvature
    # compute sign using cross product of vectors (p2-p1) x (p3-p2)
    vx1 = x2 - x1
    vy1 = y2 - y1
    vx2 = x3 - x2
    vy2 = y3 - y2
    cross = vx1 * vy2 - vy1 * vx2
    if cross < 0:
        curvature = -curvature
    return curvature


class Controller(Node):
    def __init__(self, node_name="adaptive_predictive_pure_pursuit"):
        super().__init__(node_name)

        # Parameters (added predictive parameters)
        self.declare_parameter("frequency", 20.0)

        self.declare_parameter("min_lookahead", 0.2)
        self.declare_parameter("max_lookahead", 1.0)
        self.declare_parameter("lookahead_gain", 1.0)

        self.declare_parameter("base_lin_vel", 0.6)
        self.declare_parameter("max_lin_vel", 0.8)
        self.declare_parameter("max_ang_vel", 2.0)
        self.declare_parameter("stop_thres", 0.15)
        self.declare_parameter("curvature_slowdown_gain", 1.0)
        self.declare_parameter("goal_slowdown_distance", 0.5)
        self.declare_parameter("enable_debug_log", False)

        # predictive-specific params
        self.declare_parameter("prediction_horizon", 6)          # number of points ahead to consider
        self.declare_parameter("prediction_weight_decay", 0.8)   # how fast weights decay for farther points
        self.declare_parameter("pred_lookahead_scale", 0.6)      # reduce lookahead when predicted curvature high
        self.declare_parameter("feedforward_gain", 1.0)          # scale feedforward angular velocity

        # read params
        self.frequency_ = self.get_parameter("frequency").value
        self.min_ld_ = self.get_parameter("min_lookahead").value
        self.max_ld_ = self.get_parameter("max_lookahead").value
        self.ld_gain_ = self.get_parameter("lookahead_gain").value
        self.base_lin_vel_ = self.get_parameter("base_lin_vel").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.curvature_slowdown_gain_ = self.get_parameter("curvature_slowdown_gain").value
        self.goal_slowdown_distance_ = self.get_parameter("goal_slowdown_distance").value
        self.enable_debug_log_ = self.get_parameter("enable_debug_log").value

        # predictive params
        self.prediction_horizon_ = int(self.get_parameter("prediction_horizon").value)
        self.pred_weight_decay_ = float(self.get_parameter("prediction_weight_decay").value)
        self.pred_lookahead_scale_ = float(self.get_parameter("pred_lookahead_scale").value)
        self.feedforward_gain_ = float(self.get_parameter("feedforward_gain").value)

        # Subscriptions & publishers
        self.sub_path_ = self.create_subscription(Path, "path", self.callbackSubPath_, 10)
        self.sub_odom_ = self.create_subscription(Odometry, "odom", self.callbackSubOdom_, 10)
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.pub_lookahead_ = self.create_publisher(PoseStamped, "lookahead", 10)

        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # State
        self.received_path_ = False
        self.received_odom_ = False
        self.path_poses_ = []

    def callbackSubPath_(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty path!")
            return
        self.path_poses_ = msg.poses.copy()
        self.received_path_ = True
        self.get_logger().info(f"Received path with {len(self.path_poses_)} poses")

    def callbackSubOdom_(self, msg: Odometry):
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rbt_yaw_ = atan2(siny_cosp, cosy_cosp)

        self.rbt_speed_ = msg.twist.twist.linear.x
        self.received_odom_ = True

    def computeAdaptiveLookahead_(self):
        Ld = self.min_ld_ + self.ld_gain_ * abs(self.rbt_speed_)
        return max(self.min_ld_, min(self.max_ld_, Ld))

    def compute_predicted_curvature_(self, closest_idx):
        """
        Compute a weighted average predicted curvature using curvature_from_three_points
        across the next 'prediction_horizon_' points (triples).
        Returns a signed curvature (can be negative for left/right).
        """
        n = len(self.path_poses_)
        if n < 3 or closest_idx >= n - 2:
            return 0.0

        total_weight = 0.0
        weighted_curv = 0.0
        # We compute curvature using triples (i, i+1, i+2) for i in [closest_idx .. closest_idx + horizon - 2]
        max_i = min(n - 3, closest_idx + self.prediction_horizon_ - 1)
        for j, i in enumerate(range(closest_idx, max_i + 1)):
            p1 = self.path_poses_[i].pose.position
            p2 = self.path_poses_[i + 1].pose.position
            p3 = self.path_poses_[i + 2].pose.position
            k = curvature_from_three_points((p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y))
            # weight closer triples more strongly; geometric decay
            weight = (self.pred_weight_decay_ ** j)
            total_weight += weight
            weighted_curv += weight * k

        if total_weight <= 0:
            return 0.0
        return weighted_curv / total_weight

    def get_closest_index_(self):
        """Return index of the closest path pose and its distance."""
        if not self.path_poses_:
            return None, None
        closest_dist = float("inf")
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i
        return closest_idx, closest_dist

    def getLookaheadPoint_(self):
        if not self.path_poses_:
            return None, None

        closest_idx, _ = self.get_closest_index_()
        if closest_idx is None:
            return None, None

        target_ld = self.computeAdaptiveLookahead_()
        lookahead_idx = len(self.path_poses_) - 1
        for i in range(closest_idx, len(self.path_poses_)):
            px = self.path_poses_[i].pose.position.x
            py = self.path_poses_[i].pose.position.y
            d = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if d >= target_ld:
                lookahead_idx = i
                break

        pose = self.path_poses_[lookahead_idx]
        lx = pose.pose.position.x
        ly = pose.pose.position.y

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = lx
        msg.pose.position.y = ly
        self.pub_lookahead_.publish(msg)

        return lx, ly, closest_idx

    def callbackTimer_(self):
        if not (self.received_path_ and self.received_odom_):
            return

        res = self.getLookaheadPoint_()
        if res is None:
            return
        lx, ly, closest_idx = res

        # Transform target to robot frame
        dx = lx - self.rbt_x_
        dy = ly - self.rbt_y_
        local_x = dx * cos(self.rbt_yaw_) + dy * sin(self.rbt_yaw_)
        local_y = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)

        dist = hypot(local_x, local_y)

        # determine curvature at lookahead (geometric) — avoid division by zero
        denom = max(1e-6, local_x ** 2 + local_y ** 2)
        curvature_lookahead = (2.0 * local_y) / denom

        # predictive curvature from upcoming segments
        curvature_pred = self.compute_predicted_curvature_(closest_idx)

        # blend or combine curvature: choose the one with larger magnitude (conservative),
        # but preserve sign. Alternatively, you could blend: blended = alpha*curv_look + (1-alpha)*curv_pred
        blended_curvature = curvature_lookahead
        if abs(curvature_pred) > abs(curvature_lookahead):
            blended_curvature = curvature_pred

        # optionally shrink lookahead proactively if predicted curvature is large
        # scale factor between (pred_lookahead_scale_, 1.0)
        k = min(1.0, max(0.0, 1.0 - self.pred_lookahead_scale_ * min(1.0, abs(curvature_pred))))
        adaptive_ld = max(self.min_ld_, min(self.max_ld_, self.min_ld_ + self.ld_gain_ * abs(self.rbt_speed_) * k))

        # velocity regulation uses blended curvature (predictive effect)
        curve_factor = 1.0 / (1.0 + self.curvature_slowdown_gain_ * abs(blended_curvature))
        final_goal_x = self.path_poses_[-1].pose.position.x
        final_goal_y = self.path_poses_[-1].pose.position.y
        dist_to_goal = hypot(final_goal_x - self.rbt_x_, final_goal_y - self.rbt_y_)
        goal_factor = min(1.0, dist_to_goal / self.goal_slowdown_distance_)

        # base linear speed
        lin_vel = self.base_lin_vel_ * curve_factor * goal_factor
        lin_vel = min(lin_vel, self.max_lin_vel_)

        # feedforward angular velocity from predicted curvature to reduce lag
        ang_feedforward = self.feedforward_gain_ * lin_vel * curvature_pred

        # feedback angular velocity from current lookahead geometry
        ang_feedback = lin_vel * curvature_lookahead

        # combine (you may tune weights)
        ang_vel_raw = 0.6 * ang_feedback + 0.4 * ang_feedforward

        # clamp angular velocity
        ang_vel = max(-self.max_ang_vel_, min(self.max_ang_vel_, ang_vel_raw))

        # stop condition
        if dist < self.stop_thres_:
            lin_vel = 0.0
            ang_vel = 0.0

        # publish
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = lin_vel
        cmd.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(cmd)

        if self.enable_debug_log_:
            self.get_logger().info(
                f"ld={adaptive_ld:.2f}, curv_look={curvature_lookahead:.3f}, curv_pred={curvature_pred:.3f}, "
                f"v={lin_vel:.2f}, w={ang_vel:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
