from math import atan2, cos, hypot, inf, sin, exp


import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node




class Controller(Node):


    def __init__(self, node_name="regulated_pure_pursuit"):
        super().__init__(node_name)


        # Parameters: Declare ============================================================
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.4))
        self.declare_parameter("base_lin_vel", float(0.15))
        self.declare_parameter("max_lin_vel", float(0.3))
        self.declare_parameter("max_ang_vel", float(2.0))
        self.declare_parameter("stop_thres", float(0.15))
        self.declare_parameter("curvature_slowdown_gain", float(0.8))
        self.declare_parameter("goal_slowdown_distance", float(0.5))
        self.declare_parameter("enable_debug_log", False)


        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.base_lin_vel_ = self.get_parameter("base_lin_vel").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.curvature_slowdown_gain_ = self.get_parameter("curvature_slowdown_gain").value
        self.goal_slowdown_distance_ = self.get_parameter("goal_slowdown_distance").value
        self.enable_debug_log_ = self.get_parameter("enable_debug_log").value


        # Subscribers ============================================================
        self.sub_path_ = self.create_subscription(Path, "path", self.callbackSubPath_, 10)
        self.sub_odom_ = self.create_subscription(Odometry, "odom", self.callbackSubOdom_, 10)


        # Publishers ============================================================
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.pub_lookahead_ = self.create_publisher(PoseStamped, "lookahead", 10)


        # Timer ============================================================
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)


        # State Variables ============================================================
        self.received_path_ = False
        self.received_odom_ = False
        self.path_poses_ = []


    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:
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
        self.received_odom_ = True


    # =========================================================================
    # PURE PURSUIT LOGIC
    # =========================================================================
    def getLookaheadPoint_(self):
        if not self.path_poses_:
            return None, None


        # find closest point
        closest_dist = inf
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i


        # find lookahead
        lookahead_idx = len(self.path_poses_) - 1
        for i in range(closest_idx, len(self.path_poses_)):
            px = self.path_poses_[i].pose.position.x
            py = self.path_poses_[i].pose.position.y
            d = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if d >= self.lookahead_distance_:
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


        return lx, ly


    def callbackTimer_(self):
        if not self.received_path_ or not self.received_odom_:
            return


        lookahead_x, lookahead_y = self.getLookaheadPoint_()
        if lookahead_x is None:
            return


        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        local_x = dx * cos(self.rbt_yaw_) + dy * sin(self.rbt_yaw_)
        local_y = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)
        dist = hypot(local_x, local_y)


        # stop condition
        if dist < self.stop_thres_:
            lin_vel = 0.0
            ang_vel = 0.0
        else:
            # curvature (avoid division by zero)
            denom = max(1e-6, local_x ** 2 + local_y ** 2)
            curvature = (2.0 * local_y) / denom


            # === Velocity Regulation ===
            # base speed reduced by curvature and goal proximity
            curve_factor = 1.0 / (1.0 + self.curvature_slowdown_gain_ * abs(curvature))
            dist_to_goal = hypot(
                self.path_poses_[-1].pose.position.x - self.rbt_x_,
                self.path_poses_[-1].pose.position.y - self.rbt_y_,
            )
            goal_factor = min(1.0, dist_to_goal / self.goal_slowdown_distance_)


            lin_vel = self.base_lin_vel_ * curve_factor * goal_factor
            lin_vel = min(lin_vel, self.max_lin_vel_)


            # angular velocity from curvature
            ang_vel = curvature * lin_vel
            ang_vel = max(-self.max_ang_vel_, min(self.max_ang_vel_, ang_vel))


        # publish command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = lin_vel
        msg.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg)


        if self.enable_debug_log_:
            self.get_logger().info(
                f"ld={dist:.2f}, curv={curvature:.3f}, v={lin_vel:.2f}, w={ang_vel:.2f}"
            )




# =========================================================================
# MAIN
# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()




if __name__ == "__main__":
    main()