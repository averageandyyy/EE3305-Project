from math import atan2, cos, hypot, sin


import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node




class Controller(Node):
    def __init__(self, node_name="adaptive_pure_pursuit"):
        super().__init__(node_name)


        # Parameters
        self.declare_parameter("frequency", 20.0)


        # ✅ Adaptive lookahead settings
        self.declare_parameter("min_lookahead", 0.3)     # smallest lookahead (m)
        self.declare_parameter("max_lookahead", 0.6)     # largest lookahead (m)
        self.declare_parameter("lookahead_gain", 1.0)    # scaling factor * speed


        self.declare_parameter("base_lin_vel", 0.2)
        self.declare_parameter("max_lin_vel", 0.4)
        self.declare_parameter("max_ang_vel", 2.0)
        self.declare_parameter("stop_thres", 0.1)
        self.declare_parameter("curvature_slowdown_gain", 0.8) # the higher the more slowdown
        self.declare_parameter("goal_slowdown_distance", 0.2) # the higher the closer the robot slow down
        self.declare_parameter("enable_debug_log", True)

        self.declare_parameter("rotate_threshold", 0.785)      # 45 deg

        self.declare_parameter("rotate_tolerance", 0.5)       # stop rotating if 11 deg
        self.declare_parameter("rotate_speed", 0.5)            # rad/s
        self.declare_parameter("rotate_gain", 1)             # angular speed scaling factor

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
        self.rotate_threshold_ = self.get_parameter("rotate_threshold").value
        self.rotate_tolerance_ = self.get_parameter("rotate_tolerance").value
        self.rotate_speed_ = self.get_parameter("rotate_speed").value
        self.rotate_gain_ = self.get_parameter("rotate_gain").value

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


    # -------------------- Callbacks --------------------------------
    def callbackSubPath_(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty path!")
            return
        self.path_poses_ = msg.poses.copy()
        self.received_path_ = True
        self.get_logger().info(f"Received path with {len(self.path_poses_)} poses")

        goal_pose = self.path_poses_[-1].pose
        # extract orientation quaternion
        q = goal_pose.orientation
        self.goal_yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def callbackSubOdom_(self, msg: Odometry):
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y


        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rbt_yaw_ = atan2(siny_cosp, cosy_cosp)


        # ✅ Robot's forward speed (used for adaptive lookahead)
        self.rbt_speed_ = msg.twist.twist.linear.x
        self.received_odom_ = True


    # -------------------- Adaptive Lookahead (only change here) ----
    def computeAdaptiveLookahead_(self):
        Ld = self.min_ld_ + self.ld_gain_ * abs(self.rbt_speed_)
        return max(self.min_ld_, min(self.max_ld_, Ld))


    def getLookaheadPoint_(self):
        if not self.path_poses_:
            return None, None


        # 1. Find closest path point
        closest_dist = float("inf")
        closest_idx = 0
        for i, pose in enumerate(self.path_poses_):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = hypot(px - self.rbt_x_, py - self.rbt_y_)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i


        # 2. Use adaptive lookahead distance
        target_ld = self.computeAdaptiveLookahead_()


        # 3. Find first point at or beyond lookahead distance
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


        # Publish lookahead point for visualization
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = lx
        msg.pose.position.y = ly
        self.pub_lookahead_.publish(msg)


        return lx, ly


    # -------------------- Main Controller Loop ----------------------
    def callbackTimer_(self):
        if not (self.received_path_ and self.received_odom_):
            return


        lx, ly = self.getLookaheadPoint_()
        if lx is None:
            return


        # Transform target to robot frame
        dx = lx - self.rbt_x_
        dy = ly - self.rbt_y_
        local_x = dx * cos(self.rbt_yaw_) + dy * sin(self.rbt_yaw_)
        local_y = -dx * sin(self.rbt_yaw_) + dy * cos(self.rbt_yaw_)


        dist = hypot(local_x, local_y)

        PP_heading_error = atan2(local_y, local_x)
   

        if abs(PP_heading_error) < 0.785:  # 45 degrees
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
        else:
            # Too far off heading → rotate on the spot
            lin_vel = 0.0

            # Angular velocity proportional to heading_error, limited by max_ang_vel_
            ang_vel = self.rotate_gain_ * PP_heading_error
            ang_vel = max(-self.max_ang_vel_, min(self.max_ang_vel_, ang_vel))
            self.get_logger().info(abs(PP_heading_error))



        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = lin_vel
        cmd.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(cmd)


        if self.enable_debug_log_:
            self.get_logger().info(
                f"ld={dist:.2f}, curv={curvature:.3f}, v={lin_vel:.2f}, w={ang_vel:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()




if __name__ == "__main__":
    main()
