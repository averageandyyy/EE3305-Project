from math import atan2, cos, hypot, inf, sin

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    qos_profile_sensor_data,
    qos_profile_services_default,
)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from ee3305_py.dwa_planner import DWALocalPlanner
from ee3305_py.pure_pursuit_controller import PurePursuitController
from ee3305_py.RPP_controller import RPPController


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))
        # RPP Controller Parameters
        self.declare_parameter("min_lookahead", 0.3)  # smallest lookahead (m)
        self.declare_parameter("max_lookahead", 0.7)  # largest lookahead (m)
        self.declare_parameter("lookahead_gain", 1.0)  # scaling factor * speed
        self.declare_parameter("base_lin_vel", 0.2)
        self.declare_parameter(
            "curvature_slowdown_gain", 0.4
        )  # the higher the more slowdown
        self.declare_parameter(
            "goal_slowdown_distance", 0.3
        )  # the higher the closer the robot slow down
        self.declare_parameter("enable_debug_log", True)
        self.declare_parameter("rotate_threshold", 0.785)  # 45 deg
        self.declare_parameter("rotate_tolerance", 0.2)  # stop rotating if 11 deg
        self.declare_parameter("rotate_speed", 0.5)  # rad/s
        self.declare_parameter("rotate_gain", 1)  # angular speed scaling factor

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value
        self.min_ld_ = self.get_parameter("min_lookahead").value
        self.max_ld_ = self.get_parameter("max_lookahead").value
        self.ld_gain_ = self.get_parameter("lookahead_gain").value
        self.base_lin_vel_ = self.get_parameter("base_lin_vel").value
        self.curvature_slowdown_gain_ = self.get_parameter(
            "curvature_slowdown_gain"
        ).value
        self.goal_slowdown_distance_ = self.get_parameter(
            "goal_slowdown_distance"
        ).value
        self.enable_debug_log_ = self.get_parameter("enable_debug_log").value
        self.rotate_threshold_ = self.get_parameter("rotate_threshold").value
        self.rotate_tolerance_ = self.get_parameter("rotate_tolerance").value
        self.rotate_speed_ = self.get_parameter("rotate_speed").value
        self.rotate_gain_ = self.get_parameter("rotate_gain").value

        # Handles: Topic Subscribers
        # !TODO: path subscriber
        self.sub_path_ = self.create_subscription(
            Path,
            "path",
            self.callbackSubPath_,
            10,
        )

        # !TODO: odometry subscriber
        self.sub_odom_ = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            10,
        )

        # Handles: Topic Publishers
        # !TODO: command velocities publisher
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped,
            "cmd_vel",
            10,
        )

        # !TODO: lookahead point publisher
        self.pub_look_ahead_ = self.create_publisher(
            PoseStamped,
            "lookahead",
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False

        # Testing variables
        self.enable_controls_ = True
        self.using_dwa = False
        self.using_RPP = True
        self.trajectories_publisher = self.create_publisher(
            MarkerArray,
            "/trajectories",
            10,
        )

        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        # Add costmap subscriber for DWA planner
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # Pure Pursuit Controller
        self.pure_pursuit_controller_ = PurePursuitController(
            stop_threshold=self.stop_thres_,
            lookahead_linear_velocity=self.lookahead_lin_vel_,
            max_linear_velocity=self.max_lin_vel_,
            max_angular_velocity=self.max_ang_vel_,
        )

        self.RPP_controller_ = RPPController(
            min_lookahead=self.min_ld_,
            max_lookahead=self.max_ld_,
            lookahead_gain=self.ld_gain_,
            base_linear_velocity=self.base_lin_vel_,
            max_linear_velocity=self.max_lin_vel_,
            max_angular_velocity=self.max_ang_vel_,
            curvature_slowdown_gain=self.curvature_slowdown_gain_,
            goal_slowdown_distance=self.goal_slowdown_distance_,
            stop_threshold=self.stop_thres_,
            rotate_threshold=self.rotate_threshold_,
            rotate_tolerance=self.rotate_tolerance_,
            rotate_speed=self.rotate_speed_,
            rotate_gain=self.rotate_gain_,
        )


    # Callbacks =============================================================

    # Occupancy grid subscriber callback for DWA planner
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):

        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        self.costmap_ = msg.data
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.costmap_max_access_cost_ = 90  # Hardcoded for now

        self.dwa_planner_ = DWALocalPlanner(
            costmap=self.costmap_,
            origin_x=self.costmap_origin_x_,
            origin_y=self.costmap_origin_y_,
            resolution=self.costmap_resolution_,
            columns=self.costmap_cols_,
            rows=self.costmap_rows_,
            max_access_cost=self.costmap_max_access_cost_,
            max_linear_velocity=self.max_lin_vel_,
            max_angular_velocity=self.max_ang_vel_,
        )

        self.received_map_ = True

    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:  # not msg.poses is fine but not clear
            self.get_logger().warn(f"Received path message is empty!")
            return  # do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.

        # !TODO: copy the array from the path
        self.path_poses_ = msg.poses.copy()
        self.received_path_ = True

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        # !TODO: write robot pose to rbt_x_, rbt_y_, rbt_yaw_
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        delta_x = 2 * (q.w * q.z + q.x * q.y)
        delta_y = 1 - 2 * (q.y * q.y + q.z * q.z)
        phi = atan2(delta_x, delta_y)
        self.rbt_yaw_ = phi

        # Added velocity variables
        self.rbt_linear_velocity_ = msg.twist.twist.linear.x
        self.rbt_angular_velocity_ = msg.twist.twist.angular.z

        self.received_odom_ = True

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot
        closest_dist = inf
        closest_idx = 0

        for i, pose in enumerate(self.path_poses_):
            curr_x = pose.pose.position.x
            curr_y = pose.pose.position.y
            curr_dist = hypot(curr_x - self.rbt_x_, curr_y - self.rbt_y_)

            if curr_dist < closest_dist:
                closest_dist = curr_dist
                closest_idx = i

        # From the closest point, proceed towards the goal and find the lookahead point
        lookahead_idx = len(self.path_poses_) - 1  # Default to goal point

        for i in range(closest_idx, len(self.path_poses_)):
            pose = self.path_poses_[i]
            curr_x = pose.pose.position.x
            curr_y = pose.pose.position.y
            curr_dist = hypot(curr_x - self.rbt_x_, curr_y - self.rbt_y_)

            # Find the first point that is at least lookahead distance away
            if curr_dist >= self.lookahead_distance_:
                lookahead_idx = i
                break  # gotten first pt

        # Get the lookahead coordinates
        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_look_ahead_.publish(msg_lookahead)  # original code was missing "_"

        # Return the coordinates
        return lookahead_x, lookahead_y
    
    # Compute adaptive lookahead distance
    def computeAdaptiveLookahead_(self):
        Ld = self.min_ld_ + self.ld_gain_ * abs(self.rbt_linear_velocity_)
        return max(self.min_ld_, min(self.max_ld_, Ld))
    
    def get_RPP_lookahead_point_(self):
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
        self.pub_look_ahead_.publish(msg)

        return lx, ly


    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        if not self.received_odom_ or not self.received_path_ or not self.received_map_:
            return  # return silently if path or odom is not received.

        # get lookahead point as subgoal
        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        if self.using_dwa:
            best_velocity_command, trajectories, best_traj_index = (
                self.dwa_planner_.generate_best_velocity_command(
                    current_x=self.rbt_x_,
                    current_y=self.rbt_y_,
                    current_yaw=self.rbt_yaw_,
                    current_linear_velocity=self.rbt_linear_velocity_,
                    current_angular_velocity=self.rbt_angular_velocity_,
                    goal_x=lookahead_x,
                    goal_y=lookahead_y,
                )
            )
            marker_array = MarkerArray()
            for i, trajectory in enumerate(trajectories):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "trajectory"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.01  # line width
                marker.color.a = 1.0
                if i == best_traj_index:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 5.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                for point in trajectory:
                    x, y, _ = point
                    p = PoseStamped()
                    p.pose.position.x = x
                    p.pose.position.y = y
                    marker.points.append(p.pose.position)

                marker_array.markers.append(marker)

            self.trajectories_publisher.publish(marker_array)
        elif self.using_RPP:
            lookaheadRPP_x, lookaheadRPP_y = self.get_RPP_lookahead_point_()
            best_velocity_command = self.RPP_controller_.get_velocity_command(
                current_x=self.rbt_x_,
                current_y=self.rbt_y_,
                current_yaw=self.rbt_yaw_,
                lookahead_x=lookaheadRPP_x,
                lookahead_y=lookaheadRPP_y,
            )
        else:
            best_velocity_command = self.pure_pursuit_controller_.get_velocity_command(
                current_x=self.rbt_x_,
                current_y=self.rbt_y_,
                current_yaw=self.rbt_yaw_,
                lookahead_x=lookahead_x,
                lookahead_y=lookahead_y,
            )

        lin_vel, ang_vel = best_velocity_command
        self.get_logger().info(
            f"Computed velocities - Linear: {lin_vel:.3f} m/s, Angular: {ang_vel:.3f} rad/s"
        )

        # publish velocities
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel

        if self.enable_controls_:
            self.pub_cmd_vel_.publish(msg_cmd_vel)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
