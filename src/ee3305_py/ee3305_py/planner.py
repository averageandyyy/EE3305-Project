import time
from heapq import heappop, heappush
from math import atan2, floor, hypot, inf

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_services_default

from ee3305_py.a_star_planner import AStarPlanner
from ee3305_py.beizer import BezierSmoother
from ee3305_py.fast_rrt_star_planner import FastRRTStarPlanner


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        return self.g < other.g


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Bezier smoothing parameters
        self.declare_parameter("bezier_offset_frac", float(0.5))
        self.declare_parameter("bezier_samples_per_seg", int(10))
        self.declare_parameter("bezier_target_spacing", float(0.04))
        self.declare_parameter("bezier_max_points", int(80000))
        self.declare_parameter("bezier_downsample_min_dist", float(0.5))
        self.declare_parameter("bezier_downsample_angle_thresh", float(60.0))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Bezier parameters
        self.bezier_offset_frac_ = self.get_parameter("bezier_offset_frac").value
        self.bezier_samples_per_seg_ = self.get_parameter(
            "bezier_samples_per_seg"
        ).value
        self.bezier_target_spacing_ = self.get_parameter("bezier_target_spacing").value
        self.bezier_max_points_ = self.get_parameter("bezier_max_points").value
        self.bezier_downsample_min_dist_ = self.get_parameter(
            "bezier_downsample_min_dist"
        ).value
        self.bezier_downsample_angle_thresh_ = self.get_parameter(
            "bezier_downsample_angle_thresh"
        ).value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # !TODO: Path request subscriber
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10,
        )

        # Handles: Publishers
        # !TODO: Path publisher
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):

        # !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y

        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):

        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        self.costmap_ = msg.data
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.FRRTStarPlanner_ = FastRRTStarPlanner(
            self.costmap_,
            self.costmap_origin_x_,
            self.costmap_origin_y_,
            self.costmap_resolution_,
            self.costmap_cols_,
            self.costmap_rows_,
            self.max_access_cost_,
            use_costmap=False,
        )

        self.AStarPlanner_ = AStarPlanner(
            self.costmap_,
            self.costmap_origin_x_,
            self.costmap_origin_y_,
            self.costmap_resolution_,
            self.costmap_cols_,
            self.costmap_rows_,
            self.max_access_cost_,
        )

        self.BezierSmoother_ = BezierSmoother(
            self.costmap_,
            self.costmap_origin_x_,
            self.costmap_origin_y_,
            self.costmap_resolution_,
            self.costmap_cols_,
            self.costmap_rows_,
            self.max_access_cost_,
        )

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # Planners (uncomment each section based on what we want to run)

        # -------- Standard Dijkstra --------
        # self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        # self.has_new_request_ = False
        # return

        # -------- FastRRTStar --------
        # start_time = time.perf_counter()
        # path = self.FRRTStarPlanner_.make_plan(
        #     self.rbt_x_,
        #     self.rbt_y_,
        #     self.goal_x_,
        #     self.goal_y_,
        # )
        # end_time = time.perf_counter()
        # print(f"FastRRTStar planning took {end_time - start_time:.4f} seconds.")

        # if len(path) == 0:
        #     self.get_logger().warn("No Path Found!")
        # else:
        #     msg_path = Path()
        #     msg_path.header.stamp = self.get_clock().now().to_msg()
        #     msg_path.header.frame_id = "map"
        #     for node in path:
        #         pose = PoseStamped()
        #         pose.pose.position.x = node.position_x
        #         pose.pose.position.y = node.position_y
        #         msg_path.poses.append(pose)
        #     self.pub_path_.publish(msg_path)
        #     self.get_logger().info(
        #         f"Path Found from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f},{self.goal_y_:7.3f})"
        #     )
        # self.has_new_request_ = False
        # return

        # -------- A* --------
        # path = self.AStarPlanner_.make_plan(
        #     self.rbt_x_,
        #     self.rbt_y_,
        #     self.goal_x_,
        #     self.goal_y_,
        # )

        # if len(path) == 0:
        #     self.get_logger().warn("No Path Found!")
        # else:
        #     msg_path = Path()
        #     msg_path.header.stamp = self.get_clock().now().to_msg()
        #     msg_path.header.frame_id = "map"
        #     for node in path:
        #         pose = PoseStamped()
        #         pose.pose.position.x = node[0]
        #         pose.pose.position.y = node[1]
        #         msg_path.poses.append(pose)
        #     self.pub_path_.publish(msg_path)
        #     self.get_logger().info(
        #         f"Path Found from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f},{self.goal_y_:7.3f})"
        #     )
        # self.has_new_request_ = False
        # return

        # -------- Dijkstra + Bezier Smoothing --------
        start_time = time.perf_counter()
        # Get raw path from Dijkstra as list of (x, y) tuples
        raw_path = []
        nodes = self._dijkstra_get_path(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        for node in nodes:
            x, y = self.CRToXY_(node.c, node.r)
            raw_path.append((x, y))
        end_time = time.perf_counter()
        print(f"Dijkstra planning took {end_time - start_time:.4f} seconds.")

        if len(raw_path) == 0:
            self.get_logger().warn("No Path Found!")
            self.has_new_request_ = False
            return

        # Apply Bezier smoothing
        def est_yaw(pts, head=True):
            if len(pts) < 2: return None
            a, b = (pts[0], pts[1]) if head else (pts[-2], pts[-1])
            return atan2(b[1]-a[1], b[0]-a[0])

        # estimates the yaw at the start and end of the planned path
        # (give dir of where it should face initially and at the end)
        start_yaw = est_yaw(raw_path, head=True)
        end_yaw = est_yaw(raw_path, head=False)

        smoothed = self.BezierSmoother_.smooth(
            raw_pts=raw_path,
            offset_frac=self.bezier_offset_frac_,
            samples_per_seg=self.bezier_samples_per_seg_,
            start_yaw=start_yaw,
            end_yaw=end_yaw,
            target_spacing=self.bezier_target_spacing_,
            max_points=self.bezier_max_points_,
            min_dist=self.bezier_downsample_min_dist_,
            angle_thresh_deg=self.bezier_downsample_angle_thresh_
        )

        # Publish smoothed path
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"
        for (x, y) in smoothed:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            msg_path.poses.append(pose)
        self.pub_path_.publish(msg_path)
        self.get_logger().info(
            f"Path Found (Dijkstra + Bezier) from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f},{self.goal_y_:7.3f})"
        )

        # -------- A* + Bezier Smoothing --------
        # start_time = time.perf_counter()
        # raw_path = self.AStarPlanner_.make_plan(
        #     self.rbt_x_,
        #     self.rbt_y_,
        #     self.goal_x_,
        #     self.goal_y_,
        # )
        # end_time = time.perf_counter()
        # print(f"A* planning took {end_time - start_time:.4f} seconds.")

        # if len(raw_path) == 0:
        #     self.get_logger().warn("No Path Found!")
        #     self.has_new_request_ = False
        #     return

        # # Apply Bezier smoothing
        # def est_yaw(pts, head=True):
        #     if len(pts) < 2: return None
        #     a, b = (pts[0], pts[1]) if head else (pts[-2], pts[-1])
        #     return atan2(b[1]-a[1], b[0]-a[0])

        # start_yaw = est_yaw(raw_path, head=True)
        # end_yaw = est_yaw(raw_path, head=False)

        # smoothed = self.BezierSmoother_.smooth(
        #     raw_pts=raw_path,
        #     offset_frac=self.bezier_offset_frac_,
        #     samples_per_seg=self.bezier_samples_per_seg_,
        #     start_yaw=start_yaw,
        #     end_yaw=end_yaw,
        #     target_spacing=self.bezier_target_spacing_,
        #     max_points=self.bezier_max_points_,
        #     min_dist=self.bezier_downsample_min_dist_,
        #     angle_thresh_deg=self.bezier_downsample_angle_thresh_
        # )

        # # Publish smoothed path
        # msg_path = Path()
        # msg_path.header.stamp = self.get_clock().now().to_msg()
        # msg_path.header.frame_id = "map"
        # for (x, y) in smoothed:
        #     pose = PoseStamped()
        #     pose.pose.position.x = float(x)
        #     pose.pose.position.y = float(y)
        #     msg_path.poses.append(pose)
        # self.pub_path_.publish(msg_path)
        # self.get_logger().info(
        #     f"Path Found (A* + Bezier) from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f},{self.goal_y_:7.3f})"
        # )

        # -------- FastRRTStar + Bezier Smoothing --------
        # start_time = time.perf_counter()
        # path = self.FRRTStarPlanner_.make_plan(
        #     self.rbt_x_,
        #     self.rbt_y_,
        #     self.goal_x_,
        #     self.goal_y_,
        # )
        # end_time = time.perf_counter()
        # print(f"FastRRTStar planning took {end_time - start_time:.4f} seconds.")

        # if len(path) == 0:
        #     self.get_logger().warn("No Path Found!")
        #     self.has_new_request_ = False
        #     return

        # # Convert RRT* nodes to (x,y) tuples
        # raw_path = [(n.position_x, n.position_y) for n in path]

        # # Apply Bezier smoothing
        # def est_yaw(pts, head=True):
        #     if len(pts) < 2: return None
        #     a, b = (pts[0], pts[1]) if head else (pts[-2], pts[-1])
        #     return atan2(b[1]-a[1], b[0]-a[0])

        # start_yaw = est_yaw(raw_path, head=True)
        # end_yaw = est_yaw(raw_path, head=False)

        # smoothed = self.BezierSmoother_.smooth(
        #     raw_pts=raw_path,
        #     offset_frac=self.bezier_offset_frac_,
        #     samples_per_seg=self.bezier_samples_per_seg_,
        #     start_yaw=start_yaw,
        #     end_yaw=end_yaw,
        #     target_spacing=self.bezier_target_spacing_,
        #     max_points=self.bezier_max_points_,
        #     min_dist=self.bezier_downsample_min_dist_,
        #     angle_thresh_deg=self.bezier_downsample_angle_thresh_
        # )

        # # Publish smoothed path
        # msg_path = Path()
        # msg_path.header.stamp = self.get_clock().now().to_msg()
        # msg_path.header.frame_id = "map"
        # for (x, y) in smoothed:
        #     pose = PoseStamped()
        #     pose.pose.position.x = float(x)
        #     pose.pose.position.y = float(y)
        #     msg_path.poses.append(pose)
        # self.pub_path_.publish(msg_path)
        # self.get_logger().info(
        #     f"Path Found (FastRRTStar + Bezier) from Rbt @ ({self.rbt_x_:7.3f}, {self.rbt_y_:7.3f}) to Goal @ ({self.goal_x_:7.3f},{self.goal_y_:7.3f})"
        # )

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        c = floor((x - self.costmap_origin_x_) / self.costmap_resolution_ - 0.5)
        r = floor((y - self.costmap_origin_y_) / self.costmap_resolution_ - 0.5)

        return c, r

    # Converts cell column and cell row to world coordinates.
    def CRToXY_(self, c, r):
        # Columns correspond to  x and rows correspond to y
        x = self.costmap_origin_x_ + (0.5 + c) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (0.5 + r) * self.costmap_resolution_

        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        return r * self.costmap_cols_ + c

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        return c < 0 or c >= self.costmap_cols_ or r < 0 or r >= self.costmap_rows_

    # Helper function: Returns path nodes from Dijkstra without publishing (for smoothing)
    def _dijkstra_get_path(self, start_x, start_y, goal_x, goal_y):
        # Initializations ---------------------------------
        nodes = [
            DijkstraNode(c, r)
            for r in range(self.costmap_rows_)
            for c in range(self.costmap_cols_)
        ]

        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        nodes[rbt_idx].g = 0
        start_node = nodes[rbt_idx]

        open_list = []
        heappush(open_list, start_node)
        visited = set()

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:
            node = heappop(open_list)

            if self.CRToIndex_(node.c, node.r) in visited:
                continue

            visited.add(self.CRToIndex_(node.c, node.r))

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                path = []
                while node.parent != None:
                    path.append(node)
                    node = node.parent
                path.reverse()
                return path

            # Neighbor Loop
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                nb_c = dc + node.c
                nb_r = dr + node.r
                nb_idx = self.CRToIndex_(nb_c, nb_r)

                if self.outOfMap_(nb_c, nb_r):
                    continue

                nb_node = nodes[nb_idx]

                if nb_idx in visited:
                    continue

                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                nb_x, nb_y = self.CRToXY_(nb_c, nb_r)
                node_x, node_y = self.CRToXY_(node.c, node.r)
                new_g = node.g + hypot(nb_x - node_x, nb_y - node_y) * (
                    self.costmap_[nb_idx] + 1
                )

                if new_g < nb_node.g:
                    nb_node.g = new_g
                    nb_node.parent = node
                    heappush(open_list, nb_node)

        return []  # No path found

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        start_time = time.perf_counter()
        # Initializations ---------------------------------

        # Initialize nodes
        # Initialize all nodes with infinite cost and no parents by default
        nodes = [
            DijkstraNode(c, r)
            for r in range(self.costmap_rows_)
            for c in range(self.costmap_cols_)
        ]

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        nodes[rbt_idx].g = 0
        start_node = nodes[rbt_idx]

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Create visited set
        visited = set()

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if visited
            if self.CRToIndex_(node.c, node.r) in visited:
                continue

            # Mark node as visited using index as unique identifier
            visited.add(self.CRToIndex_(node.c, node.r))

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                end_time = time.perf_counter()
                print(
                    f"Dijkstra's algorithm took {end_time - start_time:.4f} seconds. with visited {len(visited)} nodes."
                )
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                # obtain the path from the nodes.
                path = []
                while node.parent != None:
                    path.append(node)
                    node = node.parent
                path.reverse()
                for node in path:
                    pose = PoseStamped()
                    pose.pose.position.x, pose.pose.position.y = self.CRToXY_(
                        node.c,
                        node.r,
                    )
                    msg_path.poses.append(pose)

                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = dc + node.c
                nb_r = dr + node.r
                nb_idx = self.CRToIndex_(nb_c, nb_r)

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_idx in visited:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                # Get the relative g-cost and push to open-list
                nb_x, nb_y = self.CRToXY_(nb_c, nb_r)
                node_x, node_y = self.CRToXY_(node.c, node.r)
                new_g = node.g + hypot(nb_x - node_x, nb_y - node_y) * (
                    self.costmap_[nb_idx] + 1
                )

                if new_g < nb_node.g:
                    nb_node.g = new_g
                    nb_node.parent = node
                    heappush(open_list, nb_node)

        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
