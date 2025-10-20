import random
from math import atan2, cos, sin

from scipy.spatial import KDTree

from ee3305_py.bresenham import get_bresenham_line
from ee3305_py.node import Node


class FastRRTStarPlanner:
    """
    Python implementation of Fast-RRT* algorithm
    Heavily referenced from: https://github.com/mitre/Fast-RRT-Star/tree/main
    The primary selling point of F-RRT* is the dichotomy bisection to create nodes closer to obstacles. This is due to the general observation
    that optimal paths tend to be close to obstacles. Beyond that, F-RRT* is very similar to RRT* with the addition of the "reachest" node concept.

    In general, sampling-based planners like RRT and RRT* are not very efficient in 2D environments with obstacles. A* or Dijkstra's algorithm on a grid
    is usually preferred. Besides their sampling nature, the primary reason why sampling-based planners are made much more inefficient is the need to check
    for collisions.
    """

    def __init__(
        self,
        costmap: list[int],
        origin_x: float,
        origin_y: float,
        resolution: float,
        columns: int,
        rows: int,
        max_access_cost: int,
        max_iterations: int = 20000,
        min_iterations: int = 2000,
        expansion_radius: float = 5.0,
        search_radius: float = 2.0,
        dichotomy_distance: float = 0.05,
        tolerance_distance: float = 0.25,
        use_costmap=True,
    ):
        self.costmap_ = costmap
        self.costmap_origin_x_ = origin_x
        self.costmap_origin_y_ = origin_y
        self.costmap_resolution_ = resolution
        self.costmap_cols_ = columns
        self.costmap_rows_ = rows
        self.max_access_cost_ = max_access_cost
        self.max_iterations_ = max_iterations
        self.min_iterations_ = min_iterations
        self.expansion_radius_ = expansion_radius
        self.search_radius_ = search_radius
        self.dichotomy_distance_ = dichotomy_distance
        self.tolerance_distance_ = tolerance_distance
        self.use_costmap_ = use_costmap

        self.start_x = None
        self.start_y = None
        self.goal_x = None
        self.goal_y = None

        # data in KDTree corresponds to self.nodes
        self.nodes = []

        self.x_lower_bound = self.costmap_origin_x_
        self.x_upper_bound = (
            self.costmap_origin_x_ + self.costmap_cols_ * self.costmap_resolution_
        )
        self.y_lower_bound = self.costmap_origin_y_
        self.y_upper_bound = (
            self.costmap_origin_y_ + self.costmap_rows_ * self.costmap_resolution_
        )

    def sample_free_node(self):
        return Node(
            random.uniform(self.x_lower_bound, self.x_upper_bound),
            random.uniform(self.y_lower_bound, self.y_upper_bound),
        )

    def get_nearest_node(self, node: Node) -> int:
        # Linear search for the nearest node
        nearest_index = 0
        nearest_distance = float("inf")
        for i, existing_node in enumerate(self.nodes):
            dist = node.get_connection_cost(existing_node)
            if dist < nearest_distance:
                nearest_distance = dist
                nearest_index = i
        return nearest_index

    def get_nearby_nodes(self, node: Node) -> list[int]:
        indices = []
        for i, existing_node in enumerate(self.nodes):
            dist = node.get_connection_cost(existing_node)
            if dist <= self.search_radius_:
                indices.append(i)
        return indices

    def find_reachest(self, nearest_node: Node, candidate_node: Node) -> Node:
        # Get the most ancestral node that is collision-free to the candidate_node
        reachest_node = nearest_node
        while reachest_node != self.nodes[0]:  # While not the start node
            if self.is_collision_free(reachest_node.parent, candidate_node):
                reachest_node = reachest_node.parent
            else:
                return reachest_node
        return reachest_node

    def is_collision_free(self, node1: Node, node2: Node) -> bool:
        c1, r1 = self.world_to_map(node1.position_x, node1.position_y)
        c2, r2 = self.world_to_map(node2.position_x, node2.position_y)
        points = get_bresenham_line(c1, r1, c2, r2)
        for col, row in points:
            if (
                col < 0
                or col >= self.costmap_cols_
                or row < 0
                or row >= self.costmap_rows_
            ):
                return False
            if self.costmap_[row * self.costmap_cols_ + col] > self.max_access_cost_:
                return False
        return True

    def map_to_world(self, c: int, r: int) -> tuple[float, float]:
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_
        return (x, y)

    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        c = int((x - self.costmap_origin_x_) / self.costmap_resolution_ - 0.5)
        r = int((y - self.costmap_origin_y_) / self.costmap_resolution_ - 0.5)
        return (c, r)

    def create_node(self, reachest_node: Node, candidate_node: Node) -> Node:
        """
        Create a new node between reachest_node and candidate_node using dichotomy bisection
        Return None if no valid node can be created
        Arguments:
            reachest_node: Node - the most ancestral node that is collision-free to candidate_node
            candidate_node: Node - the randomly sampled node
        Returns:
            Node or None - the new node created between reachest_node and candidate_node, or None if no valid node can be created
        """
        # This is where the dichotomy bisection within F-RRT* takes place

        allow_node = reachest_node.copy()
        start_node = self.nodes[0]
        forbid_node = None

        if start_node != reachest_node:
            # forbid_node is a copy of reachest_node parent. has same position_x, position_y, cost, parent reference
            forbid_node = reachest_node.parent.copy()

            while self.distance(allow_node, forbid_node) > self.dichotomy_distance_:
                mid_x = (allow_node.position_x + forbid_node.position_x) / 2
                mid_y = (allow_node.position_y + forbid_node.position_y) / 2
                mid_node = Node(mid_x, mid_y)

                if self.is_collision_free(mid_node, candidate_node):
                    # Only update position_x and position_y to avoid changing the reference of allow_node
                    # This brings allow_node closer to an obstacle, starting from reachest_node
                    allow_node.position_x = mid_node.position_x
                    allow_node.position_y = mid_node.position_y
                else:
                    # This brings forbid_node closer to reachest_node, starting from reachest_node.parent
                    forbid_node.position_x = mid_node.position_x
                    forbid_node.position_y = mid_node.position_y

            forbid_node = candidate_node.copy()

            while self.distance(allow_node, forbid_node) > self.dichotomy_distance_:
                mid_x = (allow_node.position_x + forbid_node.position_x) / 2
                mid_y = (allow_node.position_y + forbid_node.position_y) / 2
                mid_node = Node(mid_x, mid_y)

                if self.is_collision_free(mid_node, reachest_node.parent):
                    # This brings allow_node closer to candidate_node
                    allow_node.position_x = mid_node.position_x
                    allow_node.position_y = mid_node.position_y
                else:
                    # This brings forbid_node closer to allow_node, possibly reachest_node if allow_node was not updated
                    forbid_node.position_x = mid_node.position_x
                    forbid_node.position_y = mid_node.position_y

        create_node = Node()

        # Possible for allow_node to converge to reachest_node
        if (
            allow_node != reachest_node
        ):  # Was using "is" which is incorrect, checks for reference equality
            create_node.position_x = allow_node.position_x
            create_node.position_y = allow_node.position_y
        else:
            return None

        return create_node

    def distance(self, node1: Node, node2: Node) -> float:
        return (
            (node1.position_x - node2.position_x) ** 2
            + (node1.position_y - node2.position_y) ** 2
        ) ** 0.5

    def get_cost_multiplier(self, node: Node) -> float:
        if not self.use_costmap_:
            return 1.0

        c, r = self.world_to_map(node.position_x, node.position_y)
        index = r * self.costmap_cols_ + c
        cost = self.costmap_[index]
        return 1 + cost

    def steer(self, nearest_node: Node, candidate_node: Node) -> Node:
        # Steer candidate_node to be within expansion_radius_ of nearest_node
        # Modifies candidate_node in place
        if self.distance(nearest_node, candidate_node) <= self.expansion_radius_:
            cost_multiplier = self.get_cost_multiplier(candidate_node)
            candidate_node.update_parent_and_cost(nearest_node, cost_multiplier)
            return

        theta = atan2(
            candidate_node.position_y - nearest_node.position_y,
            candidate_node.position_x - nearest_node.position_x,
        )
        new_x = nearest_node.position_x + self.expansion_radius_ * cos(theta)
        new_y = nearest_node.position_y + self.expansion_radius_ * sin(theta)
        candidate_node.position_x = new_x
        candidate_node.position_y = new_y
        return

    def has_initial_plan(self, goal_node: Node) -> bool:
        within_goal_threshold = (
            self.distance(self.nodes[-1], goal_node) <= self.tolerance_distance_
        )
        last_point_collision_free = self.is_collision_free(self.nodes[-1], goal_node)

        # The original F-RRT* checks for the entire path to be collision-free but we skip for now

        return within_goal_threshold and last_point_collision_free

    def rewire(self, candidate_node: Node, nearby_nodes_indices: list[int]):
        for index in nearby_nodes_indices:
            nearby_node = self.nodes[index]
            nearby_node_cost_multiplier = self.get_cost_multiplier(nearby_node)
            rewire_cost = candidate_node.cost + candidate_node.get_connection_cost(
                nearby_node,
                nearby_node_cost_multiplier,
            )
            if rewire_cost < nearby_node.cost and self.is_collision_free(
                candidate_node,
                nearby_node,
            ):
                nearby_node.update_parent_and_cost(
                    candidate_node,
                    nearby_node_cost_multiplier,
                )

    def make_plan(
        self, start_x: float, start_y: float, goal_x: float, goal_y: float
    ) -> list[Node]:
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.nodes = []

        start_node = Node(start_x, start_y)
        self.nodes.append(start_node)

        goal_node = Node(goal_x, goal_y)
        self.path_found = False
        self.best_path_cost = float("inf")
        self.best_end_node = None

        # Early exit if start and goal are too close
        if self.distance(start_node, goal_node) <= self.tolerance_distance_:
            return []

        for i in range(self.max_iterations_):
            # Sample free node
            candidate_node = self.sample_free_node()

            # Get nearest node
            nearest_node_index = self.get_nearest_node(candidate_node)
            nearest_node = self.nodes[nearest_node_index]

            # Steer
            self.steer(nearest_node, candidate_node)

            if self.is_collision_free(nearest_node, candidate_node):
                nearby_nodes_indices = self.get_nearby_nodes(candidate_node)
                reachest_node = self.find_reachest(nearest_node, candidate_node)
                create_node = self.create_node(reachest_node, candidate_node)

                if create_node is not None:
                    create_node_cost_multiplier = self.get_cost_multiplier(create_node)
                    create_node.update_parent_and_cost(
                        reachest_node.parent,
                        create_node_cost_multiplier,
                    )

                    candidate_node_cost_multiplier = self.get_cost_multiplier(
                        candidate_node,
                    )
                    candidate_node.update_parent_and_cost(
                        create_node,
                        candidate_node_cost_multiplier,
                    )

                    self.nodes.append(create_node)
                    self.nodes.append(candidate_node)
                else:
                    candidate_node_cost_multiplier = self.get_cost_multiplier(
                        candidate_node
                    )
                    candidate_node.update_parent_and_cost(
                        reachest_node,
                        candidate_node_cost_multiplier,
                    )
                    self.nodes.append(candidate_node)

                if self.has_initial_plan(goal_node):
                    self.path_found = True
                    current_path_cost = self.nodes[-1].cost

                    if current_path_cost < self.best_path_cost:
                        self.best_path_cost = current_path_cost
                        self.best_end_node = self.nodes[-1]

                    if i >= self.min_iterations_:
                        break

                # Rewire
                self.rewire(
                    candidate_node, nearby_nodes_indices
                )  # We can use the "old" indices because candidate_node is added at the end of self.nodes

        if self.path_found:
            path = []
            node = self.best_end_node
            while node is not None:
                path.append(node)
                node = node.parent
            path.reverse()
            return path

        return []
