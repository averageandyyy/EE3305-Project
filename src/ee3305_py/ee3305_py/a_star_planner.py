from heapq import heappop, heappush


class AStarPlanner:
    """
    Reference: https://en.wikipedia.org/wiki/A*_search_algorithm
    """

    # Directions for 4-connectivity (up, right, down, left)
    DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    # Adding diagonal directions for A* to find smoother paths
    DIRECTIONS_8 = DIRECTIONS + [(1, 1), (1, -1), (-1, 1), (-1, -1)]

    def __init__(
        self,
        costmap: list[int],
        origin_x: float,
        origin_y: float,
        resolution: float,
        columns: int,
        rows: int,
        max_access_cost: int,
    ):

        self.costmap_ = costmap
        self.origin_x_ = origin_x
        self.origin_y_ = origin_y
        self.resolution_ = resolution
        self.columns_ = columns
        self.rows_ = rows
        self.max_access_cost_ = max_access_cost

        self.start_x = None
        self.start_y = None
        self.goal_x = None
        self.goal_y = None

        self.nodes = []

        self.x_lower_bound = self.origin_x_
        self.x_upper_bound = self.origin_x_ + self.columns_ * self.resolution_
        self.y_lower_bound = self.origin_y_
        self.y_upper_bound = self.origin_y_ + self.rows_ * self.resolution_

    def map_to_world(self, c: int, r: int) -> tuple[float, float]:
        x = self.origin_x_ + (c + 0.5) * self.resolution_
        y = self.origin_y_ + (r + 0.5) * self.resolution_
        return (x, y)

    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        c = int((x - self.origin_x_) / self.resolution_ - 0.5)
        r = int((y - self.origin_y_) / self.resolution_ - 0.5)
        return (c, r)

    def map_to_index(self, c: int, r: int) -> int:
        return r * self.columns_ + c

    def is_in_bounds(self, x: float, y: float) -> bool:
        return (
            self.x_lower_bound <= x <= self.x_upper_bound
            and self.y_lower_bound <= y <= self.y_upper_bound
        )

    def world_distance(self, node1: tuple[int, int], node2: tuple[int, int]) -> float:
        x1, y1 = self.map_to_world(node1[0], node1[1])
        x2, y2 = self.map_to_world(node2[0], node2[1])
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def make_plan(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> list[tuple[int, int]]:

        def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
            # Manhattan distance in map coordinates
            # return abs(a[0] - b[0]) + abs(a[1] - b[1])
            # Euclidean distance in world coordinates
            return self.world_distance(a, b)

        path_found = False
        self.start_x_map, self.start_y_map = self.world_to_map(start_x, start_y)
        self.goal_x_map, self.goal_y_map = self.world_to_map(goal_x, goal_y)
        open_set = []
        heappush(
            open_set,
            (0, (self.start_x_map, self.start_y_map)),  # (f_score, (c, r))
        )

        came_from = {}

        g_score = {
            (c, r): float("inf")
            for r in range(self.rows_)
            for c in range(self.columns_)
        }
        g_score[(self.start_x_map, self.start_y_map)] = 0.0

        f_score = {
            (c, r): float("inf")
            for r in range(self.rows_)
            for c in range(self.columns_)
        }
        f_score[(self.start_x_map, self.start_y_map)] = heuristic(
            (self.start_x_map, self.start_y_map),
            (self.goal_x_map, self.goal_y_map),
        )

        open_set_hash = {(self.start_x_map, self.start_y_map)}

        while open_set:
            _, current = heappop(open_set)
            open_set_hash.remove(current)

            if current == (self.goal_x_map, self.goal_y_map):
                path_found = True
                break

            for dc, dr in self.DIRECTIONS_8:
                neighbor = (current[0] + dc, current[1] + dr)

                if not (
                    0 <= neighbor[0] < self.columns_ and 0 <= neighbor[1] < self.rows_
                ):
                    continue

                neighbor_index = self.map_to_index(neighbor[0], neighbor[1])
                if self.costmap_[neighbor_index] >= self.max_access_cost_:
                    continue

                tentative_g_score = g_score[current] + self.world_distance(
                    current,
                    neighbor,
                ) * (self.costmap_[neighbor_index] + 1)

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(
                        neighbor,
                        (self.goal_x_map, self.goal_y_map),
                    )
                    if neighbor not in open_set_hash:
                        heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)

        if path_found:
            path = []
            current = (self.goal_x_map, self.goal_y_map)
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append((self.start_x_map, self.start_y_map))
            path.reverse()

            # Convert path to world coordinates
            path = [self.map_to_world(c, r) for c, r in path]
            return path

        return []
