from math import cos, sin


class DWALocalPlanner:
    def __init__(
        self,
        costmap: list[int],
        origin_x: float,
        origin_y: float,
        resolution: float,
        columns: int,
        rows: int,
        max_access_cost: int,
        max_linear_velocity: float = 0.2,
        max_angular_velocity: float = 2,
        max_linear_acceleration: float = 0.05,
        max_angular_acceleration: float = 0.1,
        dt: float = 0.1,
        linear_velocity_resolution: float = 0.01,
        angular_velocity_resolution: float = 0.1,
        time_horizon: float = 2.0,
    ):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.dt = dt
        self.linear_velocity_resolution = linear_velocity_resolution
        self.angular_velocity_resolution = angular_velocity_resolution
        self.time_horizon = time_horizon

        # Robot state in world frame
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_linear_velocity = None
        self.current_angular_velocity = None

        # Costmap parameters
        self.costmap = costmap
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.columns = columns
        self.rows = rows
        self.max_access_cost = max_access_cost

    # Map utilities
    def map_to_world(self, c: int, r: int) -> tuple[float, float]:
        x = self.origin_x + (c + 0.5) * self.resolution
        y = self.origin_y + (r + 0.5) * self.resolution
        return (x, y)

    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        c = int((x - self.origin_x) / self.resolution - 0.5)
        r = int((y - self.origin_y) / self.resolution - 0.5)
        return (c, r)

    def world_to_index(self, x: float, y: float) -> int:
        c, r = self.world_to_map(x, y)
        if c < 0 or c >= self.columns or r < 0 or r >= self.rows:
            return -1  # Out of bounds
        return r * self.columns + c

    def is_collision_free_or_cost(
        self,
        trajectory: list[tuple[float, float]],
    ) -> float:
        cost = 0.0
        for x, y in trajectory:
            index = self.world_to_index(x, y)
            if index == -1:
                return -1  # Out of bounds
            if self.costmap[index] >= self.max_access_cost:
                return -1  # Collision detected

            cost += self.costmap[index]
        return cost

    def get_goal_cost(
        self,
        trajectory: list[tuple[float, float]],
        goal_x: float,
        goal_y: float,
    ) -> float:
        last_x, last_y = trajectory[-1]
        # Simple Euclidean distance to goal
        return ((last_x - goal_x) ** 2 + (last_y - goal_y) ** 2) ** 0.5

    def get_speed_cost(
        self,
        linear_velocity: float,
        angular_velocity: float,
    ) -> float:
        # Prefer higher speeds

        # No need absolute because linear_velocity is always positive
        speed_cost = (self.max_linear_velocity - linear_velocity) ** 2 + (
            self.max_angular_velocity - abs(angular_velocity)
        ) ** 2
        return speed_cost

    def update_robot_state(
        self,
        x: float,
        y: float,
        yaw: float,
        linear_velocity: float,
        angular_velocity: float,
    ):
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.current_linear_velocity = linear_velocity
        self.current_angular_velocity = angular_velocity

    def generate_velocity_window(
        self,
        current_linear_velocity: float,
        current_angular_velocity: float,
    ) -> tuple[list[float], list[float]]:
        min_linear_velocity = max(
            0.0,
            current_linear_velocity - self.max_linear_acceleration * self.dt,
        )
        max_linear_velocity = min(
            self.max_linear_velocity,
            current_linear_velocity + self.max_linear_acceleration * self.dt,
        )

        min_angular_velocity = max(
            -self.max_angular_velocity,
            current_angular_velocity - self.max_angular_acceleration * self.dt,
        )
        max_angular_velocity = min(
            self.max_angular_velocity,
            current_angular_velocity + self.max_angular_acceleration * self.dt,
        )

        possible_linear_velocities = []
        v = min_linear_velocity
        while v <= max_linear_velocity:
            possible_linear_velocities.append(v)
            v += self.linear_velocity_resolution

        possible_angular_velocities = []
        w = min_angular_velocity
        while w <= max_angular_velocity:
            possible_angular_velocities.append(w)
            w += self.angular_velocity_resolution

        return (
            possible_linear_velocities,
            possible_angular_velocities,
        )

    def predict_motion(
        self,
        linear_velocity: float,
        angular_velocity: float,
        time_horizon: float,
    ):
        x = self.current_x
        y = self.current_y
        yaw = self.current_yaw

        num_steps = int(time_horizon / self.dt)

        # We actually only concern ourselves with positions
        trajectory = [(x, y)]
        for _ in range(num_steps):
            x += linear_velocity * cos(yaw) * self.dt
            y += linear_velocity * sin(yaw) * self.dt
            yaw += angular_velocity * self.dt
            trajectory.append((x, y))

        return trajectory

    def generate_best_velocity_command(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        current_linear_velocity: float,
        current_angular_velocity: float,
        goal_x: float,
        goal_y: float,
    ):

        self.update_robot_state(
            current_x,
            current_y,
            current_yaw,
            current_linear_velocity,
            current_angular_velocity,
        )

        possible_linear_velocities, possible_angular_velocities = (
            self.generate_velocity_window(
                current_linear_velocity,
                current_angular_velocity,
            )
        )

        best_velocity_command = (0.0, 0.0)
        min_cost = float("inf")

        visualize_trajectories = []

        for v in possible_linear_velocities:
            for w in possible_angular_velocities:
                trajectory = self.predict_motion(v, w, self.time_horizon)

                collision_cost = self.is_collision_free_or_cost(trajectory)
                if collision_cost == -1:
                    continue  # Skip trajectories that result in collision

                goal_cost = self.get_goal_cost(trajectory, goal_x, goal_y)
                speed_cost = self.get_speed_cost(v, w)

                total_cost = 1.0 * collision_cost + 1.0 * goal_cost + 0.1 * speed_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_velocity_command = (v, w)

                visualize_trajectories.append(trajectory)

        return (best_velocity_command, visualize_trajectories)
