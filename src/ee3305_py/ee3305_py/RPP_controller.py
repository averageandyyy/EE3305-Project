from math import cos, hypot, sin, atan2


class RPPController:
    """
    A pure pursuit controller that computes linear and angular velocity commands
    """
    def __init__(
        self,
        min_lookahead: float,
        max_lookahead: float,
        lookahead_gain: float,
        base_linear_velocity: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
        stop_threshold: float,
        curvature_slowdown_gain: float,
        goal_slowdown_distance: float,
        rotate_threshold: float,
        rotate_tolerance: float,
        rotate_speed: float,
        rotate_gain: float,
    ):
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        self.base_linear_velocity = base_linear_velocity
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.curvature_slowdown_gain = curvature_slowdown_gain
        self.goal_slowdown_distance = goal_slowdown_distance
        self.stop_threshold = stop_threshold
        self.rotate_threshold = rotate_threshold
        self.rotate_tolerance = rotate_tolerance
        self.rotate_speed = rotate_speed
        self.rotate_gain = rotate_gain

    def get_velocity_command(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        lookahead_x: float,
        lookahead_y: float,
    ):        
        if lookahead_x is None:
            return

        # Transform target to robot frame
        dx = lookahead_x - current_x
        dy = lookahead_y - current_y
        local_x = dx * cos(current_yaw) + dy * sin(current_yaw)
        local_y = -dx * sin(current_yaw) + dy * cos(current_yaw)

        dist = hypot(local_x, local_y)

        PP_heading_error = atan2(local_y, local_x)
        curvature = 0.0

        if abs(PP_heading_error) < 0.785 and dist > self.stop_threshold:  # 45 degrees and never reach goal
            # curvature (avoid division by zero)
            denom = max(1e-6, local_x**2 + local_y**2)
            curvature = (2.0 * local_y) / denom

            # === Velocity Regulation ===
            # base speed reduced by curvature and goal proximity
            curve_factor = 1.0 / (1.0 + self.curvature_slowdown_gain * abs(curvature))
            dist_to_goal = hypot(
                lookahead_x - current_x,
                lookahead_y - current_y,
            )
            goal_factor = min(1.0, dist_to_goal / self.goal_slowdown_distance)

            lin_vel = self.base_linear_velocity * curve_factor * goal_factor
            lin_vel = min(lin_vel, self.max_linear_velocity)
            # angular velocity from curvature
            ang_vel = curvature * lin_vel
            ang_vel = max(-self.max_angular_velocity, min(self.max_angular_velocity, ang_vel))
        else:
            # Too far off heading → rotate on the spot
            lin_vel = 0.0

            # Angular velocity proportional to heading_error, limited by max_ang_vel_
            ang_vel = self.rotate_gain * PP_heading_error
            ang_vel = max(-self.max_angular_velocity, min(self.max_angular_velocity, ang_vel))
            # Stop rotating if within tolerance
            if abs(PP_heading_error) < self.rotate_tolerance:
                ang_vel = 0.0

        return lin_vel, ang_vel