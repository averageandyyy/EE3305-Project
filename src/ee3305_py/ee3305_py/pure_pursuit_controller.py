from math import cos, hypot, sin


class PurePursuitController:
    """
    A pure pursuit controller that computes linear and angular velocity commands
    """

    def __init__(
        self,
        stop_threshold: float,
        lookahead_linear_velocity: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
    ):
        self.stop_threshold = stop_threshold
        self.lookahead_linear_velocity = lookahead_linear_velocity
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

    def get_velocity_command(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        lookahead_x: float,
        lookahead_y: float,
    ):
        # get distance to lookahead point (not to be confused with lookahead_distance)
        distance_to_lookahead = hypot(lookahead_x - current_x, lookahead_y - current_y)
        # stop the robot if close to the point.
        if distance_to_lookahead < self.stop_threshold:
            # saturate velocities.
            # but only when the robot is travelling too fast (which should not occur if well tuned).
            lin_vel = 0.0
            ang_vel = 0.0
        else:
            # get curvature, do transformation from robot frame to local frame
            dx = lookahead_x - current_x
            dy = lookahead_y - current_y

            local_x = (dx * cos(current_yaw)) + (dy * sin(current_yaw))
            local_y = (dy * cos(current_yaw)) - (dx * sin(current_yaw))

            # formula from slides
            curvature = (2 * local_y) / (local_x**2 + local_y**2)

            # calculate velocities
            lin_vel = self.lookahead_linear_velocity
            # idt this will ever get triggered
            if lin_vel > self.max_linear_velocity:
                lin_vel = self.max_linear_velocity

            ang_vel = lin_vel * curvature
            if ang_vel > self.max_angular_velocity:
                ang_vel = self.max_angular_velocity
            elif ang_vel < -self.max_angular_velocity:
                ang_vel = -self.max_angular_velocity

        return lin_vel, ang_vel
