import numpy as np

class WaypointNavigator:
    def __init__(
        self,
        K_linear=1.5,
        K_angular=4.0,
        target_dist_tolerance=3.0,
        min_linear_vel=30.0,
        max_linear_vel=140.0,
        max_angular_vel=120.0,
    ):
        self.Kp_linear = K_linear
        self.Kp_angular = K_angular
        self.tolerance = target_dist_tolerance
        self.min_linear_vel = min_linear_vel
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel

    def get_steering_commands(self, current_pose, waypoint):
        x, y, theta = current_pose
        x_g, y_g = waypoint

        dx = x_g - x
        dy = y_g - y
        distance = np.hypot(dx, dy)

        if distance < self.tolerance:
            return 0, 0, True

        desired_heading = np.arctan2(dy, dx)
        heading_error = desired_heading - theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        if abs(heading_error) > 0.2:
            linear_vel = 0
            angular_vel = self.Kp_angular * heading_error
        else:
            linear_vel = self.Kp_linear * distance
            if 0 < linear_vel < self.min_linear_vel:
                linear_vel = self.min_linear_vel
            angular_vel = self.Kp_angular * heading_error

        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

        return linear_vel, angular_vel, False

    def unicycle_to_differential(self, v, omega, max_speed=255):
        """Convert forward/turn velocity to signed left/right motor commands."""
        left_pwm = v - omega
        right_pwm = v + omega
        return (
            int(round(np.clip(left_pwm, -max_speed, max_speed))),
            int(round(np.clip(right_pwm, -max_speed, max_speed))),
        )
