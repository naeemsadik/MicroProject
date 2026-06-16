import math


class DifferentialOdometry:
    def __init__(
        self,
        resolution_cm_per_px=1.0,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15.0,
        ticks_per_revolution=20,
        yaw_sign=1.0,
    ):
        self.resolution_cm_per_px = float(resolution_cm_per_px)
        self.wheel_diameter_cm = float(wheel_diameter_cm)
        self.wheel_base_cm = float(wheel_base_cm)
        self.ticks_per_revolution = float(ticks_per_revolution)
        self.yaw_sign = float(yaw_sign)
        self.cm_per_tick = math.pi * self.wheel_diameter_cm / self.ticks_per_revolution

        self.pose = [0.0, 0.0, 0.0]
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.yaw_zero_deg = None

    def reset(self, pose, left_ticks=0, right_ticks=0, yaw_deg=None):
        self.pose = [float(pose[0]), float(pose[1]), float(pose[2]) if len(pose) > 2 else 0.0]
        self.last_left_ticks = int(left_ticks)
        self.last_right_ticks = int(right_ticks)
        self.yaw_zero_deg = float(yaw_deg) if yaw_deg is not None else None
        return tuple(self.pose)

    def update(self, left_ticks, right_ticks, yaw_deg=None):
        left_ticks = int(left_ticks)
        right_ticks = int(right_ticks)

        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            if yaw_deg is not None and self.yaw_zero_deg is None:
                self.yaw_zero_deg = float(yaw_deg)
            return tuple(self.pose)

        delta_left_cm = (left_ticks - self.last_left_ticks) * self.cm_per_tick
        delta_right_cm = (right_ticks - self.last_right_ticks) * self.cm_per_tick
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        distance_cm = (delta_left_cm + delta_right_cm) / 2.0

        if yaw_deg is not None:
            if self.yaw_zero_deg is None:
                self.yaw_zero_deg = float(yaw_deg)
            yaw_delta_deg = (float(yaw_deg) - self.yaw_zero_deg) * self.yaw_sign
            self.pose[2] = math.radians(yaw_delta_deg)
        else:
            delta_theta = (delta_right_cm - delta_left_cm) / self.wheel_base_cm
            self.pose[2] = _wrap_angle(self.pose[2] + delta_theta)

        distance_px = distance_cm / self.resolution_cm_per_px
        self.pose[0] += distance_px * math.cos(self.pose[2])
        self.pose[1] += distance_px * math.sin(self.pose[2])
        self.pose[2] = _wrap_angle(self.pose[2])
        return tuple(self.pose)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
