# src/virtual_hardware.py
import numpy as np

class VirtualESP32:
    def __init__(self):
        self.left_pwm = 0
        self.right_pwm = 0
        self.ultrasonic_distance = 100.0  # cm (Starts with a clear path)
        self.servo_angle = 0

    def send_motor_cmd(self, left_pwm, right_pwm):
        self.left_pwm = left_pwm
        self.right_pwm = right_pwm

    def send_servo_cmd(self, angle):
        self.servo_angle = angle
        print(f"[VIRTUAL ESP32] Servo Gripper actuated to {angle} degrees.")


class VirtualCameraLocalizer:
    def __init__(self, start_pose):
        # Internal ground-truth state tracking: [X, Y, Theta]
        self.pose = list(start_pose)

    def update_physics(self, left_pwm, right_pwm, dt=0.05):
        """Simulates physical differential drive wheel responses."""
        v_l = left_pwm * 0.15
        v_r = right_pwm * 0.15

        # Differential kinematics
        linear_vel = (v_l + v_r) / 2.0
        
        # FIXED: To turn left (positive angle), the right wheel must move faster than the left.
        angular_vel = (v_r - v_l) / 10.0  

        # Update absolute positioning matrix
        self.pose[0] += linear_vel * np.cos(self.pose[2])
        self.pose[1] += linear_vel * np.sin(self.pose[2])
        self.pose[2] += angular_vel

        # Keep orientation bounded
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))

    def get_pose(self, frame=None):
        """Simulates camera detecting an ArUco marker and returning the current pose."""
        return self.pose[0], self.pose[1], self.pose[2]
