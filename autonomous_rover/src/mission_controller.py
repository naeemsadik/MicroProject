import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from comms import ESP32Interface
    from navigator import WaypointNavigator
    from odometry import DifferentialOdometry
    from planner import AStarPlanner
    from warehouse_slots import WarehouseSlots
else:
    from .comms import ESP32Interface
    from .navigator import WaypointNavigator
    from .odometry import DifferentialOdometry
    from .planner import AStarPlanner
    from .warehouse_slots import WarehouseSlots


class MissionController:
    """
    Drives the high-level mission:

        1. Read (or scan) the package QR code.
        2. Look up the destination slot in warehouse_slots.yaml.
        3. Plan a path with A*.
        4. Send velocity commands to the ESP32 waypoint by waypoint.
        5. Close the gripper to pick the package, open it to deliver.

    Two localization modes are supported:

        * odometry  - based on wheel encoders and the MPU6050 yaw.
        * dead-reckoning - based on timed motion (used when no encoders
          are wired, which is the current hardware setup).

    The mode is selected by ``odometry.enabled`` in ``config/settings.yaml``.
    """

    def __init__(self, settings_path=None, qr_override=None, dry_run=False, regenerate_map=False):
        self.project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.settings_path = settings_path or self._project_path("config/settings.yaml")
        self.settings = self._load_settings(self.settings_path)
        self.qr_override = qr_override
        self.dry_run = dry_run
        self.regenerate_map = regenerate_map
        self.odometry_enabled = bool(self.settings.get("odometry", {}).get("enabled", False))

        serial_cfg = self.settings["serial"]
        self.esp32 = ESP32Interface(
            port=serial_cfg.get("port", "/dev/ttyACM0"),
            baudrate=int(serial_cfg.get("baudrate", 115200)),
        )
        if self.esp32.simulation_mode:
            print("[MISSION] ESP32 is not connected; switching to dry-run navigation.")
            self.dry_run = True

        self.grid = self._load_or_generate_map()
        self.planner = AStarPlanner(self.grid)

        nav_cfg = self.settings["navigation"]
        self.navigator = WaypointNavigator(
            K_linear=float(nav_cfg.get("K_linear", 1.2)),
            K_angular=float(nav_cfg.get("K_angular", 3.5)),
            target_dist_tolerance=float(nav_cfg.get("target_tolerance_px", 5.0)),
            min_linear_vel=float(nav_cfg.get("min_linear_vel", 35.0)),
            max_linear_vel=float(nav_cfg.get("max_linear_vel", 90.0)),
            max_angular_vel=float(nav_cfg.get("max_angular_vel", 90.0)),
        )
        self.max_drive_speed = float(nav_cfg.get("max_drive_speed", 90))
        self.max_turn_speed = float(nav_cfg.get("max_turn_speed", 90))

        map_cfg = self.settings["map"]
        odo_cfg = self.settings["odometry"]
        self.odometry = DifferentialOdometry(
            resolution_cm_per_px=float(map_cfg.get("resolution_cm_per_px", 1.0)),
            wheel_diameter_cm=float(odo_cfg.get("wheel_diameter_cm", 6.5)),
            wheel_base_cm=float(odo_cfg.get("wheel_base_cm", 15.0)),
            ticks_per_revolution=float(odo_cfg.get("ticks_per_revolution", 20)),
            yaw_sign=float(odo_cfg.get("yaw_sign", 1.0)),
        )

        dr_cfg = self.settings.get("dead_reckoning", {})
        self.forward_speed_cm_per_s = float(dr_cfg.get("forward_speed_cm_per_s", 20.0))
        self.backward_speed_cm_per_s = float(dr_cfg.get("backward_speed_cm_per_s", 18.0))
        self.turn_speed_deg_per_s = float(dr_cfg.get("turn_speed_deg_per_s", 60.0))

        slots_path = self._project_path(self.settings["slots"].get("file", "config/warehouse_slots.yaml"))
        self.slots = WarehouseSlots.load(slots_path)
        home = tuple(nav_cfg.get("home", self.slots.home))
        self.odometry.reset((home[0], home[1], 0.0))
        self.home_pose = (float(home[0]), float(home[1]), 0.0)

    # ------------------------------------------------------------------ run
    def run_once(self):
        try:
            if not self.dry_run:
                # Open gripper so we can pick up a fresh package.
                self.esp32.send_gripper_cmd("OPEN")
                time.sleep(0.5)

            slot_id = self._get_slot_id()
            destination = self.slots.get_destination(slot_id)
            target = destination.navigation_target

            print(f"[MISSION] QR destination {slot_id}: navigating to {target}")
            waypoints = self._plan_waypoints(target)
            print(f"[MISSION] Planned {len(waypoints)} waypoint(s): {waypoints}")

            if self.dry_run:
                print("[MISSION] Dry run complete; not sending movement commands.")
                return

            if not self.odometry_enabled:
                print("[MISSION] Using dead-reckoning localization (no encoders).")
            else:
                print("[MISSION] Using encoder odometry + MPU6050 yaw.")

            # Pick the package at the starting location.
            self.esp32.send_gripper_cmd("CLOSE")
            time.sleep(0.8)

            self._follow_waypoints(waypoints)
            self.esp32.stop()
            time.sleep(0.5)

            # Deliver the package.
            self.esp32.send_gripper_cmd("OPEN")
            time.sleep(0.8)
            print("[MISSION] Delivery complete.")
        except Exception as exc:
            print(f"[MISSION] Aborted: {exc}")
            raise
        finally:
            self.esp32.stop()

    # ---------------------------------------------------------------- helpers
    def _get_slot_id(self):
        if self.qr_override:
            return self.qr_override

        camera_cfg = self.settings["camera"]
        from qr_scanner import QRScanner

        scanner = QRScanner(camera_index=int(camera_cfg.get("index", 0)))
        print("[MISSION] Waiting for package QR code...")
        return scanner.scan(timeout_s=float(camera_cfg.get("qr_timeout_s", 30)))

    def _plan_waypoints(self, target):
        pose = self.odometry.pose
        start = (int(round(pose[0])), int(round(pose[1])))
        dense_path = self.planner.plan_path(start, target)
        if not dense_path:
            raise RuntimeError(f"No path from {start} to {target}")

        waypoints = self.planner.prune_path(dense_path)
        if waypoints and waypoints[0] == start:
            waypoints.pop(0)
        return waypoints

    def _follow_waypoints(self, waypoints):
        nav_cfg = self.settings["navigation"]
        obstacle_stop_cm = float(nav_cfg.get("obstacle_stop_cm", 18.0))
        obstacle_resume_cm = float(nav_cfg.get("obstacle_resume_cm", 25.0))
        loop_delay = 1.0 / float(nav_cfg.get("loop_hz", 10))
        deadline = time.monotonic() + float(nav_cfg.get("max_mission_seconds", 180))

        # Reset encoder / yaw counters at the start of the mission.
        try:
            self.esp32.send_command("<RESET_TICKS>")
        except Exception:
            pass

        while waypoints and time.monotonic() < deadline:
            telemetry = self.esp32.get_telemetry()
            if self.odometry_enabled:
                pose = self.odometry.update(
                    telemetry.left_ticks,
                    telemetry.right_ticks,
                    telemetry.yaw_deg,
                )
            else:
                pose = self.odometry.pose

            if telemetry.distance_cm > 0 and telemetry.distance_cm < obstacle_stop_cm:
                print(f"[SAFETY] Obstacle at {telemetry.distance_cm:.1f} cm; stopping.")
                self.esp32.stop()
                while telemetry.distance_cm < obstacle_resume_cm and time.monotonic() < deadline:
                    time.sleep(0.2)
                    telemetry = self.esp32.get_telemetry()
                continue

            target = waypoints[0]
            linear_v, angular_w, arrived = self.navigator.get_steering_commands(pose, target)
            if arrived:
                print(f"[NAV] Reached waypoint {target}")
                waypoints.pop(0)
                self.esp32.stop()
                time.sleep(0.2)
                continue

            left_speed, right_speed = self.navigator.unicycle_to_differential(linear_v, angular_w)
            left_speed, right_speed = self._clip_speeds(left_speed, right_speed)

            if self.odometry_enabled:
                self.esp32.send_velocity_cmd(left_speed, right_speed)
            else:
                # Dead reckoning: do not loop continuously, send a single
                # timed pulse per cycle and update pose in software.
                self._dead_reckoning_step(pose, target)

            time.sleep(loop_delay)

        self.esp32.stop()
        if waypoints:
            raise TimeoutError(f"Mission timed out with remaining waypoints: {waypoints}")

    def _clip_speeds(self, left_speed, right_speed):
        return (
            int(max(-self.max_drive_speed, min(self.max_drive_speed, left_speed))),
            int(max(-self.max_drive_speed, min(self.max_drive_speed, right_speed))),
        )

    def _dead_reckoning_step(self, pose, target):
        """Drive one timed step toward ``target`` using dead reckoning."""
        x, y, theta = pose
        tx, ty = target
        dx = tx - x
        dy = ty - y
        distance_px = math.hypot(dx, dy)
        if distance_px < 1:
            return

        # Convert pixel distance to cm using the map scale.
        distance_cm = distance_px * float(self.settings["map"].get("resolution_cm_per_px", 1.0))
        forward_time_s = distance_cm / max(self.forward_speed_cm_per_s, 1e-3)
        # Cap each timed step to keep recovery from errors small.
        step_time_s = max(0.1, min(forward_time_s, 0.5))

        # First, rotate to face the target.
        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(desired_heading - theta), math.cos(desired_heading - theta))
        if abs(heading_error) > math.radians(10):
            turn_time_s = abs(math.degrees(heading_error)) / max(self.turn_speed_deg_per_s, 1e-3)
            turn_time_s = max(0.1, min(turn_time_s, 1.0))
            if heading_error > 0:
                # Turn left: left wheel backward, right wheel forward
                self.esp32.send_velocity_cmd(-self.max_turn_speed, self.max_turn_speed)
            else:
                # Turn right: left wheel forward, right wheel backward
                self.esp32.send_velocity_cmd(self.max_turn_speed, -self.max_turn_speed)
            time.sleep(turn_time_s)
            self.esp32.stop()
            time.sleep(0.1)
            # Update internal heading from the commanded turn.
            self.odometry.pose[2] = _wrap_angle(self.odometry.pose[2] + heading_error)
            return

        # Drive forward for a small timed slice.
        self.esp32.send_velocity_cmd(self.max_drive_speed, self.max_drive_speed)
        time.sleep(step_time_s)
        self.esp32.stop()

        # Update the in-software pose with the distance we just commanded.
        new_x = self.odometry.pose[0] + (self.max_drive_speed / 255.0) * self.forward_speed_cm_per_s * step_time_s / float(self.settings["map"].get("resolution_cm_per_px", 1.0)) * math.cos(self.odometry.pose[2])
        new_y = self.odometry.pose[1] + (self.max_drive_speed / 255.0) * self.forward_speed_cm_per_s * step_time_s / float(self.settings["map"].get("resolution_cm_per_px", 1.0)) * math.sin(self.odometry.pose[2])
        self.odometry.pose[0] = new_x
        self.odometry.pose[1] = new_y

    def _load_or_generate_map(self):
        map_cfg = self.settings["map"]
        image_path = self._project_path(map_cfg.get("image", "maps/floorplan.png"))
        grid_path = self._project_path(map_cfg.get("grid", "maps/occupancy_grid.npy"))

        if self.regenerate_map or not os.path.exists(grid_path):
            from map_processor import MapProcessor

            processor = MapProcessor(
                resolution_cm_per_px=float(map_cfg.get("resolution_cm_per_px", 1.0)),
                robot_radius_cm=float(map_cfg.get("robot_radius_cm", 18.0)),
            )
            return processor.generate_occupancy_grid(image_path, grid_path)

        return np.load(grid_path)

    def _project_path(self, relative_path):
        if os.path.isabs(relative_path):
            return relative_path
        return os.path.join(self.project_dir, relative_path)

    @staticmethod
    def _load_settings(path):
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser(description="Run one QR-driven warehouse rover mission.")
    parser.add_argument("--settings", default=None, help="Path to settings.yaml")
    parser.add_argument("--qr", default=None, help="Use a QR payload such as R1C1 instead of the camera")
    parser.add_argument("--dry-run", action="store_true", help="Plan and print the route without driving")
    parser.add_argument("--regenerate-map", action="store_true", help="Regenerate occupancy grid from the map image")
    args = parser.parse_args()

    controller = MissionController(
        settings_path=args.settings,
        qr_override=args.qr,
        dry_run=args.dry_run,
        regenerate_map=args.regenerate_map,
    )
    controller.run_once()


if __name__ == "__main__":
    main()
