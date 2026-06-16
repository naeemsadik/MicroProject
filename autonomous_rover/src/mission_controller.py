import argparse
import os
import time

import numpy as np
import yaml

from comms import ESP32Interface
from navigator import WaypointNavigator
from odometry import DifferentialOdometry
from planner import AStarPlanner
from warehouse_slots import WarehouseSlots


class MissionController:
    def __init__(self, settings_path=None, qr_override=None, dry_run=False, regenerate_map=False):
        self.project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.settings_path = settings_path or self._project_path("config/settings.yaml")
        self.settings = self._load_settings(self.settings_path)
        self.qr_override = qr_override
        self.dry_run = dry_run
        self.regenerate_map = regenerate_map
        self.odometry_enabled = bool(self.settings.get("odometry", {}).get("enabled", True))

        serial_cfg = self.settings["serial"]
        self.esp32 = ESP32Interface(
            port=serial_cfg.get("port", "/dev/ttyUSB0"),
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
            max_linear_vel=float(nav_cfg.get("max_linear_vel", 140.0)),
            max_angular_vel=float(nav_cfg.get("max_angular_vel", 120.0)),
        )

        map_cfg = self.settings["map"]
        odo_cfg = self.settings["odometry"]
        self.odometry = DifferentialOdometry(
            resolution_cm_per_px=float(map_cfg.get("resolution_cm_per_px", 1.0)),
            wheel_diameter_cm=float(odo_cfg.get("wheel_diameter_cm", 6.5)),
            wheel_base_cm=float(odo_cfg.get("wheel_base_cm", 15.0)),
            ticks_per_revolution=float(odo_cfg.get("ticks_per_revolution", 20)),
            yaw_sign=float(odo_cfg.get("yaw_sign", 1.0)),
        )

        slots_path = self._project_path(self.settings["slots"].get("file", "config/warehouse_slots.yaml"))
        self.slots = WarehouseSlots.load(slots_path)
        home = tuple(nav_cfg.get("home", self.slots.home))
        self.odometry.reset((home[0], home[1], 0.0))

    def run_once(self):
        try:
            if not self.dry_run:
                self.esp32.send_gripper_cmd("OPEN")

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
                raise RuntimeError(
                    "Real navigation is disabled because odometry.enabled is false. "
                    "Add wheel encoders or another localization source before autonomous driving."
                )

            self.esp32.send_gripper_cmd("CLOSE")
            time.sleep(1.0)

            self._follow_waypoints(waypoints)
            self.esp32.stop()
            time.sleep(0.5)
            self.esp32.send_gripper_cmd("OPEN")
            print("[MISSION] Delivery complete.")
        finally:
            self.esp32.stop()

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

        while waypoints and time.monotonic() < deadline:
            telemetry = self.esp32.get_telemetry()
            pose = self.odometry.update(
                telemetry.left_ticks,
                telemetry.right_ticks,
                telemetry.yaw_deg,
            )

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
                continue

            left_speed, right_speed = self.navigator.unicycle_to_differential(linear_v, angular_w)
            self.esp32.send_velocity_cmd(left_speed, right_speed)
            time.sleep(loop_delay)

        self.esp32.stop()
        if waypoints:
            raise TimeoutError(f"Mission timed out with remaining waypoints: {waypoints}")

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
