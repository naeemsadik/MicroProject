"""
RPi4 admin panel for the warehouse rover.

Run on the Raspberry Pi:

    cd autonomous_rover
    pip install -r requirements.txt
    python src/admin_panel.py            # default host/port from settings
    python src/admin_panel.py --host 0.0.0.0 --port 8080

Then open the panel from a phone or laptop on the same network:

    http://RASPBERRY_PI_IP:8080

Features:
    * Live USB camera feed (MJPEG) with QR detection overlay.
    * ESP32 telemetry (ultrasonic distance, yaw, encoder ticks).
    * Manual drive controls (forward / backward / left / right / stop).
    * Gripper open / close buttons.
    * Live map preview with current pose and the planned route to the
      QR-detected destination.
    * Recent log messages.
    * List of configured warehouse slot IDs.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import yaml

# Allow running this file directly (``python src/admin_panel.py``) by
# adding the package directory to sys.path before relative imports.
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from comms import ESP32Interface
    from planner import AStarPlanner
    from warehouse_slots import WarehouseSlots, normalize_slot_id
else:
    from .comms import ESP32Interface
    from .planner import AStarPlanner
    from .warehouse_slots import WarehouseSlots, normalize_slot_id

try:
    import numpy as np
except ImportError:
    np = None


# ============================================================ camera worker
class CameraWorker:
    def __init__(self, camera_index=0, width=640, height=480):
        self.camera_index = int(camera_index)
        self.width = int(width)
        self.height = int(height)
        self.detector = cv2.QRCodeDetector()
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_qr = None
        self.latest_qr_error = None
        self.frame_ok = False
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_snapshot(self):
        with self.lock:
            return {
                "latest_jpeg": self.latest_jpeg,
                "latest_qr": self.latest_qr,
                "latest_qr_error": self.latest_qr_error,
                "frame_ok": self.frame_ok,
            }

    def _run(self):
        camera = cv2.VideoCapture(self.camera_index)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not camera.isOpened():
            with self.lock:
                self.frame_ok = False
                self.latest_qr_error = f"Could not open camera index {self.camera_index}"
            return

        try:
            while self.running:
                ok, frame = camera.read()
                if not ok or frame is None:
                    with self.lock:
                        self.frame_ok = False
                    time.sleep(0.05)
                    continue

                qr_value = None
                qr_error = None
                payload, points, _ = self.detector.detectAndDecode(frame)
                if payload:
                    try:
                        qr_value = normalize_slot_id(payload)
                    except ValueError as exc:
                        qr_error = str(exc)

                if points is not None:
                    pts = points.astype(int).reshape(-1, 2)
                    for i in range(len(pts)):
                        cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % len(pts)]), (0, 255, 0), 2)
                    if payload:
                        cv2.putText(
                            frame,
                            payload,
                            tuple(pts[0]),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )

                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue

                with self.lock:
                    self.latest_jpeg = encoded.tobytes()
                    self.frame_ok = True
                    if qr_value:
                        self.latest_qr = qr_value
                    self.latest_qr_error = qr_error

                time.sleep(0.03)
        finally:
            camera.release()


# ============================================================== auto-drive
class PosePublisher(threading.Thread):
    """
    Background publisher that copies ``controller.odometry.pose`` into
    ``admin_state.pose`` at a fixed rate, so the admin panel's map
    shows the rover moving live during a mission.

    Stops cleanly when the stop_event is set or the thread is joined.
    """

    def __init__(self, controller, admin_state, stop_event, hz=5.0):
        super().__init__(daemon=True)
        self.controller = controller
        self.admin_state = admin_state
        self._stop = stop_event
        self.period = 1.0 / max(hz, 0.5)

    def run(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                pose = self.controller.odometry.pose
                self.admin_state.set_pose(pose[0], pose[1], pose[2], source="auto")
            except Exception as exc:
                self.admin_state.log(f"[POSE] publisher error: {exc}")
            next_t += self.period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                # Wait on the event so we exit quickly on stop.
                self._stop.wait(sleep_for)
            else:
                # Falling behind; resync to avoid runaway.
                next_t = time.monotonic()


# ============================================================== auto-drive
class AutoDriver(threading.Thread):
    """
    Background thread that runs an autonomous ``MissionController``
    mission to a given slot.

    Status values:
        starting  – thread spawned, not yet planning
        planning  – A* + waypoint pruning in progress
        driving   – ESP32 is moving toward the waypoints
        picking   – gripper closing on the package at start
        delivering – gripper opening at the destination
        done      – mission finished successfully
        error     – an exception was raised (see ``error`` field)
        aborted   – user requested stop
    """

    def __init__(self, admin_state, slot_id):
        super().__init__(daemon=True)
        self.admin_state = admin_state
        self.slot_id = slot_id
        self.status = "starting"
        self.waypoints_done = 0
        self.waypoints_total = 0
        self.error = None
        self.started_at = time.monotonic()
        self._stop_requested = threading.Event()
        self._lock = threading.Lock()

    def request_stop(self):
        self._stop_requested.set()
        with self._lock:
            self.status = "aborted"

    def _set_status(self, value):
        with self._lock:
            if self.status != "aborted":
                self.status = value
        self.admin_state.log(f"[AUTO] {value}.")

    def snapshot(self):
        with self._lock:
            return {
                "status": self.status,
                "slot_id": self.slot_id,
                "waypoints_done": self.waypoints_done,
                "waypoints_total": self.waypoints_total,
                "elapsed_s": round(time.monotonic() - self.started_at, 1),
                "error": self.error,
            }

    def run(self):
        pose_publisher = None
        try:
            # Import here to avoid a hard import cost at admin-panel
            # startup (and to keep the admin panel importable even if
            # mission_controller has a transient bug).
            from mission_controller import MissionController

            self.admin_state.log(f"[AUTO] Starting autonomous drive to {self.slot_id}.")
            controller = MissionController(
                settings_path=self.admin_state.settings_path,
                qr_override=self.slot_id,
                dry_run=False,
            )

            if self._stop_requested.is_set():
                return

            # Publish the current pose to the admin panel at 5 Hz so
            # the map shows the rover moving in real time.
            pose_publisher = PosePublisher(controller, self.admin_state, self._stop_requested)
            pose_publisher.start()
            self.admin_state.log(f"[AUTO] Pose publisher started.")

            # Plan (A*) and count waypoints for the UI.
            from warehouse_slots import normalize_slot_id
            if self.slot_id.upper() == "HOME":
                # The admin panel's "Return home" target. Use the home
                # coordinate from settings / slot config.
                home = controller.settings.get("navigation", {}).get("home")
                if home is None:
                    home = controller.slots.home
                target = (int(home[0]), int(home[1]))
            else:
                destination = controller.slots.get_destination(normalize_slot_id(self.slot_id))
                target = destination.navigation_target
            target = destination.navigation_target
            pose = controller.odometry.pose
            start = (int(round(pose[0])), int(round(pose[1])))
            dense = controller.planner.plan_path(start, target)
            if not dense:
                raise RuntimeError(f"No path from {start} to {target}")
            waypoints = controller.planner.prune_path(dense)
            if waypoints and waypoints[0] == start:
                waypoints.pop(0)
            self.waypoints_total = len(waypoints)
            self._set_status("driving")

            # Patch the mission's waypoint follower so we can update
            # the "waypoints done" counter as the robot progresses.
            follow_waypoints = controller._follow_waypoints
            driver_self = self

            def _instrumented_follow(waypoints_in):
                remaining = list(waypoints_in)
                total = len(remaining)
                while remaining and not driver_self._stop_requested.is_set():
                    # Re-expose the remaining list to the status JSON.
                    with driver_self._lock:
                        driver_self.waypoints_done = total - len(remaining)
                    # Pop one waypoint at a time, delegating to the
                    # original method which already handles steering,
                    # obstacle stop, and dead-reckoning.
                    controller._follow_waypoints = follow_waypoints
                    controller._follow_waypoints(remaining[:1])
                    remaining.pop(0)
                with driver_self._lock:
                    driver_self.waypoints_done = total - len(remaining)
                if driver_self._stop_requested.is_set():
                    raise RuntimeError("auto-drive aborted by user")

            controller._follow_waypoints = _instrumented_follow
            controller.run_once()
            self._set_status("done")
            self.admin_state.log(f"[AUTO] Delivery to {self.slot_id} complete.")
        except Exception as exc:
            with self._lock:
                if self.status != "aborted":
                    self.status = "error"
                self.error = str(exc)
            self.admin_state.log(f"[AUTO] Aborted: {exc}")
        finally:
            self._stop_requested.set()
            if pose_publisher is not None:
                pose_publisher.join(timeout=0.5)
            try:
                self.admin_state.esp32.stop()
            except Exception:
                pass


# ============================================================== admin state
class AdminState:
    LOG_BUFFER_MAX = 2000

    def __init__(self, settings_path=None):
        self.project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.settings_path = settings_path or self.project_path("config/settings.yaml")
        self.settings = self.load_settings(self.settings_path)
        self.logs = []
        self.logs_lock = threading.Lock()
        # Verbose mode: when True, every motor command and pose update
        # is logged. Toggleable from the admin panel.
        self.verbose = True

        camera_cfg = self.settings.get("camera", {})
        self.camera = CameraWorker(
            camera_index=camera_cfg.get("index", 0),
            width=camera_cfg.get("width", 640),
            height=camera_cfg.get("height", 480),
        )

        serial_cfg = self.settings.get("serial", {})
        self.esp32 = ESP32Interface(
            port=serial_cfg.get("port", "/dev/ttyACM0"),
            baudrate=int(serial_cfg.get("baudrate", 115200)),
        )

        slots_path = self.project_path(self.settings.get("slots", {}).get("file", "config/warehouse_slots.yaml"))
        self.slots = WarehouseSlots.load(slots_path)
        self.grid = self.load_grid()
        self.started_at = time.monotonic()
        self.manual_busy = False
        self.manual_lock = threading.Lock()
        self.last_command = "none"
        self.pose = self._initial_pose()
        self.pose_lock = threading.Lock()
        self.pose_trail = []  # recent (x, y) samples for the map trail
        self.pose_trail_max = 200

        # Counter incremented every time the map-relevant state
        # changes. The browser polls this and only re-fetches the map
        # image when it changes — saves CPU and bandwidth.
        self.map_version = 0
        self._map_version_lock = threading.Lock()
        self._last_qr = None

        # Auto-drive state. `self.auto_driver` is None when idle, an
        # `AutoDriver` instance when a mission is running.
        self.auto_driver = None
        self.auto_lock = threading.Lock()

    def start(self):
        self.camera.start()
        self.log("RPi admin panel started.")
        if not self.esp32.simulation_mode:
            self.log(f"ESP32 connected on {self.esp32.ser.port}.")
        else:
            self.log("ESP32 serial is not connected; command buttons are in simulation/no-op mode.")

    # ----------------------------------------------------------- helpers
    def project_path(self, relative_path):
        if os.path.isabs(relative_path):
            return relative_path
        return os.path.join(self.project_dir, relative_path)

    def _initial_pose(self):
        home = self.settings.get("navigation", {}).get("home")
        if home is None:
            home = self.slots.home
        return [float(home[0]), float(home[1]), 0.0]

    def reset_pose(self):
        """Reset the visible pose back to the configured home."""
        home = self._initial_pose()
        with self.pose_lock:
            self.pose = list(home)
            self.pose_trail.clear()
        self.bump_map_version()
        self.log(f"[STATE] Pose reset to home {home}.")

    def set_pose(self, x, y, theta=None, source="odometry"):
        """Update the visible pose. Called by the AutoDriver thread
        periodically so the map shows live position."""
        with self.pose_lock:
            self.pose[0] = float(x)
            self.pose[1] = float(y)
            if theta is not None:
                self.pose[2] = float(theta)
            self.pose_trail.append((float(x), float(y)))
            if len(self.pose_trail) > self.pose_trail_max:
                self.pose_trail = self.pose_trail[-self.pose_trail_max:]
        self.bump_map_version()
        if self.verbose and source == "odometry":
            # Throttle verbose pose logs to one per second.
            now = time.monotonic()
            if not hasattr(self, "_last_pose_log") or now - self._last_pose_log > 1.0:
                self._last_pose_log = now
                self.log(f"[POSE] x={x:.1f} y={y:.1f} theta={self.pose[2]:.2f}")

    def bump_map_version(self):
        with self._map_version_lock:
            self.map_version += 1

    def get_pose_snapshot(self):
        with self.pose_lock:
            return {
                "x": self.pose[0],
                "y": self.pose[1],
                "theta": self.pose[2],
                "trail": list(self.pose_trail),
            }

    def load_grid(self):
        if np is None:
            return None
        grid_path = self.project_path(self.settings.get("map", {}).get("grid", "maps/occupancy_grid.npy"))
        map_cfg = self.settings.get("map", {})
        resolution = float(map_cfg.get("resolution_cm_per_px", 1.0))
        length_cm = int(map_cfg.get("length_cm", 200))
        width_cm = int(map_cfg.get("width_cm", 200))
        expected_w = max(1, int(round(length_cm / resolution)))
        expected_h = max(1, int(round(width_cm / resolution)))

        if os.path.exists(grid_path):
            try:
                grid = np.load(grid_path)
                # If the saved grid doesn't match the configured size,
                # build a fresh empty one of the right shape.
                if grid.shape == (expected_h, expected_w):
                    return grid
                self.log(
                    f"Saved grid shape {grid.shape} != expected "
                    f"({expected_h}, {expected_w}) from length/width config; "
                    f"rebuilding empty grid."
                )
            except Exception as exc:
                self.log(f"Could not load occupancy grid: {exc}")

        # No valid saved grid -> synthesise a fully empty one matching
        # the configured length/width. (Obstacles would have been
        # baked into the .npy by setup.sh / MapProcessor.)
        empty = np.zeros((expected_h, expected_w), dtype=np.uint8)
        self.log(
            f"Built empty occupancy grid {expected_w}x{expected_h} px "
            f"({length_cm}x{width_cm} cm @ {resolution} cm/px)."
        )
        return empty

    def warehouse_dimensions(self):
        """Return the warehouse dimensions in cm, px, and a human label."""
        map_cfg = self.settings.get("map", {})
        resolution = float(map_cfg.get("resolution_cm_per_px", 1.0))
        length_cm = int(map_cfg.get("length_cm", 200))
        width_cm = int(map_cfg.get("width_cm", 200))
        if self.grid is not None:
            h, w = self.grid.shape
            length_cm = int(round(w * resolution))
            width_cm = int(round(h * resolution))
        return {
            "length_cm": length_cm,
            "width_cm": width_cm,
            "resolution_cm_per_px": resolution,
            "length_px": int(round(length_cm / resolution)),
            "width_px": int(round(width_cm / resolution)),
            "grid_step_cm": int(map_cfg.get("grid_step_cm", 50)),
        }

    def update_config(self, length_cm=None, width_cm=None, manual_speed=None,
                      resolution_cm_per_px=None):
        """Mutate ``self.settings`` and persist to ``settings.yaml``."""
        changed = False
        map_cfg = self.settings.setdefault("map", {})
        admin_cfg = self.settings.setdefault("admin", {})
        if length_cm is not None and length_cm > 0:
            map_cfg["length_cm"] = int(length_cm)
            changed = True
        if width_cm is not None and width_cm > 0:
            map_cfg["width_cm"] = int(width_cm)
            changed = True
        if resolution_cm_per_px is not None and resolution_cm_per_px > 0:
            map_cfg["resolution_cm_per_px"] = float(resolution_cm_per_px)
            changed = True
        if manual_speed is not None and 0 < manual_speed <= 255:
            admin_cfg["manual_speed"] = int(manual_speed)
            changed = True
        if changed:
            with open(self.settings_path, "w", encoding="utf-8") as fh:
                yaml.safe_dump(self.settings, fh, sort_keys=False)
            # If length/width changed, rebuild the grid shape.
            if length_cm is not None or width_cm is not None or resolution_cm_per_px is not None:
                self.grid = self.load_grid()

    def destination_for_qr(self, qr_value):
        if not qr_value:
            return None
        try:
            destination = self.slots.get_destination(qr_value)
            return {
                "slot_id": destination.slot_id,
                "drop": list(destination.drop),
                "approach": list(destination.approach) if destination.approach else None,
                "navigation_target": list(destination.navigation_target),
            }
        except Exception as exc:
            return {"error": str(exc)}

    def route_for_qr(self, qr_value):
        if self.grid is None or not qr_value:
            return None
        destination = self.destination_for_qr(qr_value)
        if not destination or destination.get("error"):
            return None

        start = (int(round(self.pose[0])), int(round(self.pose[1])))
        target = tuple(destination["navigation_target"])
        planner = AStarPlanner(self.grid)
        dense_path = planner.plan_path(start, target)
        if not dense_path:
            return {"error": f"No path from {start} to {target}"}
        return {"waypoints": [list(point) for point in planner.prune_path(dense_path)]}

    def render_map_jpeg(self, qr_value=None):
        """Render the occupancy grid with the planned route, pose, and
        live trail as JPEG bytes."""
        if self.grid is None or np is None:
            return None

        # Base visualisation: free=white, obstacle=black.
        grid = self.grid
        display = np.where(grid == 0, 255, 0).astype(np.uint8)
        display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

        # Draw a faint cm grid overlay if configured.
        wh = self.warehouse_dimensions()
        step_cm = max(1, wh["grid_step_cm"])
        resolution = wh["resolution_cm_per_px"]
        if resolution > 0:
            step_px = max(2, int(round(step_cm / resolution)))
            h, w = display.shape[:2]
            grid_color = (210, 210, 210)
            for x in range(0, w, step_px):
                cv2.line(display, (x, 0), (x, h), grid_color, 1)
            for y in range(0, h, step_px):
                cv2.line(display, (0, y), (w, y), grid_color, 1)

        # Mark known slot drop points in red.
        for slot_id, dest in self.slots.slots.items():
            cv2.circle(display, tuple(dest.drop), 3, (0, 0, 255), -1)

        # Mark home position in blue.
        home = self.settings.get("navigation", {}).get("home", [20, 20])
        cv2.circle(display, tuple(home), 4, (255, 0, 0), -1)

        # Draw the pose trail (recent positions) in light green.
        with self.pose_lock:
            trail = list(self.pose_trail)
            pose = list(self.pose)
        if len(trail) >= 2:
            pts = [(int(round(x)), int(round(y))) for x, y in trail[-200:]]
            for i in range(len(pts) - 1):
                cv2.line(display, pts[i], pts[i + 1], (140, 255, 140), 1)

        # Mark the current pose as a filled green dot with a heading arrow.
        cv2.circle(display, (int(round(pose[0])), int(round(pose[1]))), 5, (0, 255, 0), -1)
        heading_len = 10
        cv2.line(
            display,
            (int(round(pose[0])), int(round(pose[1]))),
            (
                int(round(pose[0] + heading_len * math.cos(pose[2]))),
                int(round(pose[1] + heading_len * math.sin(pose[2]))),
            ),
            (0, 255, 0),
            2,
        )

        # Draw the planned route.
        route = self.route_for_qr(qr_value)
        if route and route.get("waypoints"):
            waypoints = route["waypoints"]
            pts = [(int(round(pose[0])), int(round(pose[1])))] + [
                (int(x), int(y)) for x, y in waypoints
            ]
            for i in range(len(pts) - 1):
                cv2.line(display, pts[i], pts[i + 1], (0, 165, 255), 2)
            cv2.circle(display, pts[-1], 5, (0, 165, 255), -1)

        # Scale bar: 10% of width = X cm.
        h, w = display.shape[:2]
        bar_len_px = max(20, int(round(w * 0.10)))
        bar_cm = int(round(bar_len_px * resolution))
        bar_y = h - 10
        bar_x0 = 10
        cv2.line(display, (bar_x0, bar_y), (bar_x0 + bar_len_px, bar_y), (255, 255, 255), 2)
        cv2.putText(
            display,
            f"{bar_cm} cm",
            (bar_x0, bar_y - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
        )

        ok, encoded = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        return encoded.tobytes()

    # ---------------------------------------------------------- controls
    def send_action(self, action, speed=None):
        action = str(action).strip().lower()
        if speed is None:
            speed = int(self.settings.get("admin", {}).get("manual_speed", 90))
        else:
            speed = max(0, min(255, int(speed)))
        duration = float(self.settings.get("admin", {}).get("nudge_duration_s", 0.35))

        if action == "stop":
            self.esp32.stop()
            self.last_command = "stop"
            self.log("Sent stop command.")
            return
        if action == "gripper_open":
            self.esp32.send_gripper_cmd("OPEN")
            self.last_command = "gripper open"
            self.log("Sent gripper open command.")
            return
        if action == "gripper_close":
            self.esp32.send_gripper_cmd("CLOSE")
            self.last_command = "gripper close"
            self.log("Sent gripper close command.")
            return
        if action == "ping":
            ok = self.esp32.ping()
            self.last_command = "ping"
            self.log(f"Ping -> {'PONG' if ok else 'no response'}.")
            return

        motions = {
            "forward": (speed, speed),
            "backward": (-speed, -speed),
            "left": (-speed, speed),
            "right": (speed, -speed),
        }
        if action not in motions:
            raise ValueError(f"Unknown action: {action}")

        with self.manual_lock:
            if self.manual_busy:
                raise RuntimeError("A manual nudge is already running")
            self.manual_busy = True

        left_speed, right_speed = motions[action]
        self.last_command = action
        self.log(f"Manual nudge: {action} for {duration:.2f}s at speed {speed}.")
        threading.Thread(
            target=self._run_nudge,
            args=(left_speed, right_speed, duration),
            daemon=True,
        ).start()

    def _run_nudge(self, left_speed, right_speed, duration):
        try:
            self.esp32.send_velocity_cmd(left_speed, right_speed)
            self.log_motor(left_speed, right_speed)
            time.sleep(duration)
            self.esp32.stop()
            self.log_motor(0, 0)
        finally:
            with self.manual_lock:
                self.manual_busy = False

    # --------------------------------------------------------- auto-drive
    def start_auto_drive(self, slot_id):
        """
        Launch an autonomous drive to ``slot_id`` in a background thread.

        Returns ``(ok: bool, payload: dict)``. ``ok=False`` and an
        ``error`` field is returned if a mission is already running.
        """
        with self.auto_lock:
            if self.auto_driver is not None and self.auto_driver.is_alive():
                return False, {"error": "auto-drive already running"}
            self.auto_driver = AutoDriver(self, slot_id)
            self.auto_driver.start()
        return True, {"auto_drive": self.auto_drive_status()}

    def stop_auto_drive(self):
        """Best-effort stop: halts the motors and asks the loop to abort."""
        with self.auto_lock:
            driver = self.auto_driver
        if driver is not None:
            driver.request_stop()
        self.esp32.stop()
        return {"auto_drive": self.auto_drive_status()}

    def auto_drive_status(self):
        with self.auto_lock:
            driver = self.auto_driver
        if driver is None:
            return {"status": "idle"}
        return driver.snapshot()

    # --------------------------------------------------------- status json
    def status(self):
        camera = self.camera.get_snapshot()
        telemetry = self.esp32.get_telemetry()
        qr_value = camera["latest_qr"]
        # If the QR changed, bump the map version so the browser
        # refetches the map preview with the new planned route.
        if qr_value != self._last_qr:
            self._last_qr = qr_value
            self.bump_map_version()
        with self._map_version_lock:
            map_version = self.map_version
        return {
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "camera_ok": camera["frame_ok"],
            "qr": qr_value,
            "qr_error": camera["latest_qr_error"],
            "destination": self.destination_for_qr(qr_value),
            "route": self.route_for_qr(qr_value),
            "esp32_connected": not self.esp32.simulation_mode,
            "telemetry": {
                "distance_cm": telemetry.distance_cm,
                "left_ticks": telemetry.left_ticks,
                "right_ticks": telemetry.right_ticks,
                "yaw_deg": telemetry.yaw_deg,
                "age_s": round(time.monotonic() - telemetry.updated_at, 1),
            },
            "pose": self.get_pose_snapshot(),
            "warehouse": self.warehouse_dimensions(),
            "manual_busy": self.manual_busy,
            "last_command": self.last_command,
            "auto_drive": self.auto_drive_status(),
            "map_version": map_version,
            "verbose": self.verbose,
            "slots": sorted(self.slots.slots.keys()),
            "logs": self.get_logs(),
        }

    def log(self, message):
        entry = f"[{time.strftime('%H:%M:%S')}] {message}"
        print(entry, flush=True)
        with self.logs_lock:
            self.logs.append(entry)
            self.logs = self.logs[-self.LOG_BUFFER_MAX:]

    def log_motor(self, left, right):
        """Verbose log of every motor command. No-op if verbose=False."""
        if not self.verbose:
            return
        self.log(f"[MOTOR] L={left:+d} R={right:+d}")

    def log_pose(self, x, y, theta, source="odometry"):
        if not self.verbose:
            return
        self.log(f"[POSE:{source}] x={x:.1f} y={y:.1f} theta={math.degrees(theta):.1f} deg")

    def log_waypoint(self, idx, total, target):
        self.log(f"[NAV] Reached waypoint {idx}/{total} at {target}")

    def get_logs(self):
        with self.logs_lock:
            return list(self.logs)

    @staticmethod
    def load_settings(path):
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}


# ============================================================ http handler
class AdminRequestHandler(BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self.send_html(INDEX_HTML)
        elif parsed.path == "/video":
            self.stream_video()
        elif parsed.path == "/api/status":
            self.send_json(self.state.status())
        elif parsed.path == "/api/snapshot.jpg":
            self.send_camera_snapshot()
        elif parsed.path == "/api/map.jpg":
            self.send_map_snapshot()
        elif parsed.path == "/api/logs":
            # Returns the full log buffer (up to 2000 lines).
            self.send_json({"logs": self.state.get_logs()})
        elif parsed.path == "/api/logs/download":
            data = ("\n".join(self.state.get_logs()) + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header(
                "Content-Disposition",
                'attachment; filename="rover.log"',
            )
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_error(404, "Not found")

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        params = urllib.parse.parse_qs(parsed.query)

        if parsed.path == "/api/command":
            action = params.get("action", [""])[0]
            speed = params.get("speed", [None])[0]
            try:
                self.state.send_action(action, speed=speed)
                self.send_json({"ok": True})
            except Exception as exc:
                self.state.log(f"Command failed: {exc}")
                self.send_json({"ok": False, "error": str(exc)}, status=400)
            return

        if parsed.path == "/api/drive":
            action = params.get("action", ["start"])[0]
            if action == "stop":
                payload = self.state.stop_auto_drive()
                self.send_json({"ok": True, **payload})
                return
            # action == "start"
            slot_id = params.get("slot", [""])[0]
            try:
                ok, payload = self.state.start_auto_drive(slot_id)
            except Exception as exc:
                self.state.log(f"Auto-drive failed to start: {exc}")
                self.send_json({"ok": False, "error": str(exc)}, status=400)
                return
            if ok:
                self.send_json({"ok": True, **payload})
            else:
                self.send_json({"ok": False, **payload}, status=409)
            return

        if parsed.path == "/api/return_home":
            try:
                home = self.state.settings.get("navigation", {}).get("home")
                if home is None:
                    home = self.state.slots.home
                slot_id = "HOME"
                ok, payload = self.state.start_auto_drive(slot_id)
                if not ok:
                    # If a mission is already running, refuse; user
                    # should hit Stop first.
                    self.send_json({"ok": False, **payload}, status=409)
                    return
                self.send_json({"ok": True, "home": list(home)})
            except Exception as exc:
                self.state.log(f"return_home failed: {exc}")
                self.send_json({"ok": False, "error": str(exc)}, status=400)
            return

        if parsed.path == "/api/emergency_stop":
            try:
                self.state.esp32.stop()
                self.state.stop_auto_drive()
                self.state.log("[EMERGENCY] Emergency stop triggered.")
                self.send_json({"ok": True})
            except Exception as exc:
                self.state.log(f"emergency_stop failed: {exc}")
                self.send_json({"ok": False, "error": str(exc)}, status=400)
            return

        if parsed.path == "/api/pose/reset":
            try:
                self.state.reset_pose()
                self.send_json({"ok": True})
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, status=400)
            return

        if parsed.path == "/api/verbose":
            value = params.get("value", ["true"])[0].lower() in ("1", "true", "yes", "on")
            self.state.verbose = value
            self.state.log(f"[CONFIG] Verbose logging {'enabled' if value else 'disabled'}.")
            self.send_json({"ok": True, "verbose": self.state.verbose})
            return

        if parsed.path == "/api/config":
            # POST /api/config with form fields for warehouse dims
            # and manual speed. Saves them to settings.yaml.
            try:
                length = params.get("length_cm", [None])[0]
                width = params.get("width_cm", [None])[0]
                speed = params.get("manual_speed", [None])[0]
                resolution = params.get("resolution_cm_per_px", [None])[0]
                self.state.update_config(
                    length_cm=int(length) if length else None,
                    width_cm=int(width) if width else None,
                    manual_speed=int(speed) if speed else None,
                    resolution_cm_per_px=float(resolution) if resolution else None,
                )
                self.state.log(
                    f"[CONFIG] Updated: "
                    f"length={self.state.settings['map'].get('length_cm')} cm, "
                    f"width={self.state.settings['map'].get('width_cm')} cm, "
                    f"manual_speed={self.state.settings['admin'].get('manual_speed')}, "
                    f"resolution={self.state.settings['map'].get('resolution_cm_per_px')} cm/px."
                )
                self.state.bump_map_version()
                self.send_json({"ok": True, "warehouse": self.state.warehouse_dimensions()})
            except Exception as exc:
                self.state.log(f"config update failed: {exc}")
                self.send_json({"ok": False, "error": str(exc)}, status=400)
            return

        self.send_error(404, "Not found")

    def stream_video(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        while True:
            snapshot = self.state.camera.get_snapshot()
            frame = snapshot["latest_jpeg"]
            if frame:
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
            time.sleep(0.08)

    def send_camera_snapshot(self):
        frame = self.state.camera.get_snapshot()["latest_jpeg"]
        if not frame:
            self.send_error(503, "No camera frame available")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def send_map_snapshot(self):
        camera = self.state.camera.get_snapshot()
        frame = self.state.render_map_jpeg(camera["latest_qr"])
        if not frame:
            self.send_error(503, "No map available")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def send_html(self, html):
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, payload, status=200):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        return


INDEX_HTML = r"""<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Warehouse Rover Admin</title>
  <style>
    :root { color-scheme: dark; }
    body { margin: 0; font-family: Arial, sans-serif; background: #111827; color: #e5e7eb; }
    header { padding: 12px 18px; background: #0f172a; border-bottom: 1px solid #334155; display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 8px; }
    h1 { margin: 0; font-size: 20px; }
    .header-status { font-size: 13px; color: #9ca3af; }
    main { padding: 14px; display: grid; gap: 12px; grid-template-columns: minmax(280px, 1.4fr) minmax(280px, 1fr); }
    section { background: #1f2937; border: 1px solid #374151; border-radius: 8px; padding: 12px; }
    h2 { margin: 0 0 8px 0; font-size: 15px; }
    img { width: 100%; background: #020617; border-radius: 6px; display: block; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 8px; }
    .card { background: #111827; border: 1px solid #374151; border-radius: 6px; padding: 8px; min-height: 60px; }
    .label { color: #9ca3af; font-size: 11px; margin-bottom: 4px; }
    .value { font-size: 16px; font-weight: 700; overflow-wrap: anywhere; }
    .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; max-width: 320px; }
    button { background: #2563eb; color: white; border: 0; border-radius: 6px; padding: 8px; font-weight: 700; cursor: pointer; font-size: 13px; }
    button:hover { filter: brightness(1.1); }
    button.stop { background: #dc2626; }
    button.warn { background: #f59e0b; color: #1f2937; }
    button.secondary { background: #475569; }
    button:disabled { opacity: 0.4; cursor: not-allowed; }
    pre { background: #020617; color: #d1d5db; border-radius: 6px; padding: 8px; min-height: 160px; max-height: 280px; white-space: pre-wrap; overflow: auto; font-size: 12px; }
    .logs { max-height: 320px; }
    ul { margin: 0; padding-left: 18px; font-size: 13px; }
    input, select { background: #0f172a; color: #e5e7eb; border: 1px solid #374151; border-radius: 4px; padding: 6px; font-size: 13px; }
    label { color: #9ca3af; font-size: 12px; }
    .row { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
    .pill { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 11px; font-weight: 700; }
    .pill-on  { background: #16a34a; }
    .pill-off { background: #475569; }
    .pill-err { background: #dc2626; }
    .pill-run { background: #2563eb; }
    @media (max-width: 820px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>Warehouse Rover Admin</h1>
    <span class="header-status">
      <span class="pill pill-on" id="connPill">Connecting...</span>
      <span style="margin-left: 8px;">uptime <span id="uptime">-</span></span>
    </span>
  </header>
  <main>
    <section>
      <h2>Live RPi Camera</h2>
      <img src="/video" alt="Live camera feed">
      <div class="row" style="margin-top: 8px;">
        <span>QR: <strong id="qr">-</strong></span>
        <span class="pill" id="qrPill" style="display: none;"></span>
        <span style="margin-left: auto; color: #9ca3af; font-size: 12px;">Camera: <span id="camera">-</span></span>
      </div>
    </section>
    <section>
      <h2>Warehouse Map + Route</h2>
      <img id="mappng" src="/api/map.jpg" alt="Warehouse map with planned route">
      <div id="mapMeta" style="margin-top: 6px; color: #9ca3af; font-size: 12px;">-</div>
    </section>

    <section>
      <h2>Status</h2>
      <div class="grid">
        <div class="card"><div class="label">ESP32</div><div id="esp32" class="value">-</div></div>
        <div class="card"><div class="label">Distance</div><div id="distance" class="value">-</div></div>
        <div class="card"><div class="label">Yaw</div><div id="yaw" class="value">-</div></div>
        <div class="card"><div class="label">Pose X,Y</div><div id="posexy" class="value">-</div></div>
        <div class="card"><div class="label">Heading</div><div id="heading" class="value">-</div></div>
        <div class="card"><div class="label">Last cmd</div><div id="lastcmd" class="value">-</div></div>
        <div class="card"><div class="label">Auto-drive</div><div id="autoDrive" class="value">idle</div></div>
        <div class="card"><div class="label">Map ver</div><div id="mapver" class="value">-</div></div>
      </div>
    </section>

    <section>
      <h2>Manual Drive</h2>
      <div class="row" style="margin-bottom: 8px;">
        <label for="speedSlider">Manual speed:</label>
        <input type="range" id="speedSlider" min="20" max="255" value="90" style="flex: 1; min-width: 100px;">
        <span id="speedVal" style="font-family: monospace; width: 36px;">90</span>
      </div>
      <div class="controls">
        <span></span><button onclick="cmd('forward')">Forward</button><span></span>
        <button onclick="cmd('left')">Left</button><button class="stop" onclick="cmd('stop')">Stop</button><button onclick="cmd('right')">Right</button>
        <span></span><button onclick="cmd('backward')">Backward</button><span></span>
      </div>
      <div class="row" style="margin-top: 8px;">
        <button class="secondary" onclick="cmd('gripper_open')">Open Gripper</button>
        <button class="secondary" onclick="cmd('gripper_close')">Close Gripper</button>
        <button class="secondary" onclick="pingEsp()">Ping</button>
      </div>
      <p id="commandResult" style="margin: 8px 0 0 0; font-size: 12px; color: #9ca3af;"></p>
    </section>

    <section>
      <h2>Auto-drive</h2>
      <p style="margin: 0 0 8px 0; color: #9ca3af; font-size: 12px;">
        Plan a path with A* and follow it autonomously. The gripper
        closes at the start (pick up) and opens at the destination (drop).
      </p>
      <div class="row" style="margin-bottom: 8px;">
        <label for="slotSelect">Slot:</label>
        <select id="slotSelect"></select>
        <button id="driveStartBtn" onclick="driveStart()">Start</button>
        <button class="stop" id="driveStopBtn" onclick="driveStop()" disabled>Stop</button>
      </div>
      <div class="row" style="margin-bottom: 8px;">
        <button class="secondary" onclick="returnHome()">Return home</button>
        <button class="secondary" onclick="resetPose()">Reset pose</button>
        <button class="warn" onclick="emergencyStop()">EMERGENCY STOP</button>
      </div>
      <p id="driveStatus" style="margin: 0; font-family: monospace; font-size: 13px;">Status: idle</p>
    </section>

    <section>
      <h2>Warehouse Setup</h2>
      <p style="margin: 0 0 8px 0; color: #9ca3af; font-size: 12px;">
        Set the real-world warehouse size. The occupancy grid is
        regenerated to match. Slots in <code>warehouse_slots.yaml</code>
        still use pixel coordinates (cm / resolution).
      </p>
      <div class="row" style="margin-bottom: 6px;">
        <label>Length (cm):</label>
        <input type="number" id="lengthInput" min="10" max="10000" value="200" style="width: 90px;">
        <label>Width (cm):</label>
        <input type="number" id="widthInput" min="10" max="10000" value="200" style="width: 90px;">
      </div>
      <div class="row" style="margin-bottom: 6px;">
        <label>Resolution (cm/px):</label>
        <input type="number" id="resolutionInput" min="0.1" max="100" step="0.1" value="1.0" style="width: 90px;">
        <label>Manual speed:</label>
        <input type="number" id="manualSpeedInput" min="20" max="255" value="90" style="width: 90px;">
      </div>
      <div class="row">
        <button onclick="applyConfig()">Apply &amp; save</button>
        <span id="configResult" style="font-size: 12px; color: #9ca3af;"></span>
      </div>
    </section>

    <section>
      <h2>Destination</h2>
      <pre id="destination">-</pre>
    </section>

    <section>
      <h2>Known Slots</h2>
      <ul id="slots"></ul>
    </section>

    <section style="grid-column: 1 / -1;">
      <h2>Live Logs
        <span class="row" style="float: right; gap: 6px;">
          <label style="font-size: 12px;"><input type="checkbox" id="autoscrollChk" checked> auto-scroll</label>
          <label style="font-size: 12px;"><input type="checkbox" id="verboseChk" checked> verbose</label>
          <button class="secondary" style="padding: 4px 10px; font-size: 12px;" onclick="downloadLogs()">Download .log</button>
          <button class="secondary" style="padding: 4px 10px; font-size: 12px;" onclick="clearLogs()">Clear view</button>
        </span>
      </h2>
      <pre id="logs" class="logs"></pre>
    </section>
  </main>
  <script>
    let lastMapVersion = -1;
    let lastLogsHash = 0;
    const autoscroll = () => {
      const el = document.getElementById('logs');
      if (document.getElementById('autoscrollChk').checked) {
        el.scrollTop = el.scrollHeight;
      }
    };
    async function refresh() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();
        // Connection pill
        const pill = document.getElementById('connPill');
        pill.textContent = data.esp32_connected ? 'ESP32 OK' : 'SIMULATION';
        pill.className = 'pill ' + (data.esp32_connected ? 'pill-on' : 'pill-off');
        // Status cards
        document.getElementById('esp32').textContent = data.esp32_connected ? 'Connected' : 'Sim';
        document.getElementById('distance').textContent = Number(data.telemetry.distance_cm).toFixed(1) + ' cm';
        document.getElementById('yaw').textContent = Number(data.telemetry.yaw_deg).toFixed(1) + ' deg';
        document.getElementById('uptime').textContent = data.uptime_s + ' s';
        document.getElementById('camera').textContent = data.camera_ok ? 'Live' : 'No frame';
        document.getElementById('lastcmd').textContent = data.last_command;
        document.getElementById('mapver').textContent = data.map_version;
        document.getElementById('qr').textContent = data.qr || '-';
        const qrPill = document.getElementById('qrPill');
        if (data.qr) {
          qrPill.style.display = 'inline-block';
          qrPill.textContent = 'detected';
          qrPill.className = 'pill pill-on';
        } else {
          qrPill.style.display = 'none';
        }
        // Pose
        const p = data.pose || {x: 0, y: 0, theta: 0};
        document.getElementById('posexy').textContent =
          Math.round(p.x) + ', ' + Math.round(p.y);
        const headingDeg = ((p.theta * 180 / Math.PI) % 360 + 360) % 360;
        document.getElementById('heading').textContent = headingDeg.toFixed(0) + ' deg';
        // Warehouse meta
        const wh = data.warehouse || {};
        document.getElementById('mapMeta').textContent =
          wh.length_cm + ' x ' + wh.width_cm + ' cm  (' +
          wh.length_px + ' x ' + wh.width_px + ' px @ ' +
          wh.resolution_cm_per_px + ' cm/px)';
        // Populate length/width inputs on first run.
        if (!document.getElementById('lengthInput').dataset.set) {
          document.getElementById('lengthInput').value = wh.length_cm || 200;
          document.getElementById('widthInput').value = wh.width_cm || 200;
          document.getElementById('resolutionInput').value = wh.resolution_cm_per_px || 1.0;
          document.getElementById('lengthInput').dataset.set = '1';
        }
        // Manual speed sync
        if (data.verbose !== undefined && document.getElementById('verboseChk').checked !== data.verbose) {
          document.getElementById('verboseChk').checked = data.verbose;
        }
        // Destination / route
        document.getElementById('destination').textContent = JSON.stringify(
          {destination: data.destination, route: data.route}, null, 2);
        // Logs
        const logsEl = document.getElementById('logs');
        const logsText = data.logs.join('\n');
        const h = simpleHash(logsText);
        if (h !== lastLogsHash) {
          logsEl.textContent = logsText;
          lastLogsHash = h;
          autoscroll();
        }
        // Slots list and selector
        const slotList = data.slots;
        document.getElementById('slots').innerHTML = slotList.map(s => '<li>' + s + '</li>').join('');
        const sel = document.getElementById('slotSelect');
        if (sel.options.length === 0 && slotList.length > 0) {
          for (const s of slotList) {
            const opt = document.createElement('option');
            opt.value = s;
            opt.textContent = s;
            sel.appendChild(opt);
          }
        }
        // Auto-drive status
        const ad = data.auto_drive || {status: 'idle'};
        const adText = ad.status === 'idle'
          ? 'idle'
          : (ad.slot_id ? ad.slot_id + ' - ' + ad.status : ad.status)
            + (ad.waypoints_total ? ' (' + ad.waypoints_done + '/' + ad.waypoints_total + ')' : '')
            + (ad.elapsed_s ? ' ' + ad.elapsed_s + 's' : '')
            + (ad.error ? ' - ' + ad.error : '');
        document.getElementById('autoDrive').textContent = adText;
        document.getElementById('driveStatus').textContent = 'Status: ' + adText;
        const adActive = ['starting', 'planning', 'driving', 'picking', 'delivering'].includes(ad.status);
        document.getElementById('driveStartBtn').disabled = adActive;
        document.getElementById('driveStopBtn').disabled = !adActive;
        // Refresh map only when its version changes.
        if (data.map_version !== lastMapVersion) {
          lastMapVersion = data.map_version;
          document.getElementById('mappng').src = '/api/map.jpg?t=' + Date.now();
        }
      } catch (e) {
        // ignore transient fetch errors
      }
    }
    function simpleHash(s) {
      // Cheap string hash for change detection.
      let h = 0;
      for (let i = 0; i < s.length; i++) { h = ((h << 5) - h + s.charCodeAt(i)) | 0; }
      return h;
    }
    async function postJson(url) {
      const r = await fetch(url, {method: 'POST'});
      return await r.json();
    }
    async function cmd(action) {
      // Use the slider speed for forward / backward / left / right.
      const speed = parseInt(document.getElementById('speedSlider').value, 10);
      const url = '/api/command?action=' + encodeURIComponent(action) +
                  (action === 'forward' || action === 'backward' || action === 'left' || action === 'right'
                   ? '&speed=' + speed : '');
      const data = await postJson(url);
      document.getElementById('commandResult').textContent = data.ok ? 'Sent: ' + action : 'Error: ' + (data.error || '?');
      refresh();
    }
    async function pingEsp() {
      const data = await postJson('/api/command?action=ping');
      document.getElementById('commandResult').textContent = data.ok ? ('Sent ping - ' + (data.result || 'see logs')) : 'Error';
      refresh();
    }
    async function driveStart() {
      const slot = document.getElementById('slotSelect').value;
      const data = await postJson('/api/drive?action=start&slot=' + encodeURIComponent(slot));
      document.getElementById('driveStatus').textContent = data.ok
        ? 'Status: starting drive to ' + slot
        : 'Status: error - ' + (data.error || 'unknown');
      refresh();
    }
    async function driveStop() {
      const data = await postJson('/api/drive?action=stop');
      document.getElementById('driveStatus').textContent = 'Status: stop requested';
      refresh();
    }
    async function returnHome() {
      const data = await postJson('/api/return_home');
      document.getElementById('driveStatus').textContent = data.ok
        ? 'Status: returning home'
        : 'Status: error - ' + (data.error || 'busy');
      refresh();
    }
    async function resetPose() {
      const data = await postJson('/api/pose/reset');
      document.getElementById('commandResult').textContent = data.ok ? 'Pose reset' : 'Error';
      refresh();
    }
    async function emergencyStop() {
      if (!confirm('Send EMERGENCY STOP? This halts all motors and aborts any mission.')) return;
      const data = await postJson('/api/emergency_stop');
      document.getElementById('driveStatus').textContent = 'EMERGENCY STOP sent';
      refresh();
    }
    async function applyConfig() {
      const length = document.getElementById('lengthInput').value;
      const width = document.getElementById('widthInput').value;
      const resolution = document.getElementById('resolutionInput').value;
      const speed = document.getElementById('manualSpeedInput').value;
      const url = '/api/config?length_cm=' + length +
                  '&width_cm=' + width +
                  '&resolution_cm_per_px=' + resolution +
                  '&manual_speed=' + speed;
      const data = await postJson(url);
      document.getElementById('configResult').textContent = data.ok
        ? 'Saved: ' + (data.warehouse.length_cm + 'x' + data.warehouse.width_cm + ' cm')
        : 'Error: ' + (data.error || '?');
      refresh();
    }
    async function downloadLogs() {
      window.location = '/api/logs/download';
    }
    function clearLogs() {
      document.getElementById('logs').textContent = '';
      lastLogsHash = 0;
    }
    document.getElementById('speedSlider').addEventListener('input', (e) => {
      document.getElementById('speedVal').textContent = e.target.value;
    });
    document.getElementById('verboseChk').addEventListener('change', async (e) => {
      await postJson('/api/verbose?value=' + (e.target.checked ? 'true' : 'false'));
      refresh();
    });
    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>"""


def main():
    parser = argparse.ArgumentParser(description="Run the RPi4 warehouse rover admin panel.")
    parser.add_argument("--settings", default=None, help="Path to settings.yaml")
    parser.add_argument("--host", default=None, help="Override admin.host from settings")
    parser.add_argument("--port", type=int, default=None, help="Override admin.port from settings")
    args = parser.parse_args()

    state = AdminState(settings_path=args.settings)
    state.start()

    admin_cfg = state.settings.get("admin", {})
    host = args.host or admin_cfg.get("host", "0.0.0.0")
    port = args.port or int(admin_cfg.get("port", 8080))

    AdminRequestHandler.state = state
    server = ThreadingHTTPServer((host, port), AdminRequestHandler)
    state.log(f"Open admin panel at http://{host}:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        state.log("Admin panel shutting down.")
    finally:
        state.camera.stop()
        state.esp32.stop()
        server.server_close()


if __name__ == "__main__":
    main()
