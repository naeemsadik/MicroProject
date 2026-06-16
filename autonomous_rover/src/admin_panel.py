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


# ============================================================== admin state
class AdminState:
    def __init__(self, settings_path=None):
        self.project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.settings_path = settings_path or self.project_path("config/settings.yaml")
        self.settings = self.load_settings(self.settings_path)
        self.logs = []
        self.logs_lock = threading.Lock()

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

    def load_grid(self):
        if np is None:
            return None
        grid_path = self.project_path(self.settings.get("map", {}).get("grid", "maps/occupancy_grid.npy"))
        if not os.path.exists(grid_path):
            return None
        try:
            return np.load(grid_path)
        except Exception as exc:
            self.log(f"Could not load occupancy grid: {exc}")
            return None

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
        """Render the occupancy grid with the planned route as JPEG bytes."""
        if self.grid is None or np is None:
            return None

        # Build a base visualization: free=white, obstacle=black, but invert for display.
        grid = self.grid
        display = np.where(grid == 0, 255, 0).astype(np.uint8)
        display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

        # Mark home position in blue.
        home = self.settings.get("navigation", {}).get("home", [20, 20])
        cv2.circle(display, tuple(home), 4, (255, 0, 0), -1)

        # Mark robot current pose in green.
        cv2.circle(display, (int(self.pose[0]), int(self.pose[1])), 5, (0, 255, 0), -1)
        heading_len = 10
        cv2.line(
            display,
            (int(self.pose[0]), int(self.pose[1])),
            (
                int(self.pose[0] + heading_len * np.cos(self.pose[2])),
                int(self.pose[1] + heading_len * np.sin(self.pose[2])),
            ),
            (0, 255, 0),
            2,
        )

        # Mark known slot drop points in red.
        for slot_id, dest in self.slots.slots.items():
            cv2.circle(display, tuple(dest.drop), 3, (0, 0, 255), -1)

        # Draw the planned route.
        route = self.route_for_qr(qr_value)
        if route and route.get("waypoints"):
            waypoints = route["waypoints"]
            pts = [(int(self.pose[0]), int(self.pose[1]))] + [(int(x), int(y)) for x, y in waypoints]
            for i in range(len(pts) - 1):
                cv2.line(display, pts[i], pts[i + 1], (0, 165, 255), 2)
            cv2.circle(display, pts[-1], 5, (0, 165, 255), -1)

        ok, encoded = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        return encoded.tobytes()

    # ---------------------------------------------------------- controls
    def send_action(self, action):
        action = str(action).strip().lower()
        speed = int(self.settings.get("admin", {}).get("manual_speed", 90))
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
            time.sleep(duration)
            self.esp32.stop()
        finally:
            with self.manual_lock:
                self.manual_busy = False

    # --------------------------------------------------------- status json
    def status(self):
        camera = self.camera.get_snapshot()
        telemetry = self.esp32.get_telemetry()
        qr_value = camera["latest_qr"]
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
            "manual_busy": self.manual_busy,
            "last_command": self.last_command,
            "slots": sorted(self.slots.slots.keys()),
            "logs": self.get_logs(),
        }

    def log(self, message):
        entry = f"[{time.strftime('%H:%M:%S')}] {message}"
        print(entry)
        with self.logs_lock:
            self.logs.append(entry)
            self.logs = self.logs[-120:]

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
        else:
            self.send_error(404, "Not found")

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path != "/api/command":
            self.send_error(404, "Not found")
            return

        params = urllib.parse.parse_qs(parsed.query)
        action = params.get("action", [""])[0]
        try:
            self.state.send_action(action)
            self.send_json({"ok": True})
        except Exception as exc:
            self.state.log(f"Command failed: {exc}")
            self.send_json({"ok": False, "error": str(exc)}, status=400)

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
    header { padding: 14px 18px; background: #0f172a; border-bottom: 1px solid #334155; }
    h1 { margin: 0; font-size: 22px; }
    main { padding: 16px; display: grid; gap: 14px; grid-template-columns: minmax(280px, 1.4fr) minmax(280px, 1fr); }
    section { background: #1f2937; border: 1px solid #374151; border-radius: 8px; padding: 14px; }
    h2 { margin: 0 0 10px; font-size: 16px; }
    img { width: 100%; background: #020617; border-radius: 6px; display: block; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; }
    .card { background: #111827; border: 1px solid #374151; border-radius: 6px; padding: 10px; min-height: 64px; }
    .label { color: #9ca3af; font-size: 12px; margin-bottom: 6px; }
    .value { font-size: 18px; font-weight: 700; overflow-wrap: anywhere; }
    .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; max-width: 360px; }
    button { background: #2563eb; color: white; border: 0; border-radius: 6px; padding: 10px; font-weight: 700; cursor: pointer; }
    button.stop { background: #dc2626; }
    button.secondary { background: #475569; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    pre { background: #020617; color: #d1d5db; border-radius: 6px; padding: 10px; min-height: 180px; max-height: 320px; white-space: pre-wrap; overflow: auto; }
    ul { margin: 0; padding-left: 18px; }
    @media (max-width: 820px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header><h1>Warehouse Rover Admin</h1></header>
  <main>
    <section>
      <h2>Live RPi Camera</h2>
      <img src="/video" alt="Live camera feed">
    </section>
    <section>
      <h2>Warehouse Map + Route</h2>
      <img id="mappng" src="/api/map.jpg" alt="Warehouse map with planned route">
    </section>
    <section>
      <h2>Status</h2>
      <div class="grid">
        <div class="card"><div class="label">ESP32</div><div id="esp32" class="value">-</div></div>
        <div class="card"><div class="label">Camera</div><div id="camera" class="value">-</div></div>
        <div class="card"><div class="label">QR</div><div id="qr" class="value">-</div></div>
        <div class="card"><div class="label">Distance</div><div id="distance" class="value">-</div></div>
        <div class="card"><div class="label">Yaw</div><div id="yaw" class="value">-</div></div>
        <div class="card"><div class="label">Uptime</div><div id="uptime" class="value">-</div></div>
      </div>
    </section>
    <section>
      <h2>Manual Controls</h2>
      <div class="controls">
        <span></span><button onclick="cmd('forward')">Forward</button><span></span>
        <button onclick="cmd('left')">Left</button><button class="stop" onclick="cmd('stop')">Stop</button><button onclick="cmd('right')">Right</button>
        <span></span><button onclick="cmd('backward')">Backward</button><span></span>
        <button class="secondary" onclick="cmd('gripper_open')">Open Gripper</button>
        <span></span>
        <button class="secondary" onclick="cmd('gripper_close')">Close Gripper</button>
      </div>
      <p id="commandResult"></p>
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
      <h2>Logs</h2>
      <pre id="logs"></pre>
    </section>
  </main>
  <script>
    async function refresh() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();
        document.getElementById('esp32').textContent = data.esp32_connected ? 'Connected' : 'Simulation';
        document.getElementById('camera').textContent = data.camera_ok ? 'Live' : 'No frame';
        document.getElementById('qr').textContent = data.qr || '-';
        document.getElementById('distance').textContent = Number(data.telemetry.distance_cm).toFixed(1) + ' cm';
        document.getElementById('yaw').textContent = Number(data.telemetry.yaw_deg).toFixed(1) + ' deg';
        document.getElementById('uptime').textContent = data.uptime_s + ' s';
        document.getElementById('destination').textContent = JSON.stringify({destination: data.destination, route: data.route}, null, 2);
        document.getElementById('logs').textContent = data.logs.join('\n');
        document.getElementById('slots').innerHTML = data.slots.map(s => '<li>' + s + '</li>').join('');
      } catch (e) {
        // ignore transient fetch errors
      }
      // Refresh map preview as well
      document.getElementById('mappng').src = '/api/map.jpg?t=' + Date.now();
    }
    async function cmd(action) {
      const res = await fetch('/api/command?action=' + encodeURIComponent(action), {method: 'POST'});
      const data = await res.json();
      document.getElementById('commandResult').textContent = data.ok ? 'Sent: ' + action : data.error;
      refresh();
    }
    setInterval(refresh, 1000);
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
