from dataclasses import dataclass
import threading
import time

try:
    import serial
except ImportError:
    serial = None


@dataclass
class Telemetry:
    distance_cm: float = 100.0
    left_ticks: int = 0
    right_ticks: int = 0
    yaw_deg: float = 0.0
    updated_at: float = 0.0


class ESP32Interface:
    """
    Line-based serial link to the ESP32 rover firmware.

    Protocol:

        RPi4 -> ESP32 (newline-terminated):
            <V,left,right>   signed motor PWM, -255..255
            <G,OPEN>         open gripper
            <G,CLOSE>        close gripper
            <PING>           handshake
            <RESET_TICKS>    zero encoder counters and yaw

        ESP32 -> RPi4:
            <T,distance_cm,left_ticks,right_ticks,yaw_deg>
            <PONG>
            <READY>
    """

    def __init__(self, port=None, baudrate=115200):
        self.telemetry = Telemetry(updated_at=time.monotonic())
        self._lock = threading.Lock()
        self.ser = None
        self.simulation_mode = True
        self._last_pong = False

        if port and str(port).upper() != "NONE":
            if serial is None:
                print("[COMMS] pyserial is not installed. Install requirements.txt for hardware mode.")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
                return

            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
                self.simulation_mode = False
                print(f"[COMMS] Real ESP32 connection established on port: {port}")

                self.read_thread = threading.Thread(target=self._hardware_read_loop, daemon=True)
                self.read_thread.start()
            except Exception as exc:
                print(f"[COMMS] Cannot connect to physical hardware: {exc}")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
        else:
            print("[COMMS] No hardware port provided. Running in SIL Simulation Mock Mode.")

    # ----------------------------------------------------------------- send
    def send_velocity_cmd(self, left_speed, right_speed):
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        if not self.simulation_mode:
            self._write_line(f"<V,{left_speed},{right_speed}>")

    # Backwards-compatible alias.
    send_motor_cmd = send_velocity_cmd

    def stop(self):
        self.send_velocity_cmd(0, 0)

    def send_gripper_cmd(self, action):
        action = str(action).strip().upper()
        if action not in ("OPEN", "CLOSE"):
            raise ValueError("Gripper action must be OPEN or CLOSE")
        if not self.simulation_mode:
            self._write_line(f"<G,{action}>")

    def send_servo_cmd(self, angle):
        self.send_gripper_cmd("OPEN" if int(angle) <= 45 else "CLOSE")

    def send_command(self, command):
        """Send an arbitrary newline-terminated command (e.g. ``<PING>``)."""
        cmd = str(command).strip()
        if not cmd:
            return
        if not self.simulation_mode:
            self._write_line(cmd)

    def ping(self, timeout_s=1.0):
        """Send ``<PING>`` and wait up to ``timeout_s`` for ``<PONG>``."""
        if self.simulation_mode:
            return False
        with self._lock:
            self._last_pong = False
        self.send_command("<PING>")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_pong:
                    return True
            time.sleep(0.02)
        return False

    # -------------------------------------------------------------- telemetry
    def get_telemetry(self):
        with self._lock:
            return Telemetry(
                distance_cm=self.telemetry.distance_cm,
                left_ticks=self.telemetry.left_ticks,
                right_ticks=self.telemetry.right_ticks,
                yaw_deg=self.telemetry.yaw_deg,
                updated_at=self.telemetry.updated_at,
            )

    # --------------------------------------------------------------- internals
    def _write_line(self, message):
        try:
            self.ser.write((message + "\n").encode())
        except Exception as exc:
            print(f"[COMMS] write failed: {exc}")

    def _hardware_read_loop(self):
        while self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    self._handle_line(line)
            except Exception as exc:
                print(f"[COMMS] read error: {exc}")
                break
            time.sleep(0.01)

    def _handle_line(self, line):
        if not line:
            return
        if line.startswith("<T,") and line.endswith(">"):
            parts = line[3:-1].split(",")
            if len(parts) < 4:
                return
            try:
                telemetry = Telemetry(
                    distance_cm=float(parts[0]),
                    left_ticks=int(parts[1]),
                    right_ticks=int(parts[2]),
                    yaw_deg=float(parts[3]),
                    updated_at=time.monotonic(),
                )
            except ValueError:
                return
            with self._lock:
                self.telemetry = telemetry
        elif line == "<PONG>":
            with self._lock:
                self._last_pong = True
        # <READY> and other messages are simply ignored; the read loop
        # just consumes them so the buffer does not fill up.
