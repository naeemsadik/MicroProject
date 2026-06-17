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

    Resilience:
        The USB-CDC bus on Linux resets the ESP32 every time another
        process opens /dev/ttyACM0 (which is what causes the
        "device reports readiness to read but returned no data"
        / Errno 5 storms when both the admin panel and the
        MissionController open their own ESP32Interface). To cope:

        * A single ``ESP32Interface`` per process should own the port.
        * If a read or write raises an I/O error, the read loop
          backs off briefly, closes the broken handle, and tries
          to reopen the same port once. After a successful reopen
          normal operation resumes. After a failed reopen we drop
          back to simulation mode so the rest of the app keeps
          working instead of throwing on every command.
    """

    def __init__(self, port=None, baudrate=115200):
        self.telemetry = Telemetry(updated_at=time.monotonic())
        self._lock = threading.Lock()
        self.ser = None
        self.simulation_mode = True
        self._last_pong = False
        self._port = port
        self._baudrate = baudrate
        self._reconnect_lock = threading.Lock()
        self._closed = False
        self._reconnect_attempts = 0
        self._max_reconnect_attempts = 3

        if port and str(port).upper() != "NONE":
            if serial is None:
                print("[COMMS] pyserial is not installed. Install requirements.txt for hardware mode.")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
                return

            self._open_serial(initial=True)
        else:
            print("[COMMS] No hardware port provided. Running in SIL Simulation Mock Mode.")

    def _open_serial(self, initial=False):
        """Open (or reopen) the serial port. Sets ``simulation_mode``
        accordingly and starts the read thread on a fresh handle."""
        try:
            # Closing the previous handle first, if any, avoids the
            # "multiple access on port" warning on Linux.
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.ser = serial.Serial(self._port, self._baudrate, timeout=1)
            self.simulation_mode = False
            self._reconnect_attempts = 0
            if initial:
                print(f"[COMMS] Real ESP32 connection established on port: {self._port}")
            else:
                print(f"[COMMS] Reconnected to ESP32 on {self._port}.")
            if not hasattr(self, "read_thread") or self.read_thread is None or not self.read_thread.is_alive():
                self._closed = False
                self.read_thread = threading.Thread(target=self._hardware_read_loop, daemon=True)
                self.read_thread.start()
        except Exception as exc:
            self.ser = None
            self.simulation_mode = True
            if initial:
                print(f"[COMMS] Cannot connect to physical hardware: {exc}")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
            else:
                print(f"[COMMS] Reconnect failed: {exc}")

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
        # One retry: if the first write raises (e.g. Errno 5 because
        # the ESP32 USB-CDC just reset), attempt a single reconnect
        # and retry the write before giving up.
        for attempt in (1, 2):
            if self.simulation_mode or self.ser is None:
                return
            try:
                self.ser.write((message + "\n").encode())
                return
            except Exception as exc:
                print(f"[COMMS] write failed (attempt {attempt}): {exc}")
                if attempt == 1:
                    self._try_reconnect()

    def _try_reconnect(self):
        """Try to recover the serial link after a transient I/O error."""
        with self._reconnect_lock:
            if self._closed or self._reconnect_attempts >= self._max_reconnect_attempts:
                return
            self._reconnect_attempts += 1
            # Brief backoff so the USB-CDC enumeration can settle
            # before we try to reopen the port.
            time.sleep(0.5)
            self._open_serial(initial=False)
            if self.simulation_mode:
                # Give up after exhausting attempts; fall back to mock.
                self._closed = True

    def _hardware_read_loop(self):
        while not self._closed:
            if self.simulation_mode or self.ser is None:
                # No hardware: just sleep and let any reconnect attempt
                # restore the link from elsewhere.
                time.sleep(0.05)
                continue
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    self._handle_line(line)
            except Exception as exc:
                # Don't bail permanently -- the ESP32 USB-CDC port
                # resets whenever /dev/ttyACM0 is reopened by anyone,
                # which yields transient Errno 5 / "no data" errors.
                # Sleep briefly, try to reopen, and keep going.
                print(f"[COMMS] read error: {exc}; attempting reconnect")
                self._try_reconnect()
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
