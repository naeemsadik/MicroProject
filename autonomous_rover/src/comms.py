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
    def __init__(self, port=None, baudrate=115200):
        self.ultrasonic_distance = 100.0
        self.telemetry = Telemetry(updated_at=time.monotonic())
        self._lock = threading.Lock()
        self.ser = None
        self.simulation_mode = True

        if port and port.upper() != 'NONE':
            if serial is None:
                print("[COMMS ERROR] pyserial is not installed. Install requirements.txt for hardware mode.")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
                return

            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
                self.simulation_mode = False
                print(f"[COMMS] Real ESP32 connection established on port: {port}")

                self.read_thread = threading.Thread(target=self._hardware_read_loop, daemon=True)
                self.read_thread.start()
            except Exception as e:
                print(f"[COMMS ERROR] Cannot connect to physical hardware: {e}")
                print("[COMMS] Falling back to SIL Simulation Mock Mode.")
        else:
            print("[COMMS] No hardware port provided. Running in SIL Simulation Mock Mode.")

    def send_velocity_cmd(self, left_speed, right_speed):
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        if not self.simulation_mode:
            msg = f"<V,{left_speed},{right_speed}>\n"
            self.ser.write(msg.encode())

    def send_motor_cmd(self, left_speed, right_speed):
        self.send_velocity_cmd(left_speed, right_speed)

    def stop(self):
        self.send_velocity_cmd(0, 0)

    def send_gripper_cmd(self, action):
        action = str(action).strip().upper()
        if action not in ("OPEN", "CLOSE"):
            raise ValueError("Gripper action must be OPEN or CLOSE")
        if not self.simulation_mode:
            msg = f"<G,{action}>\n"
            self.ser.write(msg.encode())

    def send_servo_cmd(self, angle):
        self.send_gripper_cmd("OPEN" if int(angle) <= 45 else "CLOSE")

    def get_telemetry(self):
        with self._lock:
            return Telemetry(
                distance_cm=self.telemetry.distance_cm,
                left_ticks=self.telemetry.left_ticks,
                right_ticks=self.telemetry.right_ticks,
                yaw_deg=self.telemetry.yaw_deg,
                updated_at=self.telemetry.updated_at,
            )

    def _hardware_read_loop(self):
        while self.ser and self.ser.is_open:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self._handle_line(line)
            time.sleep(0.01)

    def _handle_line(self, line):
        if not line.startswith("<T,") or not line.endswith(">"):
            return

        parts = line[3:-1].split(',')
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
            self.ultrasonic_distance = telemetry.distance_cm
