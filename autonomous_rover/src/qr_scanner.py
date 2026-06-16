import time

import cv2

from warehouse_slots import normalize_slot_id


class QRScanner:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.detector = cv2.QRCodeDetector()

    def scan_frame(self, frame):
        payload, points, _ = self.detector.detectAndDecode(frame)
        if not payload:
            return None
        return normalize_slot_id(payload)

    def scan(self, timeout_s=30):
        camera = cv2.VideoCapture(self.camera_index)
        if not camera.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        deadline = time.monotonic() + timeout_s if timeout_s else None
        try:
            while deadline is None or time.monotonic() < deadline:
                ok, frame = camera.read()
                if not ok or frame is None:
                    time.sleep(0.05)
                    continue

                slot_id = self.scan_frame(frame)
                if slot_id:
                    return slot_id

                time.sleep(0.03)
        finally:
            camera.release()

        raise TimeoutError(f"No QR code detected within {timeout_s} seconds")
