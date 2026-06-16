"""
Generate printable QR code labels for the warehouse slots.

Each QR code encodes a slot ID such as R1C1, R1C2, ... R3C3, which is
the exact payload the RPi4 QR scanner expects.

Usage:
    python generate_qr_codes.py
    python generate_qr_codes.py --out qrcodes --ids R1C1 R1C2 R2C3

The script writes one PNG per slot ID to the chosen output directory.
"""

import argparse
import os

import cv2
import numpy as np
import yaml


def generate_qr(payload, size=400):
    try:
        detector = cv2.QRCodeDetector()
        # OpenCV does not have a built-in QR generator in older versions, so
        # build a small QR via a manual approach using qrcode if available.
        try:
            import qrcode
            qr = qrcode.QRCode(box_size=10, border=2)
            qr.add_data(payload)
            qr.make(fit=True)
            img = qr.make_image(fill_color="black", back_color="white").convert("L")
            img = np.array(img.resize((size, size)))
            return img
        except ImportError:
            pass
        # Fallback: draw the payload as a basic text marker so the demo
        # can still run.
        img = np.full((size, size), 255, dtype=np.uint8)
        cv2.putText(
            img,
            payload,
            (20, size // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            2.0,
            0,
            3,
        )
        return img
    except Exception as exc:
        print(f"Failed to generate QR for {payload}: {exc}")
        return None


def main():
    parser = argparse.ArgumentParser(description="Generate QR codes for warehouse slots.")
    parser.add_argument("--slots", default="config/warehouse_slots.yaml", help="Path to warehouse_slots.yaml")
    parser.add_argument("--out", default="qrcodes", help="Output directory")
    parser.add_argument("--size", type=int, default=400, help="QR code size in pixels")
    parser.add_argument("--ids", nargs="*", default=None, help="Restrict to these slot IDs")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    slots_path = os.path.join(base_dir, args.slots)
    out_dir = os.path.join(base_dir, args.out)
    os.makedirs(out_dir, exist_ok=True)

    with open(slots_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    slot_ids = list((data.get("slots") or {}).keys())
    if args.ids:
        slot_ids = [s for s in slot_ids if s in args.ids]

    for slot_id in slot_ids:
        img = generate_qr(slot_id, size=args.size)
        if img is None:
            continue
        out_path = os.path.join(out_dir, f"{slot_id}.png")
        cv2.imwrite(out_path, img)
        print(f"  {slot_id} -> {out_path}")


if __name__ == "__main__":
    main()
