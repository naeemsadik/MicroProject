import re
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class SlotDestination:
    slot_id: str
    drop: tuple[int, int]
    approach: tuple[int, int] | None = None

    @property
    def navigation_target(self):
        return self.approach or self.drop


class WarehouseSlots:
    def __init__(self, slots, home=(20, 20)):
        self.slots = slots
        self.home = tuple(home)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}

        raw_slots = data.get("slots", {})
        slots = {}
        for raw_id, raw_value in raw_slots.items():
            slot_id = normalize_slot_id(raw_id)
            if isinstance(raw_value, dict):
                drop = _as_point(raw_value["drop"])
                approach = _as_point(raw_value["approach"]) if raw_value.get("approach") else None
            else:
                drop = _as_point(raw_value)
                approach = None
            slots[slot_id] = SlotDestination(slot_id=slot_id, drop=drop, approach=approach)

        return cls(slots=slots, home=_as_point(data.get("home", (20, 20))))

    def get_destination(self, slot_id):
        normalized = normalize_slot_id(slot_id)
        if normalized not in self.slots:
            known = ", ".join(sorted(self.slots)) or "none configured"
            raise KeyError(f"Unknown warehouse slot '{slot_id}'. Known slots: {known}")
        return self.slots[normalized]


def normalize_slot_id(value):
    text = str(value).strip().upper().replace("-", "").replace("_", "")
    match = re.fullmatch(r"R0*(\d+)C0*(\d+)", text)
    if not match:
        raise ValueError(f"QR payload must look like R1C3 or R02C05, got '{value}'")
    return f"R{int(match.group(1))}C{int(match.group(2))}"


def _as_point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"Expected [x, y] coordinate, got {value!r}")
    return int(value[0]), int(value[1])
