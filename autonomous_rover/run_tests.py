"""
Unit tests for the warehouse rover Python stack.

Run with:
    cd autonomous_rover
    python3 run_tests.py
"""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.map_processor import MapProcessor
from src.planner import AStarPlanner
from src.navigator import WaypointNavigator
from src.odometry import DifferentialOdometry
from src.warehouse_slots import WarehouseSlots, normalize_slot_id
from src.comms import ESP32Interface, Telemetry


PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


class TestWarehouseSlots(unittest.TestCase):
    def test_normalize(self):
        self.assertEqual(normalize_slot_id("R1C3"), "R1C3")
        self.assertEqual(normalize_slot_id("r1c3"), "R1C3")
        self.assertEqual(normalize_slot_id("R01C03"), "R1C3")
        self.assertEqual(normalize_slot_id("r-1_c-3"), "R1C3")
        with self.assertRaises(ValueError):
            normalize_slot_id("invalid")

    def test_load(self):
        path = os.path.join(PROJECT_DIR, "config/warehouse_slots.yaml")
        slots = WarehouseSlots.load(path)
        self.assertIn("R1C1", slots.slots)
        self.assertEqual(slots.get_destination("R1C1").drop, (190, 50))


class TestPlanner(unittest.TestCase):
    def setUp(self):
        self.grid = np.load(os.path.join(PROJECT_DIR, "maps/occupancy_grid.npy"))
        self.planner = AStarPlanner(self.grid)

    def test_path_home_to_r2c1(self):
        path = self.planner.plan_path((20, 20), (90, 50))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (20, 20))
        self.assertEqual(path[-1], (90, 50))

    def test_prune_reduces(self):
        path = self.planner.plan_path((20, 20), (90, 130))
        self.assertIsNotNone(path)
        pruned = self.planner.prune_path(path)
        self.assertLessEqual(len(pruned), len(path))

    def test_no_path_through_wall(self):
        # Goal inside an obstacle cell: planner should return None.
        # The shelves occupy x=40..60, so (50, 100) is an obstacle.
        result = self.planner.plan_path((20, 20), (50, 100))
        self.assertIsNone(result)


class TestNavigator(unittest.TestCase):
    def test_arrives_at_target(self):
        nav = WaypointNavigator(target_dist_tolerance=1.0)
        v, w, arrived = nav.get_steering_commands((0, 0, 0), (0, 0))
        self.assertTrue(arrived)

    def test_steers_toward_target(self):
        nav = WaypointNavigator()
        v, w, arrived = nav.get_steering_commands((0, 0, 0), (100, 0))
        self.assertFalse(arrived)
        self.assertGreater(v, 0)
        self.assertAlmostEqual(w, 0, places=1)

    def test_turns_to_face(self):
        nav = WaypointNavigator()
        # Facing east, target north -> should turn left (positive angular vel)
        v, w, _ = nav.get_steering_commands((0, 0, 0), (0, 100))
        self.assertGreater(w, 0)

    def test_unicycle_to_differential(self):
        nav = WaypointNavigator()
        l, r = nav.unicycle_to_differential(100, 0)
        self.assertEqual(l, 100)
        self.assertEqual(r, 100)
        l, r = nav.unicycle_to_differential(0, 50)
        self.assertEqual(l, -50)
        self.assertEqual(r, 50)


class TestOdometry(unittest.TestCase):
    def test_reset(self):
        o = DifferentialOdometry()
        o.reset((10, 20, 0))
        self.assertEqual(o.pose, [10.0, 20.0, 0.0])

    def test_update_with_yaw(self):
        o = DifferentialOdometry()
        o.reset((0, 0, 0))
        o.update(10, 10, yaw_deg=0)
        self.assertGreater(o.pose[0], 0)

    def test_update_no_yaw(self):
        o = DifferentialOdometry()
        o.reset((0, 0, 0))
        o.update(10, 10)
        self.assertGreater(o.pose[0], 0)


class TestComms(unittest.TestCase):
    def test_simulation_mode(self):
        e = ESP32Interface(port=None)
        self.assertTrue(e.simulation_mode)
        # These should not raise even in sim mode
        e.send_velocity_cmd(100, 100)
        e.stop()
        e.send_gripper_cmd("OPEN")
        e.send_gripper_cmd("CLOSE")

    def test_invalid_gripper(self):
        e = ESP32Interface(port=None)
        with self.assertRaises(ValueError):
            e.send_gripper_cmd("WIGGLE")

    def test_clip_speed(self):
        e = ESP32Interface(port=None)
        # Cannot easily verify what was sent, but should not raise
        e.send_velocity_cmd(999, -999)


class TestMissionDryRun(unittest.TestCase):
    def test_dry_run(self):
        from src.mission_controller import MissionController
        c = MissionController(qr_override="R1C1", dry_run=True)
        c.run_once()


def main():
    unittest.main(verbosity=2)


if __name__ == "__main__":
    main()
