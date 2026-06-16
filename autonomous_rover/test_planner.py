# test_planner.py (Updated for robust path handling)
import os
import numpy as np
import matplotlib.pyplot as plt
from src.map_processor import MapProcessor
from src.planner import AStarPlanner

# Get the absolute directory where this test_planner.py file lives
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Construct absolute paths to your maps
IMAGE_PATH = os.path.join(BASE_DIR, 'maps', 'floorplan.png')
GRID_PATH = os.path.join(BASE_DIR, 'maps', 'occupancy_grid.npy')

# 1. Process the image
processor = MapProcessor(resolution_cm_per_px=1.0, robot_radius_cm=10.0)
grid = processor.generate_occupancy_grid(IMAGE_PATH, GRID_PATH)

# 2. Plan a path using image coordinates: x grows right, y grows downward.
start_pose = (20, 20)
goal_pose = (180, 180)

planner = AStarPlanner(grid)
dense_path = planner.plan_path(start_pose, goal_pose)

# --- NEW: Prune the dense path down to nodes ---
if dense_path:
    path = planner.prune_path(dense_path)
    print(f"Compressed path from {len(dense_path)} coordinates down to {len(path)} turning nodes!")
    print("Waypoint Nodes:", path)
else:
    path = None

# 3. Visualize the output
plt.figure(figsize=(10, 10))
# cmap='gray' sets 0 to black and 1 to white. 
# Since we inverted it in map_processor, we display it properly here.
plt.imshow(grid, cmap='gray_r')

if path:
    path_x, path_y = zip(*path)
    plt.plot(path_x, path_y, color='red', linewidth=3, label="A* Planned Path")
    print(f"Path successfully generated with {len(path)} waypoints!")
else:
    print("Failed to find a path. Check if start or goal coordinates are inside an obstacle.")

plt.scatter(*start_pose, color='green', s=100, label="Start")
plt.scatter(*goal_pose, color='blue', s=100, label="Goal")
plt.legend()
plt.title("Warehouse Rover Path Planning Verification")
plt.show()
