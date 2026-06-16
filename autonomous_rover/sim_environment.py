# sim_environment.py (Updated: Larger, complex textured map)
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.map_processor import MapProcessor
from src.planner import AStarPlanner
from src.navigator import WaypointNavigator
from src.virtual_hardware import VirtualESP32, VirtualCameraLocalizer

# Temporary function to generate a COMPLEX textured map if none exists
def generate_complex_test_photo(image_path):
    print("[SIMULATION] Creating complex textured mock warehouse photo...")
    
    # 1. Create base 'floor' with realistic texture (Min:160, Max:220 grey with noise)
    w, h = 1000, 1000 # 1000x1000 image = 10m x 10m arena (1cm/px)
    noise_speckles = (np.random.randint(160, 220, (h, w))).astype(np.uint8)
    floor_photo = cv2.GaussianBlur(noise_speckles, (13,13), 0) # Smooth noise into concrete texture

    # 2. Add an uneven lighting gradient (Simulates shadows across the map)
    light_gradient = np.linspace(240, 100, h)
    floor_photo = (floor_photo * (light_gradient[:, np.newaxis] / 255.0)).astype(np.uint8)

    # 3. Draw COMPLEX Black Obstacle Shelves (cv2 uses pixel units here)
    # CV2 Rectangles use (x, y) coordinates for points
    color = (0, 0, 0)
    cv2.rectangle(floor_photo, (150, 150), (250, 850), color, -1) # Long shelf 1
    cv2.rectangle(floor_photo, (400, 150), (500, 450), color, -1) # Segmented shelf 2
    cv2.rectangle(floor_photo, (400, 550), (500, 850), color, -1) # Segmented shelf 2b
    cv2.rectangle(floor_photo, (650, 300), (850, 700), color, -1) # Large block shelf 3
    cv2.rectangle(floor_photo, (700, 150), (900, 250), color, -1) # Horizon shelf 4
    
    # Simulate a messy dynamic obstacle (pile of boxes)
    cv2.circle(floor_photo, (150, 50), 25, color, -1)

    # Add floor boundary
    cv2.rectangle(floor_photo, (0, 0), (999, 999), color, 10)

    # Save textured "photo" to disk
    os.makedirs(os.path.dirname(image_path), exist_ok=True)
    cv2.imwrite(image_path, floor_photo)
    print(f"[SIMULATION] Complex photo successfully saved at: {image_path}")

class WarehouseSimulationWorkspace:
    def __init__(self):
        print("\n" + "="*50)
        print("[SIMULATION] Booting Virtual Warehouse SIL Twin...")
        print("="*50)
        
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_path = os.path.join(self.base_dir, 'maps', 'complex_floorplan.png')
        self.grid_path = os.path.join(self.base_dir, 'maps', 'complex_grid.npy')
        
        # Determine scale resolution (1000px = 1000cm = 10meters)
        self.res_cm_per_px = 1.0 
        
        # 1. GENERATE or LOAD Map Matrix
        if not os.path.exists(self.image_path):
            generate_complex_test_photo(self.image_path)
            
        print(f"[INIT] Processing Map from photo: {self.image_path}...")
        
        # Instantiate NEW Adaptive Processor
        # Increase safety radius because we now have shadows/noise
        self.processor = MapProcessor(resolution_cm_per_px=self.res_cm_per_px, robot_radius_cm=18.0)
        
        # Generate the safe navigable grid using adaptive logic
        self.grid = self.processor.generate_occupancy_grid(self.image_path, self.grid_path)
        print(f"[INIT] Navigable Grid Matrix Generated ({self.grid.shape[1]}x{self.grid.shape[0]} px)")

        # 2. Setup High-Fidelity Modules
        self.planner = AStarPlanner(self.grid)
        self.navigator = WaypointNavigator(K_linear=1.2, K_angular=3.5, target_dist_tolerance=4.0, min_linear_vel=45.0)
        
        # 3. Instantiate Virtual Twin Hardware
        # Start in the clear corner, aim for far corner inside a narrow corridor
        self.start_pose = (50.0, 50.0, 0.0)
        self.target_node = (950, 950)
        
        self.v_esp = VirtualESP32()
        self.v_cam = VirtualCameraLocalizer(self.start_pose)

        # 4. Generate Pruned Waypoint List
        print(f"[PLAN] Generating path from {self.start_pose[:2]} to {self.target_node}...")
        start_snap = (int(self.start_pose[0]), int(self.start_pose[1]))
        dense_path = self.planner.plan_path(start_snap, self.target_node)
        self.waypoints = self.planner.prune_path(dense_path)
        
        print(f"[PLAN] Route finalized through {len(self.waypoints)} optimized milestones.")
        if self.waypoints:
            self.waypoints.pop(0) # Remove starting vertex

        # 5. Initialize Trajectory History
        self.history_x = [self.start_pose[0]]
        self.history_y = [self.start_pose[1]]

        # 6. Build Plot GUI Viewports
        # Display the real INFLATED GRID, not the raw textured photo
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.imshow(self.grid, cmap='gray_r')
        
        # Draw target waypoints list (green dashes)
        if len(self.waypoints) > 0:
            w_x, w_y = zip(*[(50,50)] + self.waypoints)
            self.ax.plot(w_x, w_y, 'g--', alpha=0.6, label="Planned Geometric Vectors")

        self.robot_marker, = self.ax.plot([], [], 'ro', markersize=14, markerfacecolor='red', label="Rover Chassis")
        self.heading_line, = self.ax.plot([], [], 'r-', linewidth=3)
        self.trail_line, = self.ax.plot([], [], 'b-', alpha=0.7, linewidth=1.5, label="Physical Trajectory History")
        
        self.ax.legend(loc='upper left')
        self.ax.set_title(f"Warehouse AGV Simulation (Matrix: {self.grid.shape[1]}x{self.grid.shape[0]})")
        plt.tight_layout()

    # ... [Keep init_plot and update_frame methods exactly the same as in previous Virtualスタッフ Sandbox sandbox turn] ...
    def init_plot(self):
        self.robot_marker.set_data([], [])
        self.heading_line.set_data([], [])
        self.trail_line.set_data([], [])
        return self.robot_marker, self.heading_line, self.trail_line

    def update_frame(self, frame_num):
        # 1. Read location from virtual camera localizer
        x, y, theta = self.v_cam.get_pose()
        
        if not self.waypoints:
            if self.v_esp.left_pwm != 0 or self.v_esp.right_pwm != 0:
                print("[SIMULATION] Success: Destination coordinates finalized. Platform Parked.")
            self.v_esp.send_motor_cmd(0, 0)
            return self.robot_marker, self.heading_line, self.trail_line

        # 2. Feed navigation brain to calculate steering corrections
        current_target = self.waypoints[0]
        linear_v, angular_w, arrived = self.navigator.get_steering_commands((x, y, theta), current_target)

        if arrived:
            print(f"[SIMULATION] Cleared Milestone Waypoint intersection: {current_target}")
            self.waypoints.pop(0)
            return self.robot_marker, self.heading_line, self.trail_line

        # 3. Transmit instructions down serial channel to virtual motor driver board
        left_pwm, right_pwm = self.navigator.unicycle_to_differential(linear_v, angular_w)
        self.v_esp.send_motor_cmd(left_pwm, right_pwm)

        # 4. Physics Step Forward (Scaling forces because the map is larger)
        self.v_cam.update_physics(self.v_esp.left_pwm, self.v_esp.right_pwm, dt=0.03)

        # 5. Trail Tracking
        self.history_x.append(x)
        self.history_y.append(y)

        # 6. Update visualizer
        self.robot_marker.set_data([x], [y])
        self.trail_line.set_data(self.history_x, self.history_y)
        arrow_len = 25 # Scale arrow for larger map
        self.heading_line.set_data([x, x + arrow_len * np.cos(theta)], [y, y + arrow_len * np.sin(theta)])
        return self.robot_marker, self.heading_line, self.trail_line

    def run(self):
        # Increased interval to 40ms to stabilize the larger animation
        self.anim = FuncAnimation(self.fig, self.update_frame, init_func=self.init_plot, blit=True, interval=40, cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    sim = WarehouseSimulationWorkspace()
    sim.run()
