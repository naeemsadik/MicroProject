# run_custom_warehouse.py
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from src.map_processor import MapProcessor
from src.planner import AStarPlanner

def main():
    print("\n" + "="*50)
    print("      CUSTOM PHOTO-TO-ROVER PIPELINE GENERATOR      ")
    print("="*50)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. DEFINE YOUR CUSTOM IMAGE PATH HERE
    # Change 'my_real_warehouse.png' to match whatever image you put in the folder!
    custom_image_name = 'custom_warehouse1.jpg' 
    image_path = os.path.join(base_dir, 'maps', custom_image_name)
    grid_output_path = os.path.join(base_dir, 'maps', 'custom_occupancy_grid.npy')

    if not os.path.exists(image_path):
        print(f"[ERROR] Could not find your image at: {image_path}")
        print("Please place your photo inside the 'maps' folder and update the script filename.")
        return

    # 2. LOAD & PRINT IMAGE SPECS
    raw_img = cv2.imread(image_path)
    h, w, _ = raw_img.shape
    print(f"[INFO] Successfully loaded '{custom_image_name}'")
    print(f"[INFO] Image Dimensions: {w}x{h} pixels.")

    # 3. PROCESS THE PHOTO INTO A SAFE NAVIGABLE LAYOUT
    # resolution: how many centimeters does 1 pixel represent in your real space?
    # robot_radius_cm: safety buffer zone around obstacles to protect the chassis
    processor = MapProcessor(resolution_cm_per_px=2.0, robot_radius_cm=18.0)
    print("[PROCESSING] Running adaptive filtering, noise removal, and safety inflation...")
    grid = processor.generate_occupancy_grid(image_path, grid_output_path)
    print("[SUCCESS] Navigable binary matrix generated and saved.")

   # 4. CHOOSE VALID ISLE COORDINATES
    print(f"\n[CONFIG] Image limits are X: 0-{w}, Y: 0-{h}")
    
    # Coordinates use image convention: x grows right, y grows downward.
    start_node = (100, 125)   
    
    # Place Target right in the wide central corridor intersection
    target_node = (530, 300)  
    
    print(f"[CONFIG] Selected Start Coordinate: {start_node}")
    print(f"[CONFIG] Selected Target Coordinate: {target_node}")

    # 5. PLAN THE PATH
    planner = AStarPlanner(grid)
    print("[PLANNING] Calculating optimal path through your custom warehouse layout...")
    
    dense_path = planner.plan_path(start_node, target_node)
    
    # 6. VISUALIZE VERIFICATION (Moved up so we can always see where points land)
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.imshow(grid, cmap='gray_r')
    ax.plot(start_node[0], start_node[1], 'go', markersize=12, label='Rover Start')
    ax.plot(target_node[0], target_node[1], 'bo', markersize=12, label='Target Destination')

    if dense_path is not None:
        pruned_waypoints = planner.prune_path(dense_path)
        print(f"[SUCCESS] Found route! Condensed path down to {len(pruned_waypoints)} key milestones.")
        
        print("\n" + "-"*30)
        print("FEED READY FOR ROVER MOTOR CONTROLLER:")
        print("-"*30)
        for idx, wp in enumerate(pruned_waypoints):
            print(f" Milestone {idx+1}: Target Pixel Coordinates -> X: {wp[0]}, Y: {wp[1]}")
        print("-"*30)

        if pruned_waypoints:
            w_x, w_y = zip(*pruned_waypoints)
            ax.plot(w_x, w_y, 'r-o', linewidth=2, label='Fed Navigation Path')
    else:
        print("\n[PLANNING ERROR] A* could not find a path between those points.")
        print("The map window will open anyway. Look at where the green (Start) and blue (Goal) dots are.")
        print("If they are touching a black pixel or inside a tight shelf cluster, adjust the coordinates in the script!")

    ax.legend()
    ax.set_title(f"Custom Layout Navigation Map ({custom_image_name})")
    plt.show()

if __name__ == "__main__":
    main()
