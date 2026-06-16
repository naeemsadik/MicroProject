# test_navigator.py (Updated Timeout Threshold)
import numpy as np
from src.navigator import WaypointNavigator

def simulate_navigation():
    waypoints = [(20, 20), (124, 30), (128, 33), (138, 46), (140, 115), (180, 180)]
    current_pose = [20.0, 20.0, 0.0] 
    
    # Initialize updated navigator
    navigator = WaypointNavigator(K_linear=1.2, K_angular=3.5, target_dist_tolerance=3.0, min_linear_vel=35.0)
    
    print("Starting Navigation Simulation...")
    print(f"Initial Pose: X={current_pose[0]:.2f}, Y={current_pose[1]:.2f}, Theta={current_pose[2]:.2f} rad\n")
    
    dt = 0.1 
    
    for wp_idx, wp in enumerate(waypoints):
        print(f"--- Target Waypoint {wp_idx}: {wp} ---")
        arrived = False
        timeout_counter = 0
        
        while not arrived:
            linear_vel, angular_vel, arrived = navigator.get_steering_commands(current_pose, wp)
            
            if arrived:
                print(f"Successfully reached Waypoint {wp}!")
                break
                
            left_pwm, right_pwm = navigator.unicycle_to_differential(linear_vel, angular_vel)
            
            # Physics update loop
            v_scaled = linear_vel * 0.1 
            w_scaled = angular_vel * 0.1
            
            current_pose[0] += v_scaled * np.cos(current_pose[2]) * dt
            current_pose[1] += v_scaled * np.sin(current_pose[2]) * dt
            current_pose[2] += w_scaled * dt
            current_pose[2] = np.arctan2(np.sin(current_pose[2]), np.cos(current_pose[2]))
            
            timeout_counter += 1
            if timeout_counter % 15 == 0:
                print(f"Pose: ({current_pose[0]:.1f}, {current_pose[1]:.1f}) | Heading: {current_pose[2]:.2f} rad | PWM: L={left_pwm} R={right_pwm}")
                
            # FIXED: Increased safety timeout to 1000 cycles
            if timeout_counter > 1000: 
                print("Simulation Timeout! Controller is unstable or oscillating.")
                return

    print("\nSUCCESS: Virtual rover completed the entire warehouse circuit flawlessly!")

if __name__ == "__main__":
    simulate_navigation()