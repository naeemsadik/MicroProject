# src/map_processor.py
import cv2
import numpy as np

class MapProcessor:
    def __init__(self, resolution_cm_per_px=2.0, robot_radius_cm=18.0):
        self.resolution = resolution_cm_per_px
        self.robot_radius_px = int(np.ceil(robot_radius_cm / resolution_cm_per_px))

    def generate_occupancy_grid(self, image_path, output_path):
        """Processes high-detail images and blueprints into clean navigation grids."""
        # 1. Load image in grayscale
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Could not load image at {image_path}")

        # 2. Smooth minor textures out
        blurred = cv2.GaussianBlur(img, (5, 5), 0)

        # 3. SMART Thresholding (Otsu's Method)
        # Instead of guessing '220', Otsu automatically finds the perfect contrast split 
        # between the textured background and the dark ink.
        _, binary_map = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # 4. TEXT REMOVAL FILTER (Connected Components analysis)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, connectivity=8)
        clean_binary = np.zeros_like(binary_map)
        
        # Minimum pixel area to keep an item (ignores tiny text noise)
        min_obstacle_area_px = 40  

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_obstacle_area_px:
                clean_binary[labels == i] = 255

        # 5. Inflate remaining structural obstacles using a scaled safety buffer
        kernel_size = max(3, (2 * self.robot_radius_px) + 1)
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel_inflate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated_map = cv2.dilate(clean_binary, kernel_inflate, iterations=1)

        # 6. Convert back to standard orientation format (0 = free space, 1 = obstacle)
        occupancy_grid = (inflated_map > 0).astype(np.uint8)

        # Save map matrix
        np.save(output_path, occupancy_grid)
        return occupancy_grid
