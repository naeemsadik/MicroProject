# create_mock_map.py
import os
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(BASE_DIR, "maps")

# Create maps directory if it doesn't exist
os.makedirs(MAPS_DIR, exist_ok=True)

# Create a 200x200 white image (representing empty warehouse floor)
# 255 = White (Navigable), 0 = Black (Obstacles)
mock_map = np.ones((200, 200), dtype=np.uint8) * 255

# Draw some black rectangles to simulate warehouse shelving rows
# cv2.rectangle(img, pt1, pt2, color, thickness)
cv2.rectangle(mock_map, (40, 40), (60, 160), 0, -1)   # Shelf Row 1
cv2.rectangle(mock_map, (100, 40), (120, 160), 0, -1) # Shelf Row 2
cv2.rectangle(mock_map, (150, 40), (170, 110), 0, -1) # Shelf Row 3

# Draw outer walls (border)
cv2.rectangle(mock_map, (0, 0), (199, 199), 0, 5)

# Save the map
output_path = os.path.join(MAPS_DIR, "floorplan.png")
cv2.imwrite(output_path, mock_map)
print(f"Mock warehouse map successfully created at {output_path}!")
