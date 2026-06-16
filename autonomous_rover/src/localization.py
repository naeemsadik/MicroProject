import cv2
import numpy as np

class ArucoLocalizer:
    def __init__(self, aruco_dict_type=cv2.aruco.DICT_4X4_50):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Mapping marker IDs to map grid coordinates (X, Y)
        # In production, load this from config/aruco_map.json
        self.marker_map = {
            1: (10, 10), 
            2: (50, 10),
            3: (10, 50)
        }

    def get_pose(self, frame):
        """Detects ArUco markers and returns estimated map pose."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            # For simplicity, use the first detected marker to localize
            marker_id = ids[0][0]
            if marker_id in self.marker_map:
                map_x, map_y = self.marker_map[marker_id]
                
                # Calculate orientation (theta) from marker corners
                c = corners[0][0]
                vector = c[1] - c[0] # Vector from top-left to top-right corner
                theta = np.arctan2(vector[1], vector[0])
                
                return map_x, map_y, theta
        return None, None, None