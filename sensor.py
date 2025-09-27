# sensor.py

import base64
import io
import math
from PIL import Image
import numpy as np
import cv2

class ObstacleMapper:
    def __init__(self, canvas_w=650, canvas_h=600, obstacle_default_size=25):
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h
        self.obstacle_default_size = obstacle_default_size
        
        self.LOWER_BLACK = np.array([0, 0, 0])
        self.UPPER_BLACK = np.array([70, 70, 70])
        
    def _is_valid_obstacle_center(self, x, y, known_obstacles):
        """Checks if a detected point is too close to a previously known obstacle."""
        for obs in known_obstacles:
            if math.hypot(x - obs['x'], y - obs['y']) < self.obstacle_default_size + 10:
                return False
        return True

    def process_capture(self, image_data: str, known_obstacles: list) -> list:
        """Analyzes the base64 image data to detect obstacles."""
        if not image_data or not image_data.startswith("data:image"):
            return []

        try:
            # 1. Decode Image (Base64 -> PIL -> OpenCV format)
            header, encoded = image_data.split(",", 1)
            image_bytes = base64.b64decode(encoded)
            image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
            img_np = np.array(image)
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            # 2. Thresholding: Isolate black objects
            mask = cv2.inRange(img_bgr, self.LOWER_BLACK, self.UPPER_BLACK)

            # 3. Find Contours: Detect the shapes of the black objects
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            new_obstacles = []
            
            for contour in contours:
                # Filter out noise (very small contours)
                if cv2.contourArea(contour) < 100:
                    continue

                M = cv2.moments(contour)
                if M["m00"] == 0: continue
                
                # Calculate center (x, y) coordinates
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # 4. Filter: Only add if the obstacle is new and valid
                if self._is_valid_obstacle_center(cx, cy, known_obstacles):
                    new_obstacles.append({
                        "x": cx, 
                        "y": cy, 
                        "size": self.obstacle_default_size 
                    })

            return new_obstacles

        except Exception:
            return []