# sensor.py - IMPROVED VERSION with better filtering

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
        
        # IMPROVED: More restrictive color thresholds for better obstacle detection
        self.LOWER_BLACK = np.array([0, 0, 0])
        self.UPPER_BLACK = np.array([50, 50, 50])  # More restrictive threshold
        
        # Filtering parameters
        self.MIN_CONTOUR_AREA = 200  # Increased minimum area
        self.MAX_CONTOUR_AREA = 2000  # Maximum area to avoid detecting large background elements
        self.MIN_OBSTACLE_SEPARATION = 35  # Minimum distance between obstacles
        
    def _is_valid_obstacle_center(self, x, y, known_obstacles):
        """Checks if a detected point is too close to a previously known obstacle."""
        for obs in known_obstacles:
            if math.hypot(x - obs['x'], y - obs['y']) < self.MIN_OBSTACLE_SEPARATION:
                return False
        return True
    
    def _is_obstacle_shape_valid(self, contour):
        """Validate that the contour represents a likely obstacle"""
        # Check area
        area = cv2.contourArea(contour)
        if area < self.MIN_CONTOUR_AREA or area > self.MAX_CONTOUR_AREA:
            return False
        
        # Check aspect ratio (should be roughly square-ish)
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        if aspect_ratio < 0.5 or aspect_ratio > 2.0:  # Too elongated
            return False
            
        # Check if contour is roughly compact (not too irregular)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < 0.3:  # Too irregular shape
            return False
            
        return True
    
    def _filter_edge_obstacles(self, x, y, margin=30):
        """Filter out obstacles too close to canvas edges (likely UI elements)"""
        if (x < margin or x > self.canvas_w - margin or 
            y < margin or y > self.canvas_h - margin):
            return False
        return True
    
    def _is_robot_area(self, x, y, robot_radius=25):
        """Check if detected obstacle is actually the robot (around center)"""
        center_x, center_y = self.canvas_w // 2, self.canvas_h // 2
        distance_to_center = math.hypot(x - center_x, y - center_y)
        return distance_to_center < robot_radius * 2  # Don't detect robot as obstacle

    def process_capture(self, image_data: str, known_obstacles: list) -> list:
        """Analyzes the base64 image data to detect obstacles with improved filtering."""
        if not image_data or not image_data.startswith("data:image"):
            print("❌ Invalid image data")
            return []

        try:
            # 1. Decode Image (Base64 -> PIL -> OpenCV format)
            header, encoded = image_data.split(",", 1)
            image_bytes = base64.b64decode(encoded)
            image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
            img_np = np.array(image)
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            # 2. IMPROVED: Better preprocessing
            # Convert to grayscale first for better thresholding
            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            
            # Apply slight blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Use adaptive thresholding for better obstacle detection
            # This helps with varying lighting conditions
            _, binary = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)
            
            # Morphological operations to clean up the image
            kernel = np.ones((2, 2), np.uint8)
            cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

            # 3. Find Contours: Detect the shapes of the black objects
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            new_obstacles = []
            detected_positions = []  # Track all detected positions for debugging
            
            print(f"🔍 Found {len(contours)} potential obstacles")
            
            for i, contour in enumerate(contours):
                # IMPROVED: Multiple validation layers
                
                # 1. Check if contour represents a valid obstacle shape
                if not self._is_obstacle_shape_valid(contour):
                    continue

                # 2. Get center coordinates
                M = cv2.moments(contour)
                if M["m00"] == 0: 
                    continue
                
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                detected_positions.append((cx, cy))
                
                # 3. Filter edge obstacles (likely UI elements)
                if not self._filter_edge_obstacles(cx, cy):
                    print(f"  ❌ Filtered edge obstacle at ({cx}, {cy})")
                    continue
                
                # 4. Filter robot area
                if self._is_robot_area(cx, cy):
                    print(f"  ❌ Filtered robot area at ({cx}, {cy})")
                    continue
                
                # 5. Check against known obstacles
                if not self._is_valid_obstacle_center(cx, cy, known_obstacles):
                    print(f"  ❌ Too close to known obstacle at ({cx}, {cy})")
                    continue
                
                # 6. Check against other new obstacles in this detection
                is_duplicate = False
                for new_obs in new_obstacles:
                    if math.hypot(cx - new_obs['x'], cy - new_obs['y']) < self.MIN_OBSTACLE_SEPARATION:
                        is_duplicate = True
                        break
                
                if is_duplicate:
                    print(f"  ❌ Duplicate obstacle at ({cx}, {cy})")
                    continue
                
                # 7. All checks passed - add as valid obstacle
                area = cv2.contourArea(contour)
                new_obstacles.append({
                    "x": cx, 
                    "y": cy, 
                    "size": min(self.obstacle_default_size, max(20, int(math.sqrt(area / math.pi) * 2)))
                })
                print(f"  ✅ Valid obstacle at ({cx}, {cy}) with area {area}")

            print(f"🎯 Total detected: {len(detected_positions)}, Valid obstacles: {len(new_obstacles)}")
            
            # Additional sanity check - if we detect too many obstacles, something is wrong
            if len(new_obstacles) > 15:  # Reasonable upper limit
                print(f"⚠️ Detected {len(new_obstacles)} obstacles - this seems too many, filtering to strongest candidates")
                # Sort by area and keep only the largest ones
                new_obstacles.sort(key=lambda obs: obs.get('size', 0), reverse=True)
                new_obstacles = new_obstacles[:15]
                print(f"📉 Filtered to {len(new_obstacles)} strongest obstacles")

            return new_obstacles

        except Exception as e:
            print(f"❌ Sensor processing error: {e}")
            return []
