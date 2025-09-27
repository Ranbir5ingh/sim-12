# controller.py - IMPROVED FAST VERSION

import asyncio
import websockets
import json
import requests
import math
from planner import AStarPlanner
from sensor import ObstacleMapper 

class RobotController:
    # Set the fixed start position for easy reference
    START_POS = {"x": 320, "y": 300}

    def __init__(self):
        self.robot_pos = (self.START_POS["x"], self.START_POS["y"])
        self.goal = {"x": 550, "y": 80}
        self.known_obstacles = [] 
        self.path = []
        self.ws = None
        # Grid size and inflation for better clearance
        self.planner = AStarPlanner(grid_size=25, inflation_radius=3)
        self.mapper = ObstacleMapper() 
        self.server_url = "http://localhost:5001"
        self.is_collided = False
        self.goal_reached = False
        self.current_waypoint_index = 0
        
        # IMPROVED: Dynamic parameters for speed optimization
        self.base_sensing_interval = 0.05  # Much faster base sensing
        self.obstacle_sensing_interval = 0.15  # Slower when near obstacles
        self.min_step_size = 25  # Larger minimum steps
        self.max_step_size = 50  # Even larger steps when path is clear
        self.safe_distance_threshold = 80  # Distance to consider "safe" from obstacles

    async def _capture_and_map(self):
        """Captures the environment, updates the map, and returns True if new obstacles are detected."""
        try:
            url = f"{self.server_url}/capture"
            r = requests.get(url, timeout=1.0)
            r.raise_for_status()
            data = r.json()

            if data.get('status') == 'success':
                image_data = data['image_data']
                new_obs = self.mapper.process_capture(image_data, self.known_obstacles)
                
                if new_obs:
                    print(f"🔍 Detected {len(new_obs)} new obstacles")
                    for obs in new_obs:
                        print(f"  - Obstacle at ({obs['x']}, {obs['y']})")
                    self.known_obstacles.extend(new_obs)
                    return True 
                
            return False

        except requests.exceptions.RequestException as e:
            print(f"Sensing failed: {e}")
            return False

    def _get_nearest_obstacle_distance(self, x, y):
        """Get distance to nearest known obstacle"""
        if not self.known_obstacles:
            return float('inf')
        
        min_dist = float('inf')
        for obs in self.known_obstacles:
            dist = math.hypot(x - obs['x'], y - obs['y'])
            min_dist = min(min_dist, dist)
        return min_dist

    def _is_path_clear_to_waypoint(self, current_pos, waypoint):
        """Check if path to waypoint is clear of obstacles"""
        x1, y1 = current_pos
        x2, y2 = waypoint
        
        # Sample points along the path
        distance = math.hypot(x2 - x1, y2 - y1)
        if distance == 0:
            return True
            
        num_samples = max(3, int(distance / 20))  # Sample every 20 pixels
        
        for i in range(1, num_samples + 1):
            t = i / num_samples
            sample_x = x1 + t * (x2 - x1)
            sample_y = y1 + t * (y2 - y1)
            
            # Check if sample point is too close to obstacles
            if self._get_nearest_obstacle_distance(sample_x, sample_y) < 40:  # Safety margin
                return False
        
        return True

    async def _reset_and_reinit_env(self):
        """Calls /reset, then re-sends all known obstacles for the next plan."""
        try:
            print("🔄 Resetting environment...")
            # 1. Call /reset to move robot to START_POS (320, 300) and clear collision status
            requests.post(f"{self.server_url}/reset", timeout=2.0)
            
            # 2. Re-send all known obstacles (as /reset clears the server's list)
            if self.known_obstacles:
                payload = {"obstacles": self.known_obstacles}
                requests.post(f"{self.server_url}/obstacles/positions", json=payload, timeout=2.0)
                print(f"📍 Re-sent {len(self.known_obstacles)} known obstacles to server")

            # 3. Update local state
            self.robot_pos = (self.START_POS["x"], self.START_POS["y"])
            self.path = []
            self.current_waypoint_index = 0
            self.is_collided = True # Force re-plan immediately
            
            print(f"✅ Reset complete. Starting from {self.robot_pos}")
            
        except requests.exceptions.RequestException as e:
            print(f"❌ Failed to reset environment: {e}")
            self.goal_reached = True 

    async def connect(self):
        uri = "ws://localhost:8080"
        async with websockets.connect(uri) as websocket:
            self.ws = websocket
            print("🔗 Connected to simulator")

            listen_task = asyncio.create_task(self._listen_for_feedback())
            act_task = asyncio.create_task(self._path_execution_loop())

            await asyncio.gather(listen_task, act_task)

    def move_to(self, x, y):
        """Move robot with error handling"""
        url = f"{self.server_url}/move"
        payload = {"x": x, "y": y}
        try:
            response = requests.post(url, json=payload, timeout=0.1)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(f"Move command failed: {e}")
    
    def _adaptive_move_to_waypoint(self, target_x, target_y):
        """Adaptively move towards waypoint - larger steps when safe, smaller when near obstacles"""
        current_x, current_y = self.robot_pos
        
        # Calculate direction vector
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        
        if distance == 0:
            return [(target_x, target_y)]
        
        # Determine step size based on proximity to obstacles
        nearest_obstacle_dist = self._get_nearest_obstacle_distance(current_x, current_y)
        
        # Adaptive step size: larger when far from obstacles, smaller when close
        if nearest_obstacle_dist > self.safe_distance_threshold:
            step_size = self.max_step_size  # Large steps when safe
            print(f"🏃 Fast movement: nearest obstacle {nearest_obstacle_dist:.1f}px away")
        elif nearest_obstacle_dist > 50:
            step_size = (self.min_step_size + self.max_step_size) // 2  # Medium steps
            print(f"🚶 Medium movement: nearest obstacle {nearest_obstacle_dist:.1f}px away")
        else:
            step_size = self.min_step_size  # Small steps when close to obstacles
            print(f"🐌 Careful movement: nearest obstacle {nearest_obstacle_dist:.1f}px away")
        
        # If path is clear and distance is reasonable, move directly
        if distance <= step_size * 1.5 and self._is_path_clear_to_waypoint((current_x, current_y), (target_x, target_y)):
            print(f"✨ Direct movement to waypoint: {distance:.1f}px")
            return [(target_x, target_y)]
        
        # Create intermediate points with adaptive step size
        steps = max(1, int(distance / step_size))
        step_x = dx / steps
        step_y = dy / steps
        
        intermediate_points = []
        for i in range(1, steps + 1):
            new_x = current_x + step_x * i
            new_y = current_y + step_y * i
            intermediate_points.append((int(new_x), int(new_y)))
        
        print(f"📍 Moving in {len(intermediate_points)} steps (step_size: {step_size})")
        return intermediate_points
            
    async def _listen_for_feedback(self):
        """Listens for goal or collision events from the simulator."""
        async for message in self.ws:
            data = json.loads(message)
            if data.get("type") == "goal_reached":
                print("🎯 Goal reached! Stopping controller.")
                self.goal_reached = True
                break
            if data.get("type") == "collision":
                print("⚠️ COLLISION detected! Adding obstacle and restarting...")
                
                col_x = data['robot_position']['x']
                col_y = data['robot_position']['y']
                
                # 1. Mark collision point as obstacle with larger buffer
                collision_obstacle = {
                    "x": col_x, 
                    "y": col_y, 
                    "size": 35  # Larger size to avoid repeat collisions
                }
                
                # Check if collision point is already known
                is_new_collision = True
                for obs in self.known_obstacles:
                    if math.hypot(col_x - obs['x'], col_y - obs['y']) < 40:
                        is_new_collision = False
                        break
                        
                if is_new_collision:
                    self.known_obstacles.append(collision_obstacle)
                    print(f"📍 Added collision obstacle at ({col_x}, {col_y})")

                # 2. Stop robot immediately
                self.move_to(col_x, col_y)

                # 3. Reset the environment and re-plan from the start
                await self._reset_and_reinit_env()
                
    async def _path_execution_loop(self):
        """Handles path planning, sensing, and execution in a continuous loop."""
        # Initial environment scan
        print("🔍 Performing initial environment scan...")
        await self._capture_and_map()
        
        while not self.goal_reached:
            # Check for obstacles before planning
            new_obs_detected = await self._capture_and_map() 

            needs_replan = not self.path or self.is_collided or new_obs_detected

            if needs_replan:
                if self.is_collided:
                    print(f"🔄 Re-planning after collision from {self.robot_pos}")
                    self.is_collided = False
                elif new_obs_detected:
                    print("🔄 Re-planning due to new obstacles detected")
                else:
                    print("🔄 Initial path planning...")
                
                print(f"📊 Planning with {len(self.known_obstacles)} known obstacles")
                
                # Plan path with current known obstacles
                self.path = self.planner.plan(
                    self.robot_pos,
                    (self.goal["x"], self.goal["y"]),
                    self.known_obstacles 
                )
                
                # Remove the redundant first waypoint if it's too close to current position
                if self.path and len(self.path) > 1:
                    first_waypoint = self.path[0]
                    if math.hypot(first_waypoint[0] - self.robot_pos[0], 
                                first_waypoint[1] - self.robot_pos[1]) < 30:
                        self.path = self.path[1:]
                
                self.current_waypoint_index = 0

                if not self.path:
                    print("🛑 No valid path found! Trying to sense more obstacles...")
                    await asyncio.sleep(2)
                    continue

                print(f"✅ Planned path with {len(self.path)} waypoints")
                
            # Execute the planned path with adaptive movement
            while self.current_waypoint_index < len(self.path) and not self.is_collided and not self.goal_reached:
                waypoint = self.path[self.current_waypoint_index]
                print(f"🎯 Moving to waypoint {self.current_waypoint_index + 1}/{len(self.path)}: ({waypoint[0]}, {waypoint[1]})")
                
                # IMPROVED: Adaptive movement to waypoint
                intermediate_points = self._adaptive_move_to_waypoint(waypoint[0], waypoint[1])
                
                # Determine sensing frequency based on proximity to obstacles
                current_nearest_dist = self._get_nearest_obstacle_distance(self.robot_pos[0], self.robot_pos[1])
                if current_nearest_dist > self.safe_distance_threshold:
                    sensing_interval = self.base_sensing_interval  # Fast sensing when safe
                else:
                    sensing_interval = self.obstacle_sensing_interval  # Careful sensing near obstacles
                
                for point_idx, point in enumerate(intermediate_points):
                    if self.is_collided or self.goal_reached:
                        break
                        
                    self.move_to(point[0], point[1])
                    self.robot_pos = point
                    
                    # IMPROVED: Adaptive sensing - sense more frequently only when needed
                    await asyncio.sleep(sensing_interval)
                    
                    # Only check for new obstacles every few steps when moving fast
                    if point_idx == len(intermediate_points) - 1 or point_idx % max(1, len(intermediate_points) // 3) == 0:
                        if await self._capture_and_map():
                            print("🚨 New obstacle detected during movement - breaking to replan")
                            break
                
                if not self.is_collided and not await self._capture_and_map():
                    self.current_waypoint_index += 1
                else:
                    break  # Replan needed

            # IMPROVED: Minimal pause between iterations when moving fast
            if not self.is_collided and not self.goal_reached:
                current_nearest_dist = self._get_nearest_obstacle_distance(self.robot_pos[0], self.robot_pos[1])
                if current_nearest_dist > self.safe_distance_threshold:
                    await asyncio.sleep(0.05)  # Very short pause when safe
                else:
                    await asyncio.sleep(0.2)   # Longer pause when near obstacles
                
        print("🏁 Path execution loop finished.")
            
if __name__ == "__main__":
    controller = RobotController()
    asyncio.run(controller.connect())
