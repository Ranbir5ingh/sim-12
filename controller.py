# controller.py - FIXED VERSION

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
        # FIXED: Increased grid size and inflation for better clearance
        self.planner = AStarPlanner(grid_size=25, inflation_radius=3)
        self.mapper = ObstacleMapper() 
        self.server_url = "http://localhost:5001"
        self.is_collided = False
        self.goal_reached = False
        self.current_waypoint_index = 0
        # FIXED: Slower sensing for more careful navigation
        self.sensing_interval = 0.15
        # FIXED: Add step size for gradual movement
        self.step_size = 15

    async def _capture_and_map(self):
        """Captures the environment, updates the map, and returns True if new obstacles are detected."""
        try:
            url = f"{self.server_url}/capture"
            r = requests.get(url, timeout=1.0)  # Increased timeout
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
            
    def _gradual_move_to_waypoint(self, target_x, target_y):
        """Move gradually towards waypoint in smaller steps"""
        current_x, current_y = self.robot_pos
        
        # Calculate direction vector
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        
        if distance == 0:
            return [(target_x, target_y)]
        
        # Normalize direction and create intermediate points
        steps = max(1, int(distance / self.step_size))
        step_x = dx / steps
        step_y = dy / steps
        
        intermediate_points = []
        for i in range(1, steps + 1):
            new_x = current_x + step_x * i
            new_y = current_y + step_y * i
            intermediate_points.append((int(new_x), int(new_y)))
            
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
                
            # Execute the planned path with gradual movement
            while self.current_waypoint_index < len(self.path) and not self.is_collided and not self.goal_reached:
                waypoint = self.path[self.current_waypoint_index]
                print(f"🎯 Moving to waypoint {self.current_waypoint_index + 1}/{len(self.path)}: ({waypoint[0]}, {waypoint[1]})")
                
                # Move gradually to waypoint
                intermediate_points = self._gradual_move_to_waypoint(waypoint[0], waypoint[1])
                
                for point in intermediate_points:
                    if self.is_collided or self.goal_reached:
                        break
                        
                    self.move_to(point[0], point[1])
                    self.robot_pos = point
                    
                    # Check for obstacles during movement
                    await asyncio.sleep(self.sensing_interval)
                    
                    # Proactive obstacle detection during transit
                    if await self._capture_and_map():
                        print("🚨 New obstacle detected during movement - breaking to replan")
                        break
                
                if not self.is_collided and not await self._capture_and_map():
                    self.current_waypoint_index += 1
                else:
                    break  # Replan needed

            # Brief pause before next iteration
            if not self.is_collided and not self.goal_reached:
                await asyncio.sleep(0.2)
                
        print("🏁 Path execution loop finished.")
            
if __name__ == "__main__":
    controller = RobotController()
    asyncio.run(controller.connect())