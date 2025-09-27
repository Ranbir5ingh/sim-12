# planner.py - ROBUST VERSION for dense environments

import heapq
import math

class AStarPlanner:
    def __init__(self, grid_size=15, robot_radius=18, inflation_radius=0):  # Even finer grid, no inflation
        self.grid_size = grid_size  # Very fine grid for tight spaces
        self.robot_radius = robot_radius 
        self.inflation_radius = inflation_radius  # No inflation initially
        self.diagonal_cost = 1.414

    def _gridify(self, x, y):
        """Convert world coordinates to grid coordinates"""
        return (int(x // self.grid_size), int(y // self.grid_size))

    def _world_coords(self, gx, gy):
        """Convert grid coordinates back to world coordinates (center of cell)"""
        return (gx * self.grid_size + self.grid_size // 2, 
                gy * self.grid_size + self.grid_size // 2)

    def _heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _is_obstacle_collision(self, world_x, world_y, obstacles, safety_margin=5):
        """Check if a world coordinate collides with obstacles"""
        for obs in obstacles:
            dist = math.hypot(world_x - obs['x'], world_y - obs['y'])
            obstacle_radius = obs.get('size', 25) / 2
            total_clearance = obstacle_radius + self.robot_radius + safety_margin
            
            if dist < total_clearance:
                return True
        return False

    def _find_escape_path(self, start, obstacles):
        """Find nearest free space from start position"""
        start_x, start_y = start
        print(f"🚨 Finding escape path from dense area at {start}")
        
        # Try expanding circle around start position to find free space
        for radius in range(25, 100, 5):  # Check in expanding circles
            for angle in range(0, 360, 15):  # Check every 15 degrees
                rad = math.radians(angle)
                test_x = start_x + radius * math.cos(rad)
                test_y = start_y + radius * math.sin(rad)
                
                # Check bounds
                if test_x < 25 or test_x > 625 or test_y < 25 or test_y > 575:
                    continue
                
                if not self._is_obstacle_collision(test_x, test_y, obstacles, safety_margin=3):
                    print(f"🎯 Found escape point at ({test_x:.1f}, {test_y:.1f})")
                    return [(int(test_x), int(test_y))]
        
        print("❌ No escape path found")
        return []

    def plan(self, start, goal, obstacles, canvas_w=650, canvas_h=600):
        """Multi-strategy pathfinding for dense environments"""
        print(f"🗺️ Planning path from {start} to {goal} with {len(obstacles)} obstacles")
        
        # STRATEGY 1: Direct path (most aggressive)
        if self._has_line_of_sight(start, goal, obstacles, safety_margin=3):
            print("🚀 Direct path available!")
            return [goal]
        
        # STRATEGY 2: Check if start position is trapped
        if self._is_obstacle_collision(start[0], start[1], obstacles, safety_margin=5):
            print("⚠️ Robot appears to be trapped in obstacles - finding escape path")
            escape_path = self._find_escape_path(start, obstacles)
            if escape_path:
                # Try to plan from escape point to goal
                escape_to_goal = self.plan(escape_path[0], goal, obstacles, canvas_w, canvas_h)
                if escape_to_goal:
                    return escape_path + escape_to_goal
            return []
        
        # STRATEGY 3: Try with different safety margins (adaptive planning)
        for safety_margin in [3, 8, 15]:  # Start aggressive, get more conservative
            print(f"🔄 Attempting A* with safety margin: {safety_margin}px")
            path = self._astar_with_margin(start, goal, obstacles, canvas_w, canvas_h, safety_margin)
            if path:
                return path
        
        # STRATEGY 4: Escape-based planning if all else fails
        print("🚨 Standard A* failed - trying escape-based planning")
        return self._escape_based_planning(start, goal, obstacles)

    def _astar_with_margin(self, start, goal, obstacles, canvas_w, canvas_h, safety_margin):
        """A* implementation with configurable safety margin"""
        sx, sy = self._gridify(*start)
        gx, gy = self._gridify(*goal)
        
        grid_w = canvas_w // self.grid_size
        grid_h = canvas_h // self.grid_size
        
        # Create obstacle map with current safety margin
        blocked_cells = set()
        
        for obs in obstacles:
            obs_gx, obs_gy = self._gridify(obs['x'], obs['y'])
            
            # Check area around obstacle
            check_radius = max(1, (obs.get('size', 25) // 2 + self.robot_radius + safety_margin) // self.grid_size)
            
            for dx in range(-check_radius, check_radius + 1):
                for dy in range(-check_radius, check_radius + 1):
                    nx, ny = obs_gx + dx, obs_gy + dy
                    
                    if 0 <= nx < grid_w and 0 <= ny < grid_h:
                        world_x, world_y = self._world_coords(nx, ny)
                        if self._is_obstacle_collision(world_x, world_y, [obs], safety_margin):
                            blocked_cells.add((nx, ny))

        # Clear start and goal positions
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                blocked_cells.discard((sx + dx, sy + dy))
                blocked_cells.discard((gx + dx, gy + dy))

        print(f"  🚫 Blocked {len(blocked_cells)} cells with {safety_margin}px margin")

        # A* Search
        open_set = [(0, (sx, sy))]
        came_from = {}
        g_score = {(sx, sy): 0}
        f_score = {(sx, sy): self._heuristic((sx, sy), (gx, gy))}
        closed_set = set()
        
        # All 8 directions
        motions = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, self.diagonal_cost), (1, -1, self.diagonal_cost),
            (-1, 1, self.diagonal_cost), (-1, -1, self.diagonal_cost)
        ]

        nodes_explored = 0
        max_nodes = 8000  # Increased for dense environments
        
        while open_set and nodes_explored < max_nodes:
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
            nodes_explored += 1
            
            if current == (gx, gy):
                print(f"  ✅ Path found with {safety_margin}px margin! ({nodes_explored} nodes)")
                
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                path.reverse()
                
                # Convert to world coordinates
                world_path = [self._world_coords(px, py) for px, py in path]
                
                # Smooth path
                smoothed_path = self._smart_smooth_path(world_path, obstacles, safety_margin)
                print(f"  🛤️ {len(world_path)} → {len(smoothed_path)} waypoints after smoothing")
                
                return smoothed_path

            # Explore neighbors
            for dx, dy, cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                if nx < 0 or ny < 0 or nx >= grid_w or ny >= grid_h:
                    continue
                
                if neighbor in blocked_cells or neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, (gx, gy))
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print(f"  ❌ No path found with {safety_margin}px margin ({nodes_explored} nodes)")
        return []

    def _escape_based_planning(self, start, goal, obstacles):
        """Last resort: find intermediate waypoints to escape dense areas"""
        print("🚁 Escape-based planning: finding intermediate waypoints")
        
        # Find clear intermediate points between start and goal
        waypoints = [start]
        
        current_pos = start
        while True:
            # Find direction toward goal
            dx = goal[0] - current_pos[0]
            dy = goal[1] - current_pos[1]
            distance = math.hypot(dx, dy)
            
            if distance < 50:  # Close to goal
                waypoints.append(goal)
                break
            
            # Normalize direction
            nx = dx / distance
            ny = dy / distance
            
            # Try different step sizes to find next valid waypoint
            found_waypoint = False
            for step in [30, 40, 50, 60]:
                test_x = current_pos[0] + nx * step
                test_y = current_pos[1] + ny * step
                
                # Check bounds
                if test_x < 25 or test_x > 625 or test_y < 25 or test_y > 575:
                    continue
                
                if not self._is_obstacle_collision(test_x, test_y, obstacles, safety_margin=5):
                    waypoints.append((int(test_x), int(test_y)))
                    current_pos = (test_x, test_y)
                    found_waypoint = True
                    print(f"  📍 Added waypoint at ({test_x:.1f}, {test_y:.1f})")
                    break
            
            if not found_waypoint:
                # Try perpendicular directions to escape
                for angle_offset in [90, -90, 45, -45, 135, -135]:
                    angle = math.atan2(dy, dx) + math.radians(angle_offset)
                    for step in [25, 35, 45]:
                        test_x = current_pos[0] + step * math.cos(angle)
                        test_y = current_pos[1] + step * math.sin(angle)
                        
                        if (25 <= test_x <= 625 and 25 <= test_y <= 575 and
                            not self._is_obstacle_collision(test_x, test_y, obstacles, safety_margin=5)):
                            waypoints.append((int(test_x), int(test_y)))
                            current_pos = (test_x, test_y)
                            found_waypoint = True
                            print(f"  🔄 Escape waypoint at ({test_x:.1f}, {test_y:.1f})")
                            break
                    if found_waypoint:
                        break
            
            if not found_waypoint or len(waypoints) > 10:  # Prevent infinite loops
                break
        
        if len(waypoints) > 1:
            print(f"📍 Found {len(waypoints)} waypoints for escape path")
            return waypoints[1:]  # Return without start point
        else:
            print("❌ Escape-based planning failed")
            return []

    def _smart_smooth_path(self, path, obstacles, safety_margin):
        """Smart path smoothing that maintains safety"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            furthest_idx = current_idx + 1
            
            # Try to jump as far as possible while maintaining safety
            for test_idx in range(current_idx + 2, len(path)):
                if self._has_line_of_sight(path[current_idx], path[test_idx], obstacles, safety_margin):
                    furthest_idx = test_idx
                else:
                    break
            
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx
        
        return smoothed

    def _has_line_of_sight(self, point1, point2, obstacles, safety_margin=8):
        """Check line of sight between two points"""
        x1, y1 = point1
        x2, y2 = point2
        
        distance = math.hypot(x2 - x1, y2 - y1)
        if distance == 0:
            return True
        
        # Check more samples for dense environments
        num_samples = max(8, int(distance / 8))
        
        for i in range(1, num_samples):
            t = i / num_samples
            sample_x = x1 + t * (x2 - x1)
            sample_y = y1 + t * (y2 - y1)
            
            if self._is_obstacle_collision(sample_x, sample_y, obstacles, safety_margin):
                return False
        
        return True
