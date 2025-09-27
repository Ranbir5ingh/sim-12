# planner.py - FIXED VERSION

import heapq
import math

class AStarPlanner:
    def __init__(self, grid_size=25, robot_radius=18, inflation_radius=4):
        self.grid_size = grid_size
        self.robot_radius = robot_radius 
        self.inflation_radius = inflation_radius  # Increased default inflation
        self.diagonal_cost = 1.414  # sqrt(2) for diagonal movement

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

    def _is_obstacle_in_range(self, gx, gy, obstacles):
        """Check if a grid cell conflicts with any obstacle (with proper distance calculation)"""
        world_x, world_y = self._world_coords(gx, gy)
        
        for obs in obstacles:
            # Calculate distance from cell center to obstacle center
            dist = math.hypot(world_x - obs['x'], world_y - obs['y'])
            
            # Consider obstacle size and add comprehensive safety buffer
            obstacle_radius = obs.get('size', 30) / 2  # Half of obstacle size
            robot_clearance = self.robot_radius  # Robot's radius
            safety_buffer = 15  # Additional safety margin
            
            total_clearance_needed = obstacle_radius + robot_clearance + safety_buffer
            
            if dist < total_clearance_needed:
                return True
        return False

    def _get_cell_cost(self, gx, gy, obstacles):
        """Calculate movement cost for a cell based on proximity to obstacles"""
        base_cost = 1.0
        world_x, world_y = self._world_coords(gx, gy)
        
        min_dist_to_obstacle = float('inf')
        for obs in obstacles:
            dist = math.hypot(world_x - obs['x'], world_y - obs['y'])
            min_dist_to_obstacle = min(min_dist_to_obstacle, dist)
        
        # Add cost penalty for cells near obstacles (soft inflation)
        if min_dist_to_obstacle < 60:  # Within 60 pixels of an obstacle
            proximity_penalty = max(0, (60 - min_dist_to_obstacle) / 60) * 2.0
            return base_cost + proximity_penalty
        
        return base_cost

    def plan(self, start, goal, obstacles, canvas_w=650, canvas_h=600):
        """A* pathfinding with improved obstacle avoidance"""
        print(f"🗺️ Planning path from {start} to {goal}")
        
        sx, sy = self._gridify(*start)
        gx, gy = self._gridify(*goal)
        
        grid_w = canvas_w // self.grid_size
        grid_h = canvas_h // self.grid_size
        
        print(f"📐 Grid: {grid_w}x{grid_h}, Start: ({sx},{sy}), Goal: ({gx},{gy})")

        # Create obstacle set with both hard and soft inflation
        blocked_cells = set()
        cost_modified_cells = {}
        
        for obs in obstacles:
            obs_gx, obs_gy = self._gridify(obs['x'], obs['y'])
            
            # Hard inflation - completely blocked cells
            for dx in range(-self.inflation_radius, self.inflation_radius + 1):
                for dy in range(-self.inflation_radius, self.inflation_radius + 1):
                    nx, ny = obs_gx + dx, obs_gy + dy
                    
                    # Check bounds
                    if 0 <= nx < grid_w and 0 <= ny < grid_h:
                        # Hard block cells that are definitely too close
                        if self._is_obstacle_in_range(nx, ny, [obs]):
                            blocked_cells.add((nx, ny))
                        else:
                            # Soft inflation - higher cost cells
                            cost = self._get_cell_cost(nx, ny, [obs])
                            if cost > 1.0 and (nx, ny) not in cost_modified_cells:
                                cost_modified_cells[(nx, ny)] = cost

        print(f"🚫 Hard blocked: {len(blocked_cells)} cells, Soft penalty: {len(cost_modified_cells)} cells")
        
        # Ensure start and goal are not blocked
        if (sx, sy) in blocked_cells:
            print("⚠️ Start position is blocked! Clearing it.")
            blocked_cells.remove((sx, sy))
        
        if (gx, gy) in blocked_cells:
            print("⚠️ Goal position is blocked! Clearing it.")
            blocked_cells.remove((gx, gy))

        # A* algorithm
        open_set = [(0, (sx, sy))]
        came_from = {}
        g_score = {(sx, sy): 0}
        f_score = {(sx, sy): self._heuristic((sx, sy), (gx, gy))}
        
        # 8-directional movement with dynamic costs
        motions = [
            (1, 0, 1.0),    # Right
            (0, 1, 1.0),    # Down  
            (-1, 0, 1.0),   # Left
            (0, -1, 1.0),   # Up
            (1, 1, self.diagonal_cost),   # Diagonal down-right
            (1, -1, self.diagonal_cost),  # Diagonal up-right
            (-1, 1, self.diagonal_cost),  # Diagonal down-left
            (-1, -1, self.diagonal_cost)  # Diagonal up-left
        ]

        nodes_explored = 0
        max_nodes = grid_w * grid_h  # Prevent infinite loops
        
        while open_set and nodes_explored < max_nodes:
            current_f, current = heapq.heappop(open_set)
            nodes_explored += 1
            
            if current == (gx, gy):
                print(f"✅ Path found! Explored {nodes_explored} nodes")
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                path.reverse()
                
                # Convert back to world coordinates with path smoothing
                world_path = [self._world_coords(px, py) for px, py in path]
                
                # Optional: Smooth the path by removing unnecessary waypoints
                smoothed_path = self._smooth_path(world_path, obstacles)
                
                print(f"🛤️ Path: {len(world_path)} waypoints → {len(smoothed_path)} after smoothing")
                return smoothed_path

            # Explore neighbors
            for dx, dy, cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                # Check bounds
                if nx < 0 or ny < 0 or nx >= grid_w or ny >= grid_h:
                    continue
                
                # Check if blocked
                if neighbor in blocked_cells:
                    continue

                # Calculate movement cost including terrain penalties
                base_move_cost = cost
                terrain_cost_multiplier = cost_modified_cells.get(neighbor, 1.0)
                actual_move_cost = base_move_cost * terrain_cost_multiplier
                
                tentative_g = g_score[current] + actual_move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, (gx, gy))
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print(f"❌ No path found after exploring {nodes_explored} nodes")
        return []

    def _smooth_path(self, path, obstacles):
        """Remove unnecessary waypoints by checking line-of-sight between points"""
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]  # Always keep start point
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Try to connect current point to furthest visible point
            furthest_visible_idx = current_idx + 1
            
            for test_idx in range(current_idx + 2, len(path)):
                if self._has_line_of_sight(path[current_idx], path[test_idx], obstacles):
                    furthest_visible_idx = test_idx
                else:
                    break  # Stop at first blocked connection
            
            # Add the furthest visible point
            if furthest_visible_idx != current_idx:
                smoothed.append(path[furthest_visible_idx])
                current_idx = furthest_visible_idx
            else:
                # If we can't skip any points, move to next
                current_idx += 1
                if current_idx < len(path):
                    smoothed.append(path[current_idx])
        
        return smoothed

    def _has_line_of_sight(self, point1, point2, obstacles):
        """Check if there's a clear line of sight between two points"""
        x1, y1 = point1
        x2, y2 = point2
        
        # Sample points along the line
        distance = math.hypot(x2 - x1, y2 - y1)
        if distance == 0:
            return True
            
        num_samples = max(5, int(distance / 15))  # Sample every 15 pixels
        
        for i in range(1, num_samples):
            t = i / num_samples
            sample_x = x1 + t * (x2 - x1)
            sample_y = y1 + t * (y2 - y1)
            
            # Check if sample point conflicts with any obstacle
            for obs in obstacles:
                dist = math.hypot(sample_x - obs['x'], sample_y - obs['y'])
                obstacle_radius = obs.get('size', 30) / 2
                safety_clearance = self.robot_radius + 10
                
                if dist < (obstacle_radius + safety_clearance):
                    return False
        
        return True