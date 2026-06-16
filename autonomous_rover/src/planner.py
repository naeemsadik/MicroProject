import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.height, self.width = self.grid.shape 
        self.movements = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

    def heuristic(self, a, b):
        return np.hypot(b[0] - a[0], b[1] - a[1])

    def is_free(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

    def can_move(self, current, dx, dy):
        neighbor = (current[0] + dx, current[1] + dy)
        if not self.is_free(neighbor):
            return False

        if dx != 0 and dy != 0:
            side_x = (current[0] + dx, current[1])
            side_y = (current[0], current[1] + dy)
            if not self.is_free(side_x) or not self.is_free(side_y):
                return False

        return True

    def plan_path(self, start, goal):
        if not self.is_free(start) or not self.is_free(goal):
            return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in self.movements:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.can_move(current, dx, dy):
                    continue

                cost = np.sqrt(dx**2 + dy**2)
                tentative_g_score = g_score[current] + cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def has_line_of_sight(self, p1, p2):
        """
        Bresenham's Line Algorithm Raycaster.
        Checks if a straight line between p1 and p2 passes through any obstacles.
        """
        x1, y1 = p1
        x2, y2 = p2
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                # Check bounds and obstacles (1 = obstacle)
                if not (0 <= x < self.width and 0 <= y < self.height) or self.grid[y][x] == 1:
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                if not (0 <= x < self.width and 0 <= y < self.height) or self.grid[y][x] == 1:
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        # Double check final destination cell
        if not (0 <= x2 < self.width and 0 <= y2 < self.height) or self.grid[y2][x2] == 1:
            return False
        return True

    def prune_path(self, path):
        """
        Greedy Line-of-Sight path compressor.
        Reduces hundreds of staircase grid-pixels down to absolute minimum true geometric corners.
        """
        if path is None:
            return []

        if len(path) <= 2:
            return path

        smoothed_path = [path[0]]
        current_index = 0
        
        while current_index < len(path) - 1:
            # Look ahead from the end of the path backwards
            for lookahead_index in range(len(path) - 1, current_index, -1):
                if self.has_line_of_sight(path[current_index], path[lookahead_index]):
                    # If line of sight exists, skip all intermediate points and jump straight there
                    smoothed_path.append(path[lookahead_index])
                    current_index = lookahead_index
                    break
                    
        return smoothed_path
