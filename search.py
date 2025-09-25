# search.py - Simple version for beginners (NO local_replan)

import heapq
from collections import deque

def get_neighbors(pos, grid, time_step=0, moving_obstacles=None):
    rows, cols = len(grid), len(grid[0])
    r, c = pos
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid[nr][nc] == 999:
                continue
            if moving_obstacles and (nr, nc) in moving_obstacles.get(time_step, []):
                continue
            neighbors.append((nr, nc))
    return neighbors

def bfs(start, goal, grid, moving_obstacles=None):
    queue = deque([(start, [start])])
    visited = set([start])
    nodes_expanded = 0
    while queue:
        current, path = queue.popleft()
        nodes_expanded += 1
        if current == goal:
            cost = sum(grid[r][c] for r, c in path[1:])
            return path, cost, nodes_expanded
        for neighbor in get_neighbors(current, grid, moving_obstacles=moving_obstacles):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))
    return None, float('inf'), nodes_expanded

def ucs(start, goal, grid, moving_obstacles=None):
    pq = [(0, start, [start])]
    visited = set()
    nodes_expanded = 0
    while pq:
        cost, current, path = heapq.heappop(pq)
        nodes_expanded += 1
        if current == goal:
            return path, cost, nodes_expanded
        if current in visited:
            continue
        visited.add(current)
        for neighbor in get_neighbors(current, grid, moving_obstacles=moving_obstacles):
            if neighbor not in visited:
                move_cost = grid[neighbor[0]][neighbor[1]]
                new_cost = cost + move_cost
                heapq.heappush(pq, (new_cost, neighbor, path + [neighbor]))
    return None, float('inf'), nodes_expanded

def astar(start, goal, grid, moving_obstacles=None):
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    pq = [(heuristic(start, goal), 0, start, [start])]
    visited = set()
    nodes_expanded = 0
    while pq:
        f, g, current, path = heapq.heappop(pq)
        nodes_expanded += 1
        if current == goal:
            return path, g, nodes_expanded
        if current in visited:
            continue
        visited.add(current)
        for neighbor in get_neighbors(current, grid, moving_obstacles=moving_obstacles):
            if neighbor not in visited:
                move_cost = grid[neighbor[0]][neighbor[1]]
                new_g = g + move_cost
                new_f = new_g + heuristic(neighbor, goal)
                heapq.heappush(pq, (new_f, new_g, neighbor, path + [neighbor]))
    return None, float('inf'), nodes_expanded