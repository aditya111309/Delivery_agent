# Delivery_agent
A brief description of what this project does and who it's for

environment.py

import numpy as np

class GridEnvironment: """ Models the 2D grid environment for the delivery agent. Handles: - Loading maps from text files - Start and goal positions - Static obstacles (walls = '#') - Terrain costs (numbers, >=1) - Dynamic obstacles varying with time """

def __init__(self, map_file):
    # Load the map (grid, start, goal)
    self.grid, self.start, self.goal = self.load_map(map_file)

    # Dimensions of the grid
    self.rows, self.cols = self.grid.shape

    # Internal timer (used for dynamic obstacles that move per timestep)
    self.time = 0

    # Dict of {timestep: [(r,c), (r,c), ...]} for dynamic obstacles
    self.dynamic_obstacles = {}

def load_map(self, map_file):
    """
    Reads a grid map file where symbols have meaning:
     - 'S' = Start
     - 'G' = Goal
     - '#' = Wall (impassable)
     - 'digit' = terrain traversal cost
    Returns:
     numpy array grid, start_pos, goal_pos
    """
    grid = []
    start = goal = None
    with open(map_file) as f:
        for r, line in enumerate(f):
            row = []
            for c, ch in enumerate(line.strip()):
                if ch == 'S':
                    start = (r, c)
                    row.append(1)  # start cell acts like cost=1
                elif ch == 'G':
                    goal = (r, c)
                    row.append(1)  # goal cell is traversable
                elif ch == '#':
                    row.append(None)  # None stands for untraversable
                else:
                    row.append(int(ch))  # terrain cost
            grid.append(row)
    return np.array(grid, dtype=object), start, goal

def set_dynamic_obstacles(self, schedule):
    """
    Set schedule for dynamic obstacles.
    schedule: dict mapping timestep -> list of cells occupied at that time
    """
    self.dynamic_obstacles.update(schedule)

def is_blocked(self, pos):
    """
    Checks if a cell is blocked at the current timestep.
    Blocked if:
     - Out of bounds
     - Static wall
     - Dynamic obstacle at this timestep
    """
    r, c = pos

    # Out of bounds check
    if r < 0 or c < 0 or r >= self.rows or c >= self.cols:
        return True

    # Static wall
    if self.grid[r, c] is None:
        return True

    # Dynamic obstacles (depends on current time)
    if pos in self.dynamic_obstacles.get(self.time, []):
        return True

    return False

def step_time(self):
    """
    Move the internal clock forward by 1 timestep.
    Intended to simulate obstacles moving and environment evolving.
    """
    self.time += 1
search/utils.py -

def neighbors(env, pos): """ Returns a generator of valid, traversable neighbors (4-connected: up, down, left, right) """ r, c = pos directions = [(1,0), (-1,0), (0,1), (0,-1)] for dr, dc in directions: next_pos = (r + dr, c + dc) if not env.is_blocked(next_pos): yield next_pos

search/bfs.py -

from collections import deque from .utils import neighbors

def bfs(env): """ Breadth-First Search - Assumes uniform step cost (each move = 1). - Returns path, number of expanded nodes. """

start, goal = env.start, env.goal
frontier = deque([start])  # queue (FIFO)
parent = {start: None}     # track where each node came from
expanded = 0               # node counter

while frontier:
    node = frontier.popleft()
    expanded += 1

    if node == goal:
        return reconstruct_path(parent, node), expanded

    for n in neighbors(env, node):
        if n not in parent:   # if unseen
            parent[n] = node
            frontier.append(n)

# Goal not found
return None, expanded
def reconstruct_path(parent, node): """ Reconstructs a path from start to goal using parent pointers built during search. """ path = [] while node is not None: path.append(node) node = parent[node] return list(reversed(path))

search/ucs.py -

from heapq import heappush, heappop from .utils import neighbors

def ucs(env): """ Uniform-Cost Search - Expands lowest path-cost node first. - Guaranteed optimal (like Dijkstra). """

start, goal = env.start, env.goal
frontier = [(0, start)]     # priority queue with (cost_so_far, node)
parent = {start: None}      # track path
cost = {start: 0}           # g(n): cheapest known cost to reach n
expanded = 0

while frontier:
    g, node = heappop(frontier)
    expanded += 1

    if node == goal:
        return reconstruct_path(parent,node), expanded, g

    for n in neighbors(env, node):
        new_cost = g + env.grid[n]  # cost to neighbor
        if n not in cost or new_cost < cost[n]:
            cost[n] = new_cost
            parent[n] = node
            heappush(frontier, (new_cost, n))

# If goal not reachable
return None, expanded, float('inf')
def reconstruct_path(parent, node): path = [] while node is not None: path.append(node) node = parent[node] return list(reversed(path))

search/astar.py -

from heapq import heappush, heappop from .utils import neighbors

def manhattan(a, b): """ Manhattan distance: admissible heuristic for 4-connected grids. """ return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(env): """ A* Search Uses f(n) = g(n) + h(n) g(n): cost so far h(n): heuristic (estimated remaining cost) """

start, goal = env.start, env.goal
frontier = [(0, start)]         # (f_score, node)
parent = {start: None}
g = {start: 0}                  # actual known costs
expanded = 0

# Helpful trick: multiply heuristic by min terrain cost to avoid underestimation
min_cost = min([c for row in env.grid for c in row if c is not None])

while frontier:
    f, node = heappop(frontier)
    expanded += 1

    if node == goal:
        return reconstruct(parent, node), expanded, g[node]

    for n in neighbors(env, node):
        new_g = g[node] + env.grid[n]
        if n not in g or new_g < g[n]:
            g[n] = new_g
            parent[n] = node
            h = manhattan(n, goal) * min_cost
            heappush(frontier, (new_g + h, n))

return None, expanded, float('inf')
def reconstruct(parent, node): path = [] while node is not None: path.append(node) node = parent[node] return list(reversed(path))

search/local_replan.py -

from .astar import astar

def local_replanner(env, max_restarts=5): """ Local replanner strategy: - Starts with A* - If a dynamic obstacle blocks the path mid-way, the agent replans from its current position. - Repeats until goal or max_restarts reached. """

path = []
restarts = 0
current = env.start
total_path = [current]

while current != env.goal and restarts < max_restarts:
    # Temporarily set start position to current location
    env.start = current
    newpath, _, cost = astar(env)

    if not newpath:
        return None  # stuck

    # Drop first cell (already current location)
    for step in newpath[1:]:
        env.step_time()  # progress environment time

        # If the step is blocked by a dynamic obstacle → replan
        if env.is_blocked(step):
            restarts += 1
            break

        current = step
        total_path.append(current)
    else:
        # Finished path with no interruptions
        break

return total_path
agent.py -

from search import bfs, ucs, astar, local_replan

class DeliveryAgent: """ DeliveryAgent wraps the environment and chosen search algorithm. """

def __init__(self, env, planner):
    self.env = env
    self.planner = planner

def plan(self):
    """
    Chooses planner and runs it.
    Returns planner-specific results.
    """
    if self.planner == "bfs":
        return bfs.bfs(self.env)
    elif self.planner == "ucs":
        return ucs.ucs(self.env)
    elif self.planner == "astar":
        return astar.astar(self.env)
    elif self.planner == "local":
        return local_replan.local_replanner(self.env)
run_experiment.py -

import argparse, time from environment import GridEnvironment from agent import DeliveryAgent

def main(): # Command line interface: --map file --algo bfs|ucs|astar|local parser = argparse.ArgumentParser() parser.add_argument("--map", required=True, help="Path to grid map file") parser.add_argument("--algo", choices=["bfs","ucs","astar","local"], required=True, help="Planner type") args = parser.parse_args()

# Load environment and agent
env = GridEnvironment(args.map)
agent = DeliveryAgent(env, args.algo)

# Run planning
t0 = time.time()
result = agent.plan()
t1 = time.time()

# Display result
print("== Experiment Results ==")
print("Planner:", args.algo)
print("Runtime:", round(t1 - t0, 5), "sec")
print("Output:", result)
if name == "main": main() create readme file for this code
