import numpy as np

class GridEnvironment:
    """
    Models the 2D grid environment for the delivery agent.
    Handles:
     - Loading maps from text files
     - Start and goal positions
     - Static obstacles (walls = '#')
     - Terrain costs (numbers, >=1)
     - Dynamic obstacles varying with time
    """

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