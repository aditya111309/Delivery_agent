from search import bfs, ucs, astar, local_replan

class DeliveryAgent:
    """
    DeliveryAgent wraps the environment and chosen search algorithm.
    """

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
