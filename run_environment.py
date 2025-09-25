import argparse, time
from environment import GridEnvironment
from agent import DeliveryAgent

def main():
    # Command line interface: --map file --algo bfs|ucs|astar|local
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, help="Path to grid map file")
    parser.add_argument("--algo", choices=["bfs","ucs","astar","local"], required=True, help="Planner type")
    args = parser.parse_args()

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

if __name__ == "__main__":
    main()