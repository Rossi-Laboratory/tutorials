
import argparse
from Galaxea_R1_Navigation.main import train

def parse_args():
    parser = argparse.ArgumentParser(description="Galaxea R1 Navigation Training CLI")
    parser.add_argument('--task', type=str, default='point_goal', help='Name of the navigation task')
    parser.add_argument('--style', type=str, default='residential', help='Environment style (residential, office, warehouse)')
    parser.add_argument('--scene', type=str, default='point_goal_scene', help='Scene module to load')
    parser.add_argument('--episodes', type=int, default=1000, help='Number of training episodes')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    train(task=args.task, style=args.style, scene=args.scene, episodes=args.episodes)
