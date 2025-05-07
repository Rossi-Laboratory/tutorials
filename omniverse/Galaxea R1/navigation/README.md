
# Galaxea R1 Navigation

This repository contains navigation training tasks for the Galaxea R1 mobile robot, simulated in Omniverse Isaac Sim.

## Navigation Tasks Supported

1. PointGoal Navigation
2. Map-Based Navigation
3. Exploration (SLAM)
4. Semantic Goal Navigation
5. Multi-Goal Navigation
6. Dynamic Obstacle Avoidance

## Usage

```bash
python train_cli.py --task point_goal --style residential --scene point_goal_scene --episodes 1000
```

## Scene Styles

- residential
- office
- warehouse
- hospital

## Structure

- `scenes/`: Contains task-specific scene builders
- `reward_functions/`: Contains reward logic per task
- `config/`: Task YAML files specifying scene and style
