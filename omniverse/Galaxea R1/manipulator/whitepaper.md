# Galaxea R1 Manipulation Benchmark Suite – Technical White Paper (v2)

## 1. Introduction

The Galaxea R1 Manipulation Suite addresses the challenges of robotic skill acquisition through curriculum learning, modular reward shaping, and Sim2Real deployment. It offers a structured benchmark for evaluating policy learning in industrial and household scenarios.

## 2. System Architecture

The architecture consists of a training pipeline in Isaac Lab and a deployment pipeline in ROS2 with ONNX inference. Components include task configs, scene scripts, PPO training engine, curriculum manager, ROS2 nodes, and real robot drivers.

## 3. Task Curriculum

The suite includes 9 tasks arranged by complexity:
1. Reach
2. Grasp
3. Place
4. Stack
5. Obstacle Reach
6. Peg-in-Hole
7. Drawer Opening
8. Tool Use
9. Object Sorting

Each task builds on the previous, with reward functions and success conditions encoded modularly.

## 4. Reward Design

Rewards are shaped by distance, contact, success flags, and multi-stage bonuses. Tasks like stack, drawer, and tool use require event-triggered rewards and temporal credit assignment.

## 5. Training Setup & Hyperparameters

- PPO with GAE (lambda=0.95), Adam (lr=3e-4)
- Observation space: EE pos, joint states, cube state
- Action space: 6-DoF + 1 gripper
- Episode length: 250–300 steps
- Curriculum auto-switches on threshold success rate

## 6. Benchmark Results

| Task             | PPO SR | Rule |
|------------------|--------|------|
| Reach            | 100%   | 85%  |
| Grasp            | 98%    | 60%  |
| Place            | 95%    | 40%  |
| Stack            | 89%    | —    |
| Peg-in-Hole      | 75%    | —    |
| Drawer Opening   | 80%    | —    |
| Tool Use         | 65%    | —    |
| Sorting          | 78%    | —    |

## 7. Sim2Real Deployment

The ROS2 bridge enables policy execution in real-time on Galaxea R1 via:
- ONNX model inference node
- Gripper control integration
- Joint position publishers
- State feedback for task progression

## 8. Conclusion & Outlook

The suite enables end-to-end robotic skill training, from simulation to hardware, through task curriculum and modular design. Future work includes VLM-based generalization, multi-agent collaboration, and hierarchical skill composition.

MIT Licensed. Contact authors for collaboration.
