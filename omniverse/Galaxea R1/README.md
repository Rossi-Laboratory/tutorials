# Galaxea R1: Unified Robotics System

This repository hosts the complete AI robotics framework for **Galaxea R1**, supporting both manipulation and navigation capabilities across simulated and real-world environments.

---

## 📦 Submodules

| Module         | Description                                | Path                      |
|----------------|--------------------------------------------|---------------------------|
| Navigation     | PPO-based mobile robot tasks via Isaac Sim | `Galaxea_R1_Training/`    |
| Manipulation   | Grasp, tool use, and drawer tasks          | `manipulator/`            |
| ROS2 Bridge    | Real-world policy deployment               | `ros_bridge/`             |
| Colcon WS      | ROS2 workspace for ROS nodes               | `galaxea_nav_ws/`         |

---

## 🧭 Usage Overview

- **Training**: Use `train_cli.py` in each submodule to start PPO training.
- **Deployment**: Launch trained policies using ROS2 via `bridge_node.py`.
- **Simulation**: All tasks are built to run inside Isaac Sim with interchangeable scenes and styles.

---

## 📁 Structure

```
.
├── Galaxea_R1_Training/      # Navigation tasks & scenes
├── manipulator/              # Manipulation tasks
├── ros_bridge/               # ROS2 interface & policy node
├── galaxea_nav_ws/           # Colcon workspace
├── main.py                   # Shared training interface
├── README.md                 # This file
└── whitepaper.md             # Task logic & architecture
```

---

## ✨ Project Goal

Galaxea R1 aims to unify perception, policy learning, and deployment across **navigation and manipulation**, supporting scalable training and direct sim2real deployment via NVIDIA Jetson platforms.