# Galaxea R1 Navigation System

Galaxea R1 is a robust mobile robot navigation framework designed for seamless sim-to-real transfer using NVIDIA Isaac Sim, ROS2, and PPO-based reinforcement learning. This repository integrates training pipelines, navigation tasks, simulation scenes, and real-world deployment infrastructure in one unified package.

---

## 📦 Repository Overview

```
galaxea_nav_combined_repo/
├── Galaxea_R1_Training/     # Navigation task configs, reward functions, scenes, training CLI
├── ros_bridge/              # ROS2-compliant policy bridge (PPO → /cmd_vel)
├── galaxea_nav_ws/          # ROS2 colcon workspace (src/ros_bridge/)
├── main.py                  # Entrypoint for training or simulation
├── jetson_deploy.md         # Jetson setup and optimization
├── galaxea_nav.service      # systemd launch script for Jetson
├── README.md, whitepaper.md
```

---

## 🚀 Key Features

- 🧭 **6 Navigation Tasks**: PointGoal, MapNav, Exploration, Semantic Goal, Multi-Goal, Dynamic Avoidance
- 🏠 **Multi-Style Scenes**: residential, office, warehouse, hospital
- 🧠 **PPO Reinforcement Learning**: Flexible training CLI and reward functions
- 🤖 **ROS2 Integration**: `/scan` → PPO model → `/cmd_vel`
- 🎯 **Jetson Deployment**: Docker-ready, systemd autostart, and performance tuning

---

## 🔧 Getting Started

### Setup (x86_64 or Jetson)

```bash
git clone <repo>
cd galaxea_nav_combined_repo
pip install -r requirements.txt
```

---

## 🎓 Training Example

```bash
cd Galaxea_R1_Training
python train_cli.py --task map_nav --scene map_nav_scene --style warehouse --episodes 1000
```

---

## 🧪 ROS2 Deployment

```bash
cd galaxea_nav_ws
colcon build
source install/setup.bash
ros2 launch ros_bridge bridge.launch.py
```

Make sure `model.pt` is placed under:
```
galaxea_nav_ws/src/ros_bridge/ros_bridge/model.pt
```

---

## ⚙️ Jetson Setup

Refer to `jetson_deploy.md` and use `galaxea_nav.service` for systemd auto-launch:

```bash
sudo cp galaxea_nav.service /etc/systemd/system/
sudo systemctl enable galaxea_nav
```

---

## 🧠 Task Summary

| Task            | Scene                  | Reward Module               |
|-----------------|------------------------|-----------------------------|
| PointGoal       | point_goal_scene.py    | navigate_to_goal.py         |
| MapNav          | map_nav_scene.py       | map_following_reward.py     |
| Exploration     | exploration_scene.py   | exploration_reward.py       |
| Semantic Goal   | semantic_goal_scene.py | semantic_goal_reward.py     |
| Multi-Goal      | multi_goal_scene.py    | multi_goal_reward.py        |
| Dynamic Avoid   | dynamic_scene.py       | dynamic_avoidance_reward.py |

---

## 📂 ROS2 Workspace Build (Optional)

```
cd galaxea_nav_ws
colcon build
source install/setup.bash
ros2 run ros_bridge bridge_node
```

---

## 📜 License

MIT License © Galaxea Research Initiative