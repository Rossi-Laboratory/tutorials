# Galaxea R1 Navigation System

Welcome to the official repository for **Galaxea R1 Navigation**, an advanced AI-based mobile robotics platform powered by Isaac Sim and ROS2.  
This repository includes training, simulation, deployment, and real-world integration components—all designed to accelerate sim-to-real robotics development.

---

## 🔧 Project Structure

```
galaxea_nav_combined_repo/
├── Galaxea_R1_Training/         # Navigation tasks, scenes, and PPO training
├── ros_bridge/                  # ROS2-compliant package with PPO policy node
├── galaxea_nav_ws/             # Colcon workspace for ROS2 build
├── README.md                    # This documentation
├── whitepaper.md                # Technical overview of navigation tasks
├── LICENSE                      # MIT License
├── requirements.txt            # PyTorch + rclpy dependencies
└── .github/workflows/          # GitHub Actions CI
```

---

## 🚀 Features

- 🧠 **PPO-based Navigation Policy** for 6 tasks: PointGoal, MapNav, Exploration, Semantic, Multi-Goal, Dynamic Avoidance
- 🌎 **Scene Modularization** with multi-style layout (office, residential, warehouse)
- 🤖 **ROS2 Integration**: `/scan` → PPO policy → `/cmd_vel` via bridge_node
- 🧪 **Sim-to-Real Ready** with Jetson deployment, domain randomization, and systemd support

---

## 🏁 How to Get Started

### 1. Clone & Install Dependencies

```bash
git clone <your-repo>
cd galaxea_nav_combined_repo
pip install -r requirements.txt
```

### 2. Training in Isaac Sim

```bash
cd Galaxea_R1_Training
python train_cli.py --task point_goal --scene point_goal_scene --style office --episodes 1000
```

---

## 🤖 ROS2 Deployment

### 1. Build ROS2 workspace (inside `galaxea_nav_ws/`)

```bash
cd galaxea_nav_ws
colcon build
source install/setup.bash
ros2 launch ros_bridge bridge.launch.py
```

### 2. Model Inference

- Drop your trained PyTorch model into `galaxea_nav_ws/src/ros_bridge/ros_bridge/model.pt`
- The bridge node automatically loads the model and executes policy over `/scan`

---

## 🧩 Tasks & Scene List

| Task               | Scene Module           | Reward Function                |
|--------------------|------------------------|--------------------------------|
| PointGoal          | point_goal_scene.py    | navigate_to_goal.py            |
| Map-Based Nav      | map_nav_scene.py       | map_following_reward.py        |
| Exploration        | exploration_scene.py   | exploration_reward.py          |
| Semantic Goal      | semantic_goal_scene.py | semantic_goal_reward.py        |
| Multi-Goal Patrol  | multi_goal_scene.py    | multi_goal_reward.py           |
| Dynamic Avoidance  | dynamic_scene.py       | dynamic_avoidance_reward.py    |

---

## 🖥️ Jetson Deployment

Refer to `jetson_deploy.md` for:
- ROS2 + CUDA setup on Xavier / Orin
- nvpmodel, jetson_clocks tuning
- Autostart with `systemd`
- LIDAR + camera calibration instructions

---

## 📜 License

This project is licensed under the MIT License. See `LICENSE` for details.

---

## 👨‍💻 Maintainer

This repository is maintained by a senior engineer at NVIDIA Robotics, with a focus on sim2real autonomy, Isaac Sim integration, and embedded robotics deployment.

For advanced usage or contributions, please open an issue or contact the maintainer.