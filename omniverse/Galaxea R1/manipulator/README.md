# Galaxea R1 Manipulation Benchmark Suite

This benchmark suite provides a full-stack pipeline for learning robotic manipulation using the Galaxea R1 manipulator in simulation and Sim2Real contexts. The system is built upon NVIDIA Isaac Lab, and integrates curriculum learning, task switching, ROS 2 deployment, and ONNX-based inference. It is suitable for academic research, industrial prototyping, and real-robot transfer learning.

---

## 📁 Project Structure

```
galaxea-isaac-sim2real/
├── Galaxea_R1_Training/         # Core RL training logic and tasks
│   ├── main.py                  # PPO training launcher
│   ├── train_cli.py             # Command-line training interface
│   ├── export_model.py          # Convert trained model to ONNX
│   ├── evaluate_model.py        # Run trained policy in sim
│   ├── benchmark_runner.py      # Run multiple trials across baselines
│   ├── config/                  # YAML task configs (9 tasks)
│   ├── reward_functions/        # Custom reward logic for each task
│   ├── scenes/                  # Sim scene setup scripts
│   └── utils/                   # Curriculum, logging, gripper logic, etc.
├── ros_bridge/                  # ROS2 Sim2Real and deployment
│   ├── launch/                  # ROS2 launch files
│   ├── config/                  # ros2_control configuration
│   ├── hardware_interface/      # Mock or real robot drivers
│   └── policy_inference_node.py # ONNX policy executor for ROS2
├── .github/workflows/           # GitHub CI (syntax check)
├── requirements.txt             # Python dependencies
├── .gitignore                   # Files to ignore
└── README.md                    # Documentation
```

---

## 🧠 Supported Tasks (Curriculum Learning Pipeline)

The system supports 9 progressively complex manipulation tasks:

| Stage | Task                     | Description |
|-------|--------------------------|-------------|
| 1     | Reach                   | Move end-effector to a target position |
| 2     | Grasp                   | Approach and lift a cube using gripper |
| 3     | Place                   | Transport the lifted cube to a drop zone |
| 4     | Stack                   | Place the cube on top of a fixed block |
| 5     | Obstacle-Aware Reach    | Avoid static obstacles while reaching target |
| 6     | Peg-in-Hole             | Align and insert a peg into a socket |
| 7     | Drawer Opening          | Pull open a drawer through interaction |
| 8     | Tool Use                | Use an external object (tool) to reach target |
| 9     | Object Sorting          | Identify, pick, and place items into correct bins |

Each task includes:
- A dedicated reward function
- Scene layout configuration
- Task config YAML file
- Optional gripper/tool control

---

## 🚀 Training & Evaluation

**Train PPO on any task:**
```bash
python train_cli.py --task_cfg Galaxea_R1_Training/config/task_peg.yaml --epochs 1500
```

**Evaluate trained policy:**
```bash
python evaluate_model.py
```

**Export PyTorch model to ONNX:**
```bash
python export_model.py
```

**Compare PPO with rule-based baseline:**
```bash
python benchmark_runner.py
```

**Run curriculum sequence automatically:**
Handled via `curriculum_manager.py`. Each stage automatically advances based on success rate or epoch.

---

## 🤖 ROS 2 Deployment (Sim2Real Ready)

- Launch simulation + bridge:
```bash
ros2 launch ros_bridge/launch/galaxea_bridge.launch.py
```

- Run ONNX policy node for real robot:
```bash
ros2 run ros_bridge policy_inference_node.py
```

---

## 🛠 Advanced Features

- **Curriculum Learning**: Modular task switcher with stage advancement
- **Gripper & Tool Control**: Gripper width modeled as action dim; Tool task supports tool-use reward
- **Explainability**: `onnx_explain.py` visualizes input saliency using perturbation-based attribution
- **Dataset Collection**: `data_collector.py` records trajectory data for offline RL or behavior cloning
- **Observation Management**: `observation_manager.py` filters and formats state input
- **Training Logs**: TensorBoard logger support for reward, loss, etc.
- **Checkpoint & Early Stop**: via `trainer_callback.py`

---

## 🧪 Benchmark Use Cases
- RL curriculum benchmarks (Reach → Stack → Peg-in-Hole)
- Sim2Real policy deployment with ROS2
- Visual saliency studies on ONNX models
- Tool-using manipulator for long-horizon planning
- Object recognition + manipulation (sorting task)

---

## 📄 License
MIT License — free for academic and industrial use.

---

For questions, please contact the project lead.
