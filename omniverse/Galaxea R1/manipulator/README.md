# Galaxea R1 Manipulation Benchmark Suite

This project provides a complete end-to-end pipeline for training, evaluating, and deploying robotic manipulation skills on the Galaxea R1 robot. It includes curriculum learning over **9 progressive tasks**, integration with **Isaac Lab**, **ROS 2**, **ONNX deployment**, and support for Sim2Real applications.

---

## 📁 Project Structure

```
galaxea-isaac-sim2real/
├── Galaxea_R1_Training/
│   ├── main.py                     # PPO training entry
│   ├── train_cli.py                # CLI interface
│   ├── export_model.py             # Export to ONNX
│   ├── evaluate_model.py           # Run trained model
│   ├── benchmark_runner.py         # Compare PPO vs rule-based
│   ├── config/                     # Task configs (9 tasks)
│   ├── reward_functions/          # Custom reward scripts
│   ├── scenes/                    # Scene setup scripts
│   └── utils/                     # Helper modules (logging, curriculum, etc)
├── ros_bridge/                    # ROS 2 integration
│   ├── launch/
│   ├── config/
│   ├── hardware_interface/
│   └── policy_inference_node.py
├── .github/workflows/            # CI testing
├── requirements.txt              # Python dependencies
├── .gitignore
└── README.md
```

---

## 🧠 Supported Tasks (Curriculum Stages)

| Stage | Task                     | Description                             |
|-------|--------------------------|-----------------------------------------|
|  1    | Reach                   | Move EE to a cube target                |
|  2    | Grasp                   | Grasp the cube (with lift detection)    |
|  3    | Place                   | Move lifted cube to placement zone      |
|  4    | Stack                   | Stack cube on a second cube             |
|  5    | Obstacle-Aware Reach    | Reach with static obstacles             |
|  6    | Peg-in-Hole             | Align and insert peg into hole          |
|  7    | Drawer Opening          | Open a drawer through EE interaction    |
|  8    | Tool Use                | Use a tool to reach an external target  |
|  9    | Object Sorting          | Pick and classify objects into bins     |

---

## 🚀 Usage Examples

**Training (any task):**
```bash
python train_cli.py --task_cfg Galaxea_R1_Training/config/task_stack.yaml --epochs 1000
```

**Evaluate Model:**
```bash
python evaluate_model.py
```

**Export to ONNX:**
```bash
python export_model.py
```

**Run Benchmark:**
```bash
python benchmark_runner.py
```

---

## 🤖 ROS 2 Deployment

- Launch bridge:
```bash
ros2 launch ros_bridge/launch/galaxea_bridge.launch.py
```
- Run ONNX policy node:
```bash
ros2 run ros_bridge policy_inference_node.py
```

---

## 🧪 Research Features
- Curriculum learning (9 stages)
- Domain randomization & gripper control
- Dataset collection (`data_collector.py`)
- ONNX explainability (`onnx_explain.py`)
- Modular observations & task switching

---

## 📄 License
MIT License
