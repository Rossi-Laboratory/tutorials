
# Go1LocomotionAdvPro

A full reinforcement learning and real-world deployment framework for retraining Unitree Go1 locomotion under center-of-mass (COM) shift. Includes PyTorch PPO training, observation normalization, COM curriculum, energy-aware reward, and full ROS2 bridge for Jetson deployment.

## ✅ Jetson ROS2 Deployment Guide

### 1. Environment Setup

```bash
sudo apt update && sudo apt install python3-colcon-common-extensions python3-pip
pip install torch numpy rclpy matplotlib
```

### 2. Build ROS2 Workspace

```bash
mkdir -p ~/go1_ws/src
cd ~/go1_ws/src
# Clone or copy this repo here
cp -r Go1LocomotionAdvPro/* .
cd ~/go1_ws
colcon build --packages-select go1_ros_bridge
source install/setup.bash
```

### 3. Run Inference on Jetson

```bash
ros2 launch go1_ros_bridge go1_inference_launch.py
```

Ensure `/go1/obs` topic publishes a 30-dimensional vector.

---

## ✅ Project Structure

- `train.py` - Main PPO training entry with terrain curriculum
- `cfg/` - YAML config files for training and terrain
- `rl_module/` - Policy network, reward shaping, and smoothness modules
- `inference/` - PyTorch model loader and action predictor
- `ros_interface/` - ROS2 launch and bridge nodes for Go1
- `visual/` - COM visualizer and training metrics plot
- `utils/` - Observation normalization, logging, etc.
- `curriculum/` - Curriculum scheduler for COM shift
- `goal/` - Velocity alignment reward module
