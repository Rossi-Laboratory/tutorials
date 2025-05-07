# 🐾 How to Train Go1 to Walk with Go1LocomotionAdvPro

This guide walks you through training the Unitree Go1 quadruped to walk under altered dynamics (e.g., center of mass shifts due to added sensors), using the `Go1LocomotionAdvPro` framework.

---

## 📦 Step 1: Environment Setup

Ensure the following dependencies are installed on your development machine or Jetson:

```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions
pip install torch numpy matplotlib rclpy
```

---

## ⚙️ Step 2: Configure Your Training

Edit the training configuration file:

`cfg/train_go1_sensor_advanced.yaml`

You can customize:
- `reward_weights`: tune for velocity, energy, smoothness
- `curriculum`: schedule for COM shift adaptation
- `randomization`: physical variability during training
- `features`: enable normalization, residual policy, smoothing

Example:
```yaml
env:
  use_goal_input: true
  reward_weights:
    goal_velocity: 1.0
    energy: 0.01
  curriculum:
    enable: true
    steps: [5000, 15000]
  randomization:
    enable: true
    mass: [0.95, 1.05]
    friction: [0.6, 1.4]
features:
  obs_normalization: true
  action_smooth_penalty: true
  residual_control: false
```

---

## 🏃 Step 3: Start Training

Run PPO training with:
```bash
python3 train.py --cfg cfg/train_go1_sensor_advanced.yaml
```

This will begin training your locomotion policy. TensorBoard logs and reward breakdowns are recorded automatically.

---

## 🧠 Step 4: Export the Trained Model

After training completes, export the final policy checkpoint (e.g., `go1_policy.pth`).

---

## 🤖 Step 5: Real-World Deployment

Transfer the trained model to your Jetson platform. Use the deployment script:

```bash
./run_deploy.sh
```

This launches:
- `go1_obs_publisher_node.py` (for simulated/test observations)
- `go1_inference_node.py` (performs model inference)
- `go1_driver_node.py` or `go1_sdk_bridge_node.py` (for actuation)

---

## 🔁 Sim2Real Tips

- Enable domain randomization in config to bridge simulation and reality.
- Use `residual_control` to stabilize learned policy with PD baseline.
- Use `COM visualizer` to debug walking drift due to weight imbalance.

---

## ✅ Summary

You now have a full pipeline to retrain and deploy a walking controller for Go1 using reinforcement learning and real-world integration via ROS2.

Feel free to modify terrain, gait style, or reward shaping as needed!
