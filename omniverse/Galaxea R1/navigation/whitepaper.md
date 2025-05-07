
# Galaxea R1 Navigation Whitepaper

This document outlines the design and structure of six navigation tasks supported by the Galaxea R1 training framework.

## Task Overview

### 1. PointGoal Navigation
Navigate from a start point to a relative direction/goal without a full map.

### 2. Map-Based Navigation
Global planning with full knowledge of the environment.

### 3. Exploration
Active SLAM-based exploration in unknown environments.

### 4. Semantic Goal Navigation
Navigate to objects described with semantic labels (e.g., "table").

### 5. Multi-Goal Navigation
Visit a sequence of goal points, optionally optimized for order.

### 6. Dynamic Obstacle Avoidance
Avoid moving obstacles while navigating to a target.

## Scene System

Each task uses its own `*_scene.py` module, supporting multiple styles:
- residential
- office
- warehouse
- hospital

The scene system is modular and uses `SceneFactory` to dynamically load environments.

## Reward Functions

Reward functions are designed specifically per task and are located in `reward_functions/`. These are simple Python functions that return scalar values used during PPO training.

## Training Configuration

All tasks can be configured via YAML files under `config/`, specifying the scene, style, and relevant training parameters.

## Future Work

This framework is designed to support sim-to-real transfer. Future extensions will include:
- curriculum learning across scene complexities
- hierarchical policy architectures
- integration with ROS2 and real-world deployment

