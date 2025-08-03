# Custom DWA Local Planner for TurtleBot3 in Gazebo (ROS 2 Humble)

![ROS Humble](https://img.shields.io/badge/ROS-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blueviolet)
![License](https://img.shields.io/badge/License-MIT-green)

This repository contains a **custom Dynamic Window Approach (DWA) local planner** implementation for a differential-drive robot (TurtleBot3) in ROS 2 Humble. It includes a full simulation in **Gazebo**, obstacle avoidance using **LaserScan**, goal-driven velocity sampling, and trajectory visualization in **RViz**.

---

## 🚀 Features

- ✅ Implements DWA algorithm from scratch in Python
- ✅ Obstacle avoidance using `/scan` topic
- ✅ Goal-oriented trajectory sampling
- ✅ Smoothness and velocity cost evaluation
- ✅ Marker-based trajectory visualization in RViz
- ✅ Simulated TurtleBot3 in a custom Gazebo world
- ✅ Compatible with SLAM and Nav2 parameters

---

## 📁 Repository Structure
src/
├── custom_dwa/               # Python-based custom DWA planner
│   ├── launch/
│   │   └── dwa_planner_launch.py
│   ├── custom_dwa/
│   │   ├── dwa_planner_node.py
│   │   └── init.py
│   ├── rviz_config/
│   │   └── rviz_config.rviz
│   ├── package.xml
│   └── setup.py
│
├── my_worlds/                # Gazebo world, SLAM, and Nav2 launch configs
│   ├── launch/
│   │   ├── bringup_tb3_custom.launch.py
│   │   └── slam_tb3.launch.py
│   ├── worlds/
│   │   └── simple.world
│   ├── maps/
│   │   ├── my_map.pgm
│   │   └── my_map.yaml
│   ├── params/
│   │   └── nav2_params.yaml
│   └── ...
│
├── turtlebot3/               # ↓ Required official packages (clone manually)
├── turtlebot3_msgs/          # ↓
└── turtlebot3_simulations/   # ↓
