# dwa_custom_planner
to implement a custom Dynamic Window Approach (DWA) local planner for a TurtleBot in Gazebo using ROS2 Humble. The candidate should code the planner from scratch (without using nav2_dwb_controller)




# 🧭 Custom DWA Local Planner for TurtleBot3 (ROS 2 Humble)

**Author:** Ashish S. Talekar  
**Email:** ashishtalekar36@gmail.com  
**Robot:** TurtleBot3  
**Platform:** ROS 2 Humble, Gazebo, RViz  
**Assignment:** Implement a Custom Local Planner from Scratch

---

## 📌 Overview

This package implements a **custom Dynamic Window Approach (DWA)** local planner **from scratch**, designed for TurtleBot3 simulation in Gazebo using ROS 2 Humble. The planner subscribes to `/odom`, `/scan`, and `/goal_pose`, and publishes velocity commands to `/cmd_vel`.

It evaluates sampled velocity commands based on:
- ✅ Distance to the goal
- ✅ Obstacle avoidance (via LaserScan)
- ✅ Speed (for efficiency)
- ✅ Smoothness (yaw rate change)

---

## 🛠️ Features

- [x] Velocity sampling within dynamic constraints
- [x] Predictive trajectory simulation
- [x] Cost function-based scoring
- [x] Obstacle collision checking
- [x] Visualization of best trajectory using `MarkerArray` in RViz
- [x] Goal handling and stop behavior
- [x] Debug logs with robot state, costs, and decisions

---

## 🔧 Dependencies

Ensure you have the following packages:
- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `visualization_msgs`
- `tf_transformations`
- `numpy`

---

## 📂 Files

| File | Description |
|------|-------------|
| `dwa_planner_node.py` | Main ROS 2 node with full DWA logic |
| `launch/bringup_tb3_custom.launch.py` | Launches Gazebo world, RViz, and the DWA node |
| `rviz/dwa.rviz` | Preconfigured RViz layout to visualize planner |
| `worlds/` | Custom Gazebo world files (if applicable) |
| `README.md` | You’re reading it :) |

---

## 🚀 How to Run

1. **Build the workspace**:
   ```bash
   colcon build --packages-select custom_dwa
   source install/setup.bash
