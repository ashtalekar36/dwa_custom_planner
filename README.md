# 🛠️ Custom DWA Local Planner for TurtleBot3 in Gazebo (ROS 2 Humble)

This repository contains a **custom Dynamic Window Approach (DWA) local planner** implementation for a differential-drive robot (TurtleBot3) in ROS 2 Humble. It includes full simulation in **Gazebo**, obstacle avoidance using **LaserScan**, goal-driven velocity sampling, and trajectory visualization in **RViz**.

---

## 📁 Repository Structure

src/
├── custom_dwa/ # Python-based custom DWA planner
│ ├── launch/
│ │ └── dwa_planner_launch.py
│ ├── custom_dwa/
│ │ ├── dwa_planner_node.py
│ │ └── init.py
│ ├── rviz_config/
│ │ └── rviz_config.rviz
│ ├── package.xml, setup.py, ...
│
├── my_worlds/ # Gazebo world, SLAM, and Nav2 launch configs
│ ├── launch/
│ │ ├── bringup_tb3_custom.launch.py
│ │ └── slam_tb3.launch.py
│ ├── worlds/
│ │ └── simple.world
│ ├── maps/
│ │ ├── my_map.pgm
│ │ └── my_map.yaml
│ ├── params/
│ │ └── nav2_params.yaml
│ └── ...
│
├── turtlebot3/ # ↓ Required official packages (clone manually)
├── turtlebot3_msgs/ # ↓
└── turtlebot3_simulations/ # ↓



## 🚀 Features

- ✅ Implements DWA algorithm from scratch in Python
- ✅ Obstacle avoidance using `/scan`
- ✅ Goal-oriented trajectory sampling
- ✅ Smoothness and velocity cost evaluation
- ✅ Marker-based trajectory visualization in RViz
- ✅ Simulated TurtleBot3 in custom Gazebo world
- ✅ Compatible with SLAM and Nav2 parameters

---

## 🧩 Dependencies

Requires:
- ROS 2 Humble
- Gazebo
- Python 3
- `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations` (cloned manually)

Install dependencies:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs```
🔧 Setup Instructions
1. Clone the Repository
bash
Copy
Edit
mkdir -p ~/const_ws/src
cd ~/const_ws/src

# Clone this repository
git clone https://github.com/ashtalekar36/dwa_custom_planner.git

# Clone required TurtleBot3 packages
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
2. Install ROS 2 Dependencies
bash
Copy
Edit
cd ~/const_ws
rosdep install --from-paths src --ignore-src -r -y
3. Build the Workspace
bash
Copy
Edit
colcon build --symlink-install
source install/setup.bash
(Optionally add to .bashrc):

bash
Copy
Edit
echo "source ~/const_ws/install/setup.bash" >> ~/.bashrc
4. Set TurtleBot3 Model
bash
Copy
Edit
export TURTLEBOT3_MODEL=burger
(Optionally add to .bashrc):

bash
Copy
Edit
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
📡 Running the Simulation
Launch everything — Gazebo, RViz, robot, and planner:

bash
Copy
Edit
ros2 launch custom_dwa dwa_planner_launch.py
✅ This will:

Launch Gazebo with a custom world

Start RViz with a preconfigured layout

Spawn the TurtleBot3 in simulation

Run your custom DWA local planner node

🕹️ Usage Instructions
In RViz, use the "2D Nav Goal" tool to click on a goal location

The robot will:

Sample velocity commands (v, w)

Evaluate trajectories for obstacle-free motion

Send /cmd_vel based on best trajectory

Stop near the goal or if blocked by obstacles

👁️ Visualization Tools
/cmd_vel output can be echoed:

bash
Copy
Edit
ros2 topic echo /cmd_vel
Planned paths are published as RViz MarkerArray

📷 Screenshots (Optional)
(Add screenshots or GIFs of your robot navigating in Gazebo and RViz here)

🙏 Acknowledgements
This project was inspired by the open-source work of El2ra2 — adapted, extended, and customized for use in TurtleBot3 simulation and academic use.

📄 License
MIT License — feel free to fork and modify.

✍️ Author
Ashish S. Talekar
GitHub
Robotics & Automation | NVIDIA Isaac Sim | ROS 2 Developer

yaml
Copy
Edit

---

Let me know if you also want to include a short **demo video**, GIFs, or automatic SLAM navigation instructions.
