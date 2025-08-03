Custom DWA Local Planner for TurtleBot3 in Gazebo (ROS 2 Humble)
This repository contains a custom Dynamic Window Approach (DWA) local planner implementation for a differential-drive robot (TurtleBot3) in ROS 2 Humble. It includes a full simulation in Gazebo, obstacle avoidance using LaserScan, goal-driven velocity sampling, and trajectory visualization in RViz.

🚀 Features
✅ Implements DWA algorithm from scratch in Python

✅ Obstacle avoidance using /scan topic

✅ Goal-oriented trajectory sampling

✅ Smoothness and velocity cost evaluation

✅ Marker-based trajectory visualization in RViz

✅ Simulated TurtleBot3 in a custom Gazebo world

✅ Compatible with SLAM and Nav2 parameters

📁 Repository Structure
src/
├── custom_dwa/ # Python-based custom DWA planner
│   ├── launch/
│   │   └── dwa_planner_launch.py
│   ├── custom_dwa/
│   │   ├── dwa_planner_node.py
│   │   └── __init__.py
│   ├── rviz_config/
│   │   └── rviz_config.rviz
│   ├── package.xml
│   └── setup.py
│
├── my_worlds/ # Gazebo world, SLAM, and Nav2 launch configs
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
├── turtlebot3/             # ↓ Required official packages (clone manually)
├── turtlebot3_msgs/        # ↓
└── turtlebot3_simulations/ # ↓
🧩 Dependencies
ROS 2 Humble

Gazebo

Python 3

turtlebot3, turtlebot3_msgs, turtlebot3_simulations (cloned manually from source)

Install core ROS 2 dependencies:

Bash

sudo apt update && sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs
🔧 Setup Instructions
1. Clone the Repository
Create a workspace and clone this repository along with the required TurtleBot3 packages.

Bash

mkdir -p ~/const_ws/src
cd ~/const_ws/src

# Clone this repository
git clone https://github.com/ashtalekar36/dwa_custom_planner.git

# Clone required TurtleBot3 packages
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
2. Install ROS 2 Dependencies
Navigate to your workspace root and let rosdep install any remaining dependencies.

Bash

cd ~/const_ws
rosdep install --from-paths src --ignore-src -r -y
3. Build the Workspace
Build the packages using colcon and source the setup file.

Bash

colcon build --symlink-install
source install/setup.bash
(Optional) Add the source command to your .bashrc for convenience:

Bash

echo "source ~/const_ws/install/setup.bash" >> ~/.bashrc
4. Set TurtleBot3 Model
Export the TURTLEBOT3_MODEL environment variable. This project uses burger.

Bash

export TURTLEBOT3_MODEL=burger
(Optional) Add this to your .bashrc as well:

Bash

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
📡 Running the Simulation
Launch the entire simulation environment with a single command:

Bash

ros2 launch custom_dwa dwa_planner_launch.py
✅ This will:

Launch Gazebo with the custom world.

Start RViz with a pre-configured layout.

Spawn the TurtleBot3 in the simulation.

Run the custom DWA local planner node.

🕹️ Usage Instructions
Wait for Gazebo and RViz to fully load.

In the RViz window, use the "2D Nav Goal" tool from the top toolbar to click on a destination.

The robot will begin moving towards the goal.

The robot will sample velocity commands (v,
omega), evaluate trajectories to find an obstacle-free path, and publish the best command to /cmd_vel. It will stop upon reaching the goal or if it becomes blocked.

👁️ Visualization Tools
Monitor Velocity Commands: You can inspect the planner's output in a separate terminal:

Bash

ros2 topic echo /cmd_vel
RViz Markers: The planner publishes the sampled trajectories as a MarkerArray, which can be visualized in RViz to see how the robot is making its decisions.

📷 Screenshots
(Add screenshots or GIFs of your robot navigating in Gazebo and RViz here)

🙏 Acknowledgements
This project was inspired by the open-source work of El2ra2 — adapted, extended, and customized for use in TurtleBot3 simulation and academic use.

📄 License
This project is licensed under the MIT License. Feel free to fork and modify.

✍️ Author
Ashish S. Talekar

GitHub

Robotics & Automation | NVIDIA Isaac Sim | ROS 2 Developer
