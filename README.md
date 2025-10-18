# arduinobot_ws
🤖 3-DOF Robotic Manipulator Simulation using ROS 2 Humble
This project demonstrates the simulation of a 3-DOF robotic manipulator using the Robot Operating System 2 (ROS 2) Humble framework.
It integrates multiple key ROS 2 tools and libraries to provide a realistic representation of robotic motion, control, and sensor feedback — both visually and physically — in simulation.

🧩 Simulation Overview

The simulation environment combines the following core components:

🧱 RViz 2 – For 3D visualization of the manipulator’s structure, joint movements, and coordinate frames.

🌍 Gazebo Classic – Acts as the physics engine simulating realistic motion, gravity, and environment interaction.

📐 URDF (Unified Robot Description Format) – Defines the robot’s geometry, kinematic structure, and joint configuration.

⚙️ ROS 2 Control – Manages and configures the controllers that actuate the robot’s joints.

🧭 TF2 – Maintains and transforms coordinate frames between robot parts for consistent spatial awareness.

🦾 MoveIt 2 – Provides motion planning, trajectory execution, and collision-free path generation.

Together, these modules create a complete simulation pipeline where the manipulator’s behavior can be visualized in RViz and physically simulated in Gazebo, demonstrating how robotic systems are modeled, controlled, and visualized in ROS 2.

🔑 Key Features

🦾 3-DOF Robotic Arm – Modeled in URDF for accurate mechanical structure and motion.

🔄 Full ROS 2 Integration – Developed using nodes, launch files, and controller configurations.

🌐 Physics-Based Simulation – Realistic motion using Gazebo’s physics engine (gravity, inertia, and collisions).

👀 Visualization with RViz 2 – Displays real-time link motion, joint states, and TF frames.

🎮 Controller Configuration – Implemented with ros2_control and ros2_controllers for position and state feedback.

🧭 TF2 Transform Tree – Ensures accurate spatial relationships among robot links.

🚀 MoveIt 2 Integration – Enables motion planning and execution of precise trajectories.

📦 Package Structure

🚀 arduinobot_bringup: Contains launch files that start all functionalities of the real or simulated robot.

🎮 arduinobot_controller: Contains the ROS 2 Control configuration and hardware interface with the real robot.

📎 arduinobot_cpp_examples: Provides ROS 2 templates and examples for C++ development.

📒 arduinobot_description: Contains the URDF description, meshes, and Gazebo simulation setup.

🔋 arduinobot_firmware: Includes Arduino firmware for actuating the motors of the real robot.

🧠 arduinobot_moveit: Holds the MoveIt 2 configuration and launch files for motion planning.

📧 arduinobot_msgs: Defines new ROS 2 custom message interfaces.

🧩 arduinobot_py_examples: Provides ROS 2 templates and examples for Python development.

🗣️ arduinobot_remote: Enables remote control of the robot using Amazon Alexa voice commands.

⚒️ arduinobot_utils: Contains utility tools and helper scripts for setup and maintenance.

🧰 Setup & Installation
1. Prerequisites

Ensure the following software is installed:

🐧 Ubuntu 22.04 LTS

🦋 ROS 2 Humble

🌍 Gazebo Classic

🦾 MoveIt 2

2. Clone and Build
   # Create a new ROS 2 workspace
mkdir -p ~/arduinobot_ws/src
cd ~/arduinobot_ws/src

# Clone the repository
git clone https://github.com/<your-username>/arduinobot.git

# Build the workspace
cd ~/arduinobot_ws
colcon build

# Source the setup file
source install/setup.bash

🧠 Author

Abhishek S. Jumale


