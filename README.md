# Arduinobot — ROS 2 Manipulator Simulation & Voice Control

![CI](https://github.com/Abhishek-Jumale/arduinobot_ws/actions/workflows/ci.yml/badge.svg)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Enabled-green)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

A 3-DOF robotic manipulator simulation built with ROS 2 Humble, featuring MoveIt 2 motion planning, Gazebo physics simulation, and Amazon Alexa voice control for pick-and-place automation.

---

## Architecture

```
                    ┌──────────────────┐
                    │   Alexa Voice    │
                    │    Commands      │
                    └────────┬─────────┘
                             │
                             ▼
┌──────────┐    ┌──────────────────────┐    ┌──────────────┐
│  RViz 2  │◄───│   ROS 2 Humble       │───►│   Gazebo     │
│  (Viz)   │    │                      │    │  (Physics)   │
└──────────┘    │  ┌────────────────┐  │    └──────────────┘
                │  │  MoveIt 2      │  │
                │  │  (Planning)    │  │    ┌──────────────┐
                │  └────────────────┘  │───►│   Arduino    │
                │  ┌────────────────┐  │    │  (Hardware)  │
                │  │  ros2_control  │  │    └──────────────┘
                │  │  (Controllers) │  │
                │  └────────────────┘  │
                │  ┌────────────────┐  │
                │  │  TF2           │  │
                │  │  (Transforms)  │  │
                │  └────────────────┘  │
                └──────────────────────┘
```

## Key Features

- **3-DOF Robotic Arm** — URDF-modeled manipulator with accurate kinematics and collision geometry
- **MoveIt 2 Motion Planning** — Collision-free trajectory generation and execution
- **Gazebo Physics Simulation** — Realistic gravity, inertia, and contact dynamics
- **RViz 2 Visualization** — Real-time display of joint states, TF frames, and planned paths
- **ros2_control Integration** — Position controllers with joint state feedback
- **Alexa Voice Control** — Remote pick-and-place commands via Amazon Alexa
- **Dual Language** — Custom ROS 2 action interfaces implemented in both Python and C++
- **Hardware Bridge** — Arduino serial communication for real robot actuation
- **CI/CD Pipeline** — GitHub Actions with flake8 and cpplint on every push

## Package Structure

```
arduinobot_ws/
├── arduinobot_bringup         # Launch files for simulation and real robot
├── arduinobot_controller      # ros2_control config and hardware interface
├── arduinobot_description     # URDF, meshes, and Gazebo world files
├── arduinobot_moveit          # MoveIt 2 configuration and launch files
├── arduinobot_msgs            # Custom ROS 2 message/action definitions
├── arduinobot_remote          # Alexa voice control integration
├── arduinobot_firmware        # Arduino firmware for motor actuation
├── arduinobot_cpp_examples    # C++ ROS 2 node examples
├── arduinobot_py_examples     # Python ROS 2 node examples
└── arduinobot_utils           # Utility scripts and helpers
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic
- MoveIt 2

## Installation

```bash
# Create workspace
mkdir -p ~/arduinobot_ws/src
cd ~/arduinobot_ws/src

# Clone
git clone https://github.com/Abhishek-Jumale/arduinobot_ws.git .

# Install dependencies
cd ~/arduinobot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Usage

**Launch simulation (Gazebo + RViz):**
```bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

**Launch MoveIt 2 motion planning:**
```bash
ros2 launch arduinobot_moveit moveit.launch.py
```

**Launch Alexa voice control:**
```bash
ros2 launch arduinobot_remote remote.launch.py
```

## Technologies

| Component | Technology |
|-----------|-----------|
| Middleware | ROS 2 Humble |
| Simulation | Gazebo Classic |
| Motion Planning | MoveIt 2 |
| Visualization | RViz 2 |
| Robot Model | URDF / Xacro |
| Controllers | ros2_control |
| Transforms | TF2 |
| Hardware | Arduino (Serial) |
| Voice Control | Amazon Alexa |
| Languages | Python 3, C++17 |
| CI/CD | GitHub Actions |

## Author

**Abhishek Surendra Jumale**

- GitHub: [github.com/Abhishek-Jumale](https://github.com/Abhishek-Jumale)
- LinkedIn: [linkedin.com/in/abhishek-jumale](https://linkedin.com/in/abhishek-jumale)
