# Unitree H1 ROS 2 SDK

This repository provides ROS 2 (Humble) support for the Unitree H1 Humanoid Robot, including URDF descriptions, Gazebo simulation, and visualization tools.

## Overview

The Unitree H1 is a high-performance humanoid robot. This SDK provides:

- **h1_description**: Robot URDF model, meshes, and visualization
- **h1_gazebo**: Gazebo simulation environment and controllers

## Requirements

### System Requirements
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble Hawksbill
- Gazebo Fortress (installed with ros-humble-ros-gz)

### Dependencies

Install the required dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-urdf \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-rviz2
```

## Installation

### 1. Create a ROS 2 Workspace (if you don't have one)

```bash
mkdir -p ~/h1_ws/src
cd ~/h1_ws/src
```

### 2. Clone this Repository

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
# Or copy the contents to your workspace
```

### 3. Build the Workspace

```bash
cd ~/h1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select h1_description h1_gazebo
source install/setup.bash
```

## Usage

### View H1 Robot in RViz2

Launch the robot visualization with joint state publisher GUI:

```bash
source /opt/ros/humble/setup.bash
source ~/h1_ws/install/setup.bash
ros2 launch h1_gazebo h1_rviz.launch.py
```

This will open RViz2 with the H1 robot model and a GUI to manipulate joint positions.

### Launch H1 in Gazebo Simulation

```bash
source /opt/ros/humble/setup.bash
source ~/h1_ws/install/setup.bash
ros2 launch h1_gazebo h1_gazebo.launch.py
```

#### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `rviz` | `true` | Launch RViz2 alongside Gazebo |
| `gui` | `true` | Show Gazebo GUI |
| `world` | `h1_world.sdf` | World file to load |

Example with custom arguments:
```bash
ros2 launch h1_gazebo h1_gazebo.launch.py rviz:=false
```

## Robot Description

### Joint Configuration

The H1 robot has 19 controllable joints:

**Legs (10 joints)**
- Left leg: `left_hip_yaw_joint`, `left_hip_roll_joint`, `left_hip_pitch_joint`, `left_knee_joint`, `left_ankle_joint`
- Right leg: `right_hip_yaw_joint`, `right_hip_roll_joint`, `right_hip_pitch_joint`, `right_knee_joint`, `right_ankle_joint`

**Torso (1 joint)**
- `torso_joint`

**Arms (8 joints)**
- Left arm: `left_shoulder_pitch_joint`, `left_shoulder_roll_joint`, `left_shoulder_yaw_joint`, `left_elbow_joint`
- Right arm: `right_shoulder_pitch_joint`, `right_shoulder_roll_joint`, `right_shoulder_yaw_joint`, `right_elbow_joint`

### Sensors

- **IMU**: Located on the torso (`imu_link`)
- **Cameras**: D435 RGB-D camera, Mid360 LiDAR reference frames

## Package Structure

```
unitree-h1-ros-sdk/
├── src/
│   ├── robots/
│   │   └── h1_description/       # Robot description package
│   │       ├── urdf/             # URDF files
│   │       ├── meshes/           # 3D mesh files (DAE, STL)
│   │       ├── launch/           # Launch files
│   │       └── rviz/             # RViz configuration
│   └── h1_gazebo/                # Gazebo simulation package
│       ├── launch/               # Simulation launch files
│       ├── config/               # Controller configurations
│       ├── worlds/               # Gazebo world files
│       └── urdf/                 # Gazebo-specific URDF extensions
├── README.md
└── LICENSE
```

## ROS 2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | Current joint positions and velocities |
| `/robot_description` | `std_msgs/msg/String` | URDF robot description |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform tree |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU sensor data (simulation) |

## Troubleshooting

### Common Issues

1. **Missing mesh files**: Ensure `h1_description` package is properly built and sourced
2. **Gazebo not starting**: Check if Gazebo Fortress is installed: `gz sim --version`
3. **RViz model not showing**: Verify the robot_description topic is published: `ros2 topic echo /robot_description`

### Debug Commands

```bash
# Check if packages are built
ros2 pkg list | grep h1

# Check robot description
ros2 topic echo /robot_description --once

# List available transforms
ros2 run tf2_ros tf2_echo pelvis left_ankle_link
```

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## References

- [Unitree Robotics](https://www.unitree.com/)
- [Unitree ROS Repository](https://github.com/unitreerobotics/unitree_ros)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress)
