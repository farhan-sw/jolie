# Jolie - Omnidirectional Robot System

An autonomous omnidirectional robot system built with ROS2 Humble, featuring SLAM mapping, autonomous navigation, and real-time localization capabilities.

## 📖 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [API Reference](#api-reference)
- [Contributing](#contributing)

## 🔎 Overview

Jolie is a comprehensive robotics platform designed for omnidirectional movement with advanced autonomous capabilities. The system supports both simulation and real-world deployment, making it perfect for research, education, and practical applications.

The robot features:
- **Omnidirectional Movement**: 3-wheel and 4-wheel drive configurations
- **SLAM Mapping**: Real-time simultaneous localization and mapping
- **Autonomous Navigation**: Path planning and obstacle avoidance
- **Sensor Integration**: LIDAR, IMU, and Kinect support
- **Simulation Ready**: Full Gazebo integration

## ✨ Features

### Robot Configurations
- **Nebula 3WD**: Three-wheel omnidirectional drive
- **Nebula 4WD**: Four-wheel omnidirectional drive
- Configurable sensor payloads (LIDAR, IMU, Kinect)

### Navigation & Localization
- SLAM mapping with real-time visualization
- AMCL-based localization
- Nav2 integration for autonomous navigation
- Dynamic obstacle avoidance

### Hardware Integration
- ESP32-based micro-ROS communication
- Real-time sensor data fusion
- Hardware abstraction layer

### Simulation Support
- Full Gazebo simulation environment
- Multiple world configurations
- Hardware-in-the-loop testing

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Hardware      │    │   ROS2 Stack    │    │   Applications  │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │ ESP32 Board │ │◄──►│ │ Hardware    │ │◄──►│ │ Navigation  │ │
│ └─────────────┘ │    │ │ Interface   │ │    │ │ Stack       │ │
│ ┌─────────────┐ │    │ └─────────────┘ │    │ └─────────────┘ │
│ │ LIDAR       │ │◄──►│ ┌─────────────┐ │    │ ┌─────────────┐ │
│ └─────────────┘ │    │ │ Controller  │ │◄──►│ │ SLAM        │ │
│ ┌─────────────┐ │    │ │ Manager     │ │    │ │ Mapping     │ │
│ │ IMU Sensor  │ │◄──►│ └─────────────┘ │    │ └─────────────┘ │
│ └─────────────┘ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ ┌─────────────┐ │    │ │ Sensor      │ │◄──►│ │ Localization│ │
│ │ Kinect      │ │◄──►│ │ Drivers     │ │    │ │ System      │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🛠️ Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Python**: 3.8+
- **Memory**: 4GB RAM minimum, 8GB recommended
- **Storage**: 10GB free space

### Required Software
```bash
# ROS2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

### Additional Dependencies
```bash
# Navigation and SLAM tools
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-imu-tools

# Simulation
sudo apt install ros-humble-gazebo-ros-pkgs

# Development tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-pip
```

## 📦 Installation

### 1. Clone the Repository
```bash
cd ~/Documents/GitHub  # or your preferred directory
git clone https://github.com/farhan-sw/jolie.git
cd jolie
```

### 2. Setup ROS2 Workspace
```bash
cd jolie_humble
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
colcon build
source install/setup.bash
```

### 4. Install micro-ROS Agent (for real robot)
```bash
# Install Docker (if not already installed)
sudo apt install docker.io
sudo usermod -aG docker $USER
# Log out and log back in

# Pull micro-ROS agent
docker pull microros/micro-ros-agent:humble
```

## 🔧 Hardware Setup

### ESP32 Configuration
1. **Flash the ESP32 firmware** from `jolie_platformio/esp32s3_base/`
2. **Connect the ESP32** via USB to your computer
3. **Verify connection**: Check that device appears as `/dev/ttyACM0` or `/dev/ttyUSB0`

### Sensor Connections
- **LIDAR**: Connect via USB (typically `/dev/ttyUSB0`)
- **IMU**: Connect via serial interface
- **Kinect**: USB 3.0 connection required

### USB Device Permissions
If you encounter USB access issues:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Fix brltty conflicts (common issue)
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
```

For detailed USB troubleshooting, see `jolie_log/imu.md`.

### Kinect Setup
For Xbox Kinect installation, follow the detailed guide in `jolie_log/KINECT.md`.

### SSH Connection to Robot
For remote operation and development on the physical robot:

```bash
# Connect to robot with X11 forwarding for GUI applications
ssh -XC ubuntu@<robot_ip_address>

# Default credentials:
# Username: ubuntu
# Password: ubuntu
```

**SSH Connection Options:**
- `-X`: Enable X11 forwarding for GUI applications (RViz, Gazebo, etc.)
- `-C`: Enable compression for better performance over slower connections

**Common use cases:**
```bash
# Run RViz remotely on the robot
ssh -XC ubuntu@192.168.1.100
rviz2

# Execute commands remotely
ssh ubuntu@192.168.1.100 "cd ~/Documents/Github/jolie/jolie_humble && source install/setup.bash && ros2 topic list"

# Copy files to/from robot
scp my_map.yaml ubuntu@192.168.1.100:~/Documents/Github/jolie/jolie_humble/
scp ubuntu@192.168.1.100:~/Documents/Github/jolie/jolie_humble/log/* ./local_logs/
```

**Network Configuration:**
- Ensure your computer and robot are on the same network
- Check robot's IP address: `hostname -I` or `ip addr show`
- Test connectivity: `ping <robot_ip_address>`

## 🚀 Quick Start

### Simulation Mode
Start the robot in Gazebo simulation:
```bash
# Terminal 1: Source the workspace
cd ~/Documents/GitHub/jolie/jolie_humble
source install/setup.bash

# Launch simulation
ros2 launch jolie_bringup simulated_robot.launch.py
```

### Real Robot Mode
For physical robot operation:
```bash
# Terminal 1: Start micro-ROS agent
docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

# Terminal 2: Launch real robot
cd ~/Documents/GitHub/jolie/jolie_humble
source install/setup.bash
ros2 launch jolie_bringup real_robot.launch.py
```

## 📋 Usage

### 1. SLAM Mapping
Create a new map of your environment:
```bash
# Start the robot (simulation or real)
ros2 launch jolie_bringup real_robot.launch.py

# Start local localization (required for all operations)
ros2 launch jolie_localization local_localization.launch.py

# Start SLAM
ros2 launch jolie_mapping slam.launch.py

# Control the robot to explore (optional)
# Option 1: Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Option 2: Joystick teleop (plug in a USB joystick for manual control)
ros2 launch jolie_bringup pilot.launch.py

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 2. Localization
Use an existing map for localization:
```bash
# Start local localization (required for all operations)
ros2 launch jolie_localization local_localization.launch.py

# Global localization (initial pose estimation)
ros2 launch jolie_localization global_localization.launch.py map_name:=my_map
```

### 3. Autonomous Navigation
Navigate to goals autonomously:
```bash
# Start the robot (simulation or real)
ros2 launch jolie_bringup real_robot.launch.py

# Start local localization (required for all operations)
ros2 launch jolie_localization local_localization.launch.py

# Load a map for navigation (use existing map)
ros2 launch jolie_localization global_localization.launch.py map_name:=my_map

# Start navigation stack
ros2 launch jolie_navigation navigation.launch.py

# Send navigation goals via RViz or command line
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

### 4. Monitoring and Visualization
```bash
# Start local localization (required for all operations)
ros2 launch jolie_localization local_localization.launch.py

# Launch RViz for visualization
rviz2 -d $(ros2 pkg prefix jolie_description)/share/jolie_description/rviz/robot.rviz

# Monitor robot status
ros2 topic echo /robot_state
ros2 topic list
ros2 node list
```

## 🔧 Troubleshooting

### Common Issues

#### USB Device Not Found
```bash
# Check connected devices
ls /dev/tty*

# Check dmesg for connection issues
dmesg | grep tty

# Fix permissions
sudo chmod 666 /dev/ttyACM0  # or your device
```

#### micro-ROS Connection Failed
```bash
# Try different baud rates
docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

# Check ESP32 serial output
screen /dev/ttyACM0 115200
```

#### Build Errors
```bash
# Clean and rebuild
cd ~/Documents/GitHub/jolie/jolie_humble
rm -rf build/ install/ log/
colcon build

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### Navigation Issues
```bash
# Check transforms
ros2 run tf2_tools view_frames

# Verify map publication
ros2 topic echo /map --once

# Check localization
ros2 topic echo /amcl_pose
```

## 👨‍💻 Development

### Project Structure
```
jolie_humble/src/
├── jolie_bringup/          # Launch configurations
├── jolie_description/      # Robot URDF/meshes
├── jolie_firmware/         # Hardware interface
├── jolie_localization/     # Localization nodes
├── jolie_mapping/          # SLAM configuration
├── jolie_navigation/       # Navigation stack
├── jolie_utility/          # Utility nodes (C++)
├── jolie_utility_python/   # Utility nodes (Python)
├── sirius_controller/      # Motion controller
└── third_party/           # External packages
```

### Adding New Features

#### 1. Create a New Package
```bash
cd ~/Documents/GitHub/jolie/jolie_humble/src
ros2 pkg create --build-type ament_cmake my_new_package
```

#### 2. Add Launch Files
```python
# In launch/my_feature.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_new_package',
            executable='my_node',
            name='my_node',
            parameters=[{'param1': 'value1'}]
        )
    ])
```

#### 3. Integration with Main System
Add your launch file to the main robot launch files in `jolie_bringup/launch/`.

### Code Style Guidelines
- Follow ROS2 naming conventions
- Use meaningful variable and function names
- Add comprehensive documentation
- Include unit tests where applicable

### Testing
```bash
# Build and test
colcon build --packages-select my_new_package
colcon test --packages-select my_new_package

# Run specific tests
ros2 run my_new_package test_executable
```

## 📚 API Reference

### Key Topics

#### Sensor Data
- `/scan` (sensor_msgs/LaserScan) - LIDAR data
- `/imu/data` (sensor_msgs/Imu) - IMU measurements
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed

#### Control
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/odom` (nav_msgs/Odometry) - Odometry data

#### Navigation
- `/map` (nav_msgs/OccupancyGrid) - Map data
- `/goal_pose` (geometry_msgs/PoseStamped) - Navigation goals
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Robot pose

### Key Services
- `/global_localization` - Reset localization
- `/clear_costmaps` - Clear navigation costmaps

### Parameters
Major configurable parameters can be found in:
- `jolie_localization/config/amcl.yaml`
- `jolie_navigation/config/nav2_params.yaml`
- `jolie_mapping/config/slam_params.yaml`

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Test your changes in both simulation and real hardware (if available)
- Update documentation for new features
- Follow ROS2 best practices
- Add appropriate error handling

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

For questions and support:
- **Issues**: GitHub Issues
- **Email**: farhansw.off@gmail.com
- **Documentation**: See `jolie_log/` directory for development logs

---