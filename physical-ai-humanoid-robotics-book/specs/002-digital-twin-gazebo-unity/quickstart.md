# Quickstart: Digital Twin Module (Gazebo & Unity)

**Target**: AI and robotics students with basic programming knowledge
**Time**: 2-4 hours to complete setup and first simulation
**Prerequisites**: Basic understanding of robotics concepts, familiarity with command line

## Setup Checklist

- [ ] Install ROS 2 (Humble Hawksbill or later)
- [ ] Install Gazebo Garden or Fortress
- [ ] Install Unity Hub and Unity 2021.3 LTS or later
- [ ] Install Docusaurus prerequisites (Node.js 18+, npm/yarn)
- [ ] Clone/fork this repository
- [ ] Run local Docusaurus server to view documentation

## Environment Setup

### 1. System Requirements
```bash
# Ubuntu 22.04 LTS (recommended)
# 8GB+ RAM, 20GB+ disk space
# Multi-core processor for simulation performance
```

### 2. Core Dependencies
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
source /opt/ros/humble/setup.bash

# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install gazebo
```

### 3. Unity Installation
```bash
# Download Unity Hub from Unity website
# Install Unity 2021.3 LTS or later through Unity Hub
# Install packages: ROS TCP Connector, URDF Importer
```

### 4. Project Setup
```bash
# Navigate to project directory
cd ~/GitHub/ai-spec-driven-dev/hackathon-ai-spec-driven-dev/physical-ai-humanoid-robotics-book

# Install dependencies
cd frontend_book
npm install

# Start local development server
npm start
```

## First Simulation

### 1. Launch Gazebo Environment
```bash
# Terminal 1: Source ROS 2
source /opt/ros/humble/setup.bash
cd ~/humanoid_robot_ws  # Your robot workspace
source install/setup.bash

# Launch basic humanoid model in Gazebo
ros2 launch your_robot_gazebo your_robot_world.launch.py
```

### 2. Connect Unity Visualization
```bash
# Open Unity project
# Configure ROS TCP Connector settings
# Connect to localhost:10000 (default port)
```

### 3. Verify Sensor Simulation
```bash
# In separate terminal
source /opt/ros/humble/setup.bash
cd ~/humanoid_robot_ws
source install/setup.bash

# Check available topics
ros2 topic list | grep -E "(scan|camera|imu)"

# View sensor data
ros2 topic echo /your_robot/laser_scan
ros2 topic echo /your_robot/camera/depth/image_raw
```

## Troubleshooting

### Common Issues
- **Gazebo won't start**: Check if X11 forwarding is enabled if running in container/VM
- **Unity connection fails**: Verify ROS TCP Connector settings and network connectivity
- **Sensor topics not publishing**: Check robot state publisher and sensor plugin configurations

### Verification Commands
```bash
# Check ROS 2 installation
ros2 topic list
ros2 node list

# Verify Gazebo simulation
gz sim -v 3  # Should show version info

# Test ROS-Unity bridge
ros2 run demo_nodes_cpp talker
# Should see messages published
```

## Next Steps
1. Complete Chapter 1: Physics Simulation with Gazebo
2. Proceed to Chapter 2: Environment & Interaction in Unity
3. Explore Chapter 3: Sensor Simulation
4. Integrate all components in a complete digital twin workflow