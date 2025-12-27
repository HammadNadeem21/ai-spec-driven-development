# Quickstart: NVIDIA Isaac AI Robot Brain

**Target**: Advanced AI and robotics students with ROS 2 experience
**Time**: 4-6 hours to complete setup and first simulation
**Prerequisites**: NVIDIA GPU (RTX 3080 or equivalent), ROS 2 Humble Hawksbill, Isaac Sim and Isaac ROS installed

## Setup Checklist

- [ ] NVIDIA GPU with CUDA support (RTX 3080 or better recommended)
- [ ] Install ROS 2 Humble Hawksbill
- [ ] Install Isaac Sim (Jetpack or standalone)
- [ ] Install Isaac ROS packages
- [ ] Install Nav2 packages
- [ ] Install Docusaurus prerequisites (Node.js 18+, npm/yarn)
- [ ] Clone/fork this repository
- [ ] Run local Docusaurus server to view documentation

## Environment Setup

### 1. System Requirements
```bash
# Ubuntu 22.04 LTS (recommended)
# NVIDIA GPU with RTX capabilities
# 16GB+ RAM, 50GB+ disk space for Isaac Sim
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

# Install Isaac Sim (requires NVIDIA developer account)
# Download from NVIDIA Developer website
# Follow installation instructions for your platform
```

### 3. Isaac ROS Installation
```bash
# Create ROS workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Install Isaac ROS packages via apt
sudo apt update
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-perception
sudo apt install ros-humble-isaac-ros-vslam
sudo apt install ros-humble-isaac-ros-visual-slam

# Or build from source (for latest features)
cd src
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_perception.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Install dependencies and build
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
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

## First Isaac Sim Experience

### 1. Launch Isaac Sim Environment
```bash
# Make sure Isaac Sim is properly installed
# Launch the application from the installed location
# Or use the command line interface if available
```

### 2. Load Humanoid Robot Model
```bash
# In Isaac Sim:
# - Create a new scene or load an existing environment
# - Import your humanoid robot model (URDF format)
# - Configure sensors (cameras, LiDAR, IMU)
# - Set up lighting and environmental conditions
```

### 3. Generate Synthetic Data
```bash
# Configure synthetic data generation pipeline
# Set output format and annotation requirements
# Run simulation to generate training data
# Verify data quality and format
```

## Isaac ROS Pipeline

### 1. Launch Perception Pipeline
```bash
# Terminal 1: Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Isaac ROS VSLAM pipeline
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py
```

### 2. Connect Isaac Sim Data
```bash
# In Isaac Sim, configure the ROS bridge to publish:
# - Camera data to /camera/color/image_raw
# - Depth data to /camera/depth/image_raw
# - IMU data to /imu/data
# - LiDAR data to /scan
```

### 3. Verify Perception Output
```bash
# In separate terminal
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Check available topics
ros2 topic list | grep -E "(slam|detection|perception)"

# View SLAM pose
ros2 topic echo /slam/pose

# View object detections
ros2 topic echo /detections
```

## Navigation with Nav2

### 1. Configure Humanoid-Specific Navigation
```bash
# Create custom Nav2 configuration for humanoid constraints
# Adjust costmap parameters for bipedal locomotion
# Configure recovery behaviors for balance maintenance
```

### 2. Launch Navigation Stack
```bash
# Terminal 1: Source environments
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Nav2 with Isaac ROS perception integration
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/humanoid_nav2_config.yaml
```

### 3. Test Navigation
```bash
# Use RViz2 to send navigation goals
# Monitor robot behavior and obstacle avoidance
# Verify humanoid-specific constraints are respected
```

## Troubleshooting

### Common Issues
- **Isaac Sim won't start**: Check NVIDIA GPU drivers and CUDA installation
- **Isaac ROS nodes fail**: Verify GPU compatibility and CUDA setup
- **Performance issues**: Reduce simulation quality settings
- **ROS bridge connection fails**: Check network connectivity and topic names

### Verification Commands
```bash
# Check Isaac ROS installation
ros2 pkg list | grep isaac

# Verify GPU acceleration
nvidia-smi

# Test ROS-Isaac integration
ros2 topic list
ros2 node list

# Check Isaac ROS pipeline status
ros2 run isaac_ros_visual_slam isaac_ros_visual_slam_node --ros-args --list-parameters
```

## Next Steps
1. Complete Chapter 1: NVIDIA Isaac Sim for Perception & Data
2. Proceed to Chapter 2: Isaac ROS & Hardware-Accelerated Perception
3. Explore Chapter 3: Navigation with Nav2 for Humanoids
4. Integrate all components in a complete AI brain workflow