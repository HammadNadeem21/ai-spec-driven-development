# Data Model: Digital Twin Module (Gazebo & Unity)

**Design Phase**: Phase 1 of 3 | **Status**: Complete | **Date**: 2025-12-20
**Input**: Feature specification from `/specs/002-digital-twin-gazebo-unity/spec.md`

## Core Entities

### Digital Twin
- **Definition**: A high-fidelity virtual representation of a physical humanoid robot that simulates physics, environment, and sensor behavior
- **Attributes**:
  - `id`: Unique identifier for the digital twin instance
  - `name`: Human-readable name for the twin
  - `robot_model`: Reference to the URDF model used
  - `physics_config`: Configuration for Gazebo physics simulation
  - `visualization_config`: Configuration for Unity visualization
  - `sensor_config`: Configuration for sensor simulation
  - `state`: Current simulation state (paused, running, stopped)
- **Relationships**: Contains one Robot Model, one Physics Simulation, one Visualization Environment, and multiple Sensor Simulations

### Physics Simulation (Gazebo)
- **Definition**: Component that models gravity, collisions, and dynamics to accurately represent real-world robot behavior
- **Attributes**:
  - `gravity`: Gravity vector (x, y, z)
  - `collision_detection`: Collision detection algorithm used
  - `dynamics_solver`: Physics engine solver settings
  - `simulation_step`: Time step for physics calculations
  - `real_time_factor`: Real-time simulation factor
- **Relationships**: Belongs to one Digital Twin, connects to ROS 2 via topics

### Visualization Environment (Unity)
- **Definition**: Visual rendering system that provides realistic display of robot movements and environmental interactions
- **Attributes**:
  - `render_quality`: Quality level for rendering
  - `environment`: Scene/environment configuration
  - `lighting`: Lighting setup for the scene
  - `camera_config`: Camera positioning and settings
  - `sync_rate`: Synchronization rate with physics simulation
- **Relationships**: Belongs to one Digital Twin, connects to ROS 2 via topics

### Sensor Simulation
- **Definition**: Component that generates realistic sensor data (LiDAR, depth cameras, IMUs) for AI training and testing
- **Attributes**:
  - `type`: Sensor type (LiDAR, camera, IMU, etc.)
  - `position`: Position on the robot model
  - `orientation`: Orientation relative to robot frame
  - `parameters`: Sensor-specific parameters (range, resolution, etc.)
  - `topic_name`: ROS 2 topic name for data output
- **Relationships**: Belongs to one Digital Twin, outputs to ROS 2 topics

### Simulation Pipeline
- **Definition**: The data flow system that connects physics simulation, visualization, and sensor data with ROS 2
- **Attributes**:
  - `sync_strategy`: Strategy for synchronizing components
  - `data_flow`: Direction and timing of data flow
  - `error_handling`: Error handling and recovery mechanisms
- **Relationships**: Connects Physics Simulation, Visualization Environment, and Sensor Simulations to ROS 2

## Data Flow Architecture

### Primary Data Flows
1. **Robot State Flow**: Robot model → Physics simulation → ROS 2 → Visualization
2. **Sensor Data Flow**: Physics simulation → Sensor simulation → ROS 2 topics
3. **Control Command Flow**: ROS 2 → Physics simulation → Robot model

### ROS 2 Topic Structure
```
/your_robot/
├── joint_states          # Robot joint positions from physics
├── tf                    # Transform frames
├── tf_static             # Static transform frames
├── laser_scan            # LiDAR sensor data
├── camera/               # Camera sensor data
│   ├── color/image_raw
│   └── depth/image_raw
└── imu/data              # IMU sensor data
```

## State Management

### Simulation States
- **IDLE**: Simulation loaded but not running
- **RUNNING**: Simulation actively computing physics and updates
- **PAUSED**: Simulation temporarily stopped
- **ERROR**: Simulation in error state requiring intervention

### State Transitions
- IDLE → RUNNING: When simulation start is requested
- RUNNING → PAUSED: When pause is requested
- PAUSED → RUNNING: When resume is requested
- RUNNING/PAUSED → IDLE: When simulation is stopped

## Configuration Schema

### Digital Twin Configuration (YAML)
```yaml
digital_twin:
  name: "humanoid_digital_twin"
  robot_model: "path/to/robot.urdf"
  physics:
    gravity: [0.0, 0.0, -9.81]
    real_time_factor: 1.0
    step_size: 0.001
  visualization:
    render_quality: "high"
    sync_rate: 60
  sensors:
    - type: "lidar"
      topic: "/robot/laser_scan"
      position: [0.5, 0.0, 0.8]
    - type: "imu"
      topic: "/robot/imu/data"
      position: [0.0, 0.0, 1.0]
```

### Sensor Configuration Schema
```yaml
sensors:
  lidar:
    type: "ray"
    topic: "/scan"
    frame_id: "laser_frame"
    range:
      min: 0.1
      max: 30.0
      resolution: 0.01
    scan:
      horizontal:
        samples: 720
        resolution: 0.5
        min_angle: -2.356194
        max_angle: 2.356194
  camera:
    type: "camera"
    topic: "/camera/color/image_raw"
    frame_id: "camera_frame"
    image:
      width: 640
      height: 480
      format: "R8G8B8"
```

## Integration Points

### ROS 2 Interface
- **Node Name**: `digital_twin_bridge`
- **Services**:
  - `/digital_twin/start`
  - `/digital_twin/stop`
  - `/digital_twin/pause`
  - `/digital_twin/reset`
- **Parameters**:
  - `use_sim_time`: Enable simulation time
  - `physics_rate`: Physics update rate
  - `visualization_rate`: Visualization update rate

### Unity-ROS Bridge
- **Connection Type**: TCP/IP socket
- **Default Port**: 10000
- **Message Types**: Custom messages for state synchronization
- **Transforms**: Coordinate frame conversion between Unity and ROS

## Validation Rules

### Data Integrity
- All URDF references must exist and be valid
- Sensor configurations must match available simulation capabilities
- Physics parameters must be within realistic bounds
- Topic names must follow ROS naming conventions

### Consistency Requirements
- Physics simulation and visualization must maintain consistent timing
- Sensor data must reflect current physics state
- Transform frames must be consistent across all components
- Simulation state must be synchronized across all components