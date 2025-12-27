---
title: Navigation with Nav2 for Humanoids
sidebar_label: Chapter 3 - Nav2 for Humanoids
---

# Navigation with Nav2 for Humanoids

## Introduction to Nav2 Architecture

The Navigation 2 (Nav2) stack is ROS 2's official navigation framework that provides a complete solution for robot path planning, obstacle avoidance, and navigation execution. For humanoid robots, Nav2 requires specific adaptations to handle the unique challenges of bipedal locomotion, balance requirements, and dynamic movement patterns.

### Core Components of Nav2

1. **Global Planner**: Computes optimal paths from start to goal
2. **Local Planner**: Executes short-term motion while avoiding obstacles
3. **Costmap 2D**: Maintains obstacle and cost information
4. **Recovery Behaviors**: Handles navigation failures and stuck situations
5. **Lifecycle Manager**: Controls the state transitions of navigation components

### Nav2 for Humanoid-Specific Navigation

Traditional Nav2 was designed primarily for wheeled robots with simple kinematic constraints. Humanoid robots require:

- **Balance-aware path planning**: Consideration of center of mass and stability
- **Dynamic obstacle avoidance**: Adaptation to bipedal locomotion patterns
- **Step-aware navigation**: Understanding of foot placement and stepping
- **Recovery for bipedal systems**: Specialized recovery behaviors for humanoid robots

## Nav2 Behavior Trees

### Understanding Behavior Trees in Nav2

Behavior trees provide a flexible and modular approach to organizing navigation tasks. In Nav2, behavior trees replace the monolithic navigation architecture with a more flexible, composable system.

### Key Behavior Tree Concepts

1. **Nodes**: Individual actions or conditions
2. **Composites**: Combine multiple nodes (sequences, selectors)
3. **Decorators**: Modify node behavior (inverter, repeater)
4. **Conditions**: Boolean checks that return success/failure

### Humanoid-Specific Behavior Tree Design

For humanoid robots, navigation behavior trees must include:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <GoalReached/>
            <Fallback>
                <RecoveryNode number_of_retries="2">
                    <PipelineSequence>
                        <ComputePathToPose goal="{goal}" path="{path}"/>
                        <SmoothPath input_path="{path}" output_path="{path}"/>
                        <FollowPath path="{path}" velocity="{velocity}"/>
                    </PipelineSequence>
                    <ReactiveFallback>
                        <AvoidanceController velocity="{velocity}" goal="{goal}" path="{path}"/>
                        <IsGoalReached goal="{goal}" tolerance="0.5"/>
                    </ReactiveFallback>
                </RecoveryNode>
                <ComputePathToPose goal="{goal}" path="{path}"/>
                <SmoothPath input_path="{path}" output_path="{path}"/>
                <FollowPath path="{path}" velocity="{velocity}"/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
```

### Custom Humanoid Behaviors

Humanoid robots need specialized behaviors:

1. **BalanceCheck**: Verify stability before movement
2. **StepPlanner**: Plan individual steps for bipedal locomotion
3. **FootPlacement**: Optimize foot positions for stability
4. **PostureAdjust**: Adjust body posture during navigation

## Path Planning Adapted for Bipedal Humanoids

### Humanoid Kinematic Constraints

Bipedal humanoid robots have unique kinematic constraints that affect path planning:

- **Step Size Limits**: Maximum distance between consecutive steps
- **Balance Envelope**: Maintain center of mass within support polygon
- **Turning Radius**: Limited by leg length and hip joint constraints
- **Obstacle Clearance**: Need for sufficient space for leg swing

### Costmap Adaptations

The costmap for humanoid robots must consider:

1. **Stepable Terrain**: Identify surfaces suitable for foot placement
2. **Stability Zones**: Mark areas where robot can maintain balance
3. **Obstacle Heights**: Consider obstacles that affect leg swing
4. **Ground Properties**: Surface type affects traction and foot placement

### Custom Path Planners for Humanoids

Traditional path planners (A*, Dijkstra, etc.) need adaptation:

1. **Step-Constrained Planning**: Ensure paths respect step size limits
2. **Balance-Aware Planning**: Plan paths that maintain center of mass stability
3. **Dynamic Planning**: Adapt to changing balance constraints during movement

### Implementation Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
import math

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Subscribe to costmap and robot pose
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.costmap_callback, 10)

        self.robot_pose_subscription = self.create_subscription(
            PoseStamped, '/robot_pose',
            self.pose_callback, 10)

        # Publisher for humanoid-adapted path
        self.path_publisher = self.create_publisher(
            Path, '/humanoid/global_plan', 10)

        # Humanoid-specific parameters
        self.step_size_limit = 0.3  # meters
        self.balance_margin = 0.1   # meters
        self.foot_separation = 0.2  # meters

    def plan_path(self, start_pose, goal_pose):
        """Plan a path considering humanoid kinematic constraints"""
        # Implement humanoid-specific path planning algorithm
        # This would consider step size limits and balance constraints
        path = Path()
        path.header.frame_id = "map"

        # Generate waypoints respecting humanoid constraints
        current_pose = start_pose
        while not self.at_goal(current_pose, goal_pose):
            next_waypoint = self.calculate_next_waypoint(
                current_pose, goal_pose)

            # Verify step is within humanoid constraints
            if self.is_valid_humanoid_step(current_pose, next_waypoint):
                pose_stamped = PoseStamped()
                pose_stamped.pose = next_waypoint
                path.poses.append(pose_stamped)
                current_pose = next_waypoint
            else:
                # Find alternative path
                break

        return path

    def is_valid_humanoid_step(self, current_pose, next_pose):
        """Check if step is valid for humanoid robot"""
        # Calculate step distance
        dx = next_pose.position.x - current_pose.position.x
        dy = next_pose.position.y - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if within step size limit
        if distance > self.step_size_limit:
            return False

        # Check if step maintains balance
        # (This would involve more complex balance calculations)

        return True

    def calculate_next_waypoint(self, current_pose, goal_pose):
        """Calculate next waypoint respecting humanoid constraints"""
        # Implement humanoid-aware waypoint calculation
        # Consider step size, balance, and obstacle avoidance
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Obstacle Avoidance for Humanoid Robots

### Challenges in Humanoid Obstacle Avoidance

Humanoid robots face unique challenges in obstacle avoidance:

- **Dynamic Balance**: Avoiding obstacles while maintaining balance
- **Foot Placement**: Finding safe foot placement locations
- **Leg Swing Space**: Ensuring obstacles don't interfere with leg movement
- **Turning Mechanics**: Different turning mechanics compared to wheeled robots

### Local Planner Adaptations

The local planner for humanoid robots must consider:

1. **Step-Aware Local Planning**: Plan immediate steps rather than continuous motion
2. **Balance Preservation**: Maintain stability during obstacle avoidance
3. **Leg Swing Planning**: Account for leg swing trajectory during obstacle avoidance
4. **Reactive Behaviors**: Quick responses to sudden obstacles

### Implementation Strategies

1. **Sampling-Based Approaches**: Sample valid foot placements in local area
2. **Optimization-Based**: Optimize for both obstacle avoidance and balance
3. **Learning-Based**: Use machine learning to adapt to different scenarios

### Humanoid-Specific Obstacle Avoidance Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import TransformListener, Buffer
import numpy as np

class HumanoidObstacleAvoider(Node):
    def __init__(self):
        super().__init__('humanoid_obstacle_avoider')

        # Subscribe to sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        self.robot_pose_subscription = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/humanoid/cmd_vel', 10)

        # Humanoid-specific parameters
        self.min_obstacle_distance = 0.5  # meters
        self.step_width = 0.2            # meters (foot width)
        self.leg_swing_radius = 0.4      # meters

    def laser_callback(self, msg):
        """Process laser scan data for obstacle avoidance"""
        # Convert laser scan to points in robot frame
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter valid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        valid_angles = angles[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Find closest obstacles in critical directions
        front_obstacles = self.get_front_obstacles(valid_ranges, valid_angles)
        side_obstacles = self.get_side_obstacles(valid_ranges, valid_angles)

        # Generate avoidance commands considering humanoid constraints
        cmd_vel = self.generate_avoidance_command(front_obstacles, side_obstacles)

        self.cmd_vel_publisher.publish(cmd_vel)

    def get_front_obstacles(self, ranges, angles):
        """Get obstacles in front of robot within leg swing radius"""
        # Consider obstacles in front within leg swing range
        front_mask = (angles > -np.pi/4) & (angles < np.pi/4)
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        # Filter obstacles within leg swing radius
        close_mask = front_ranges < self.leg_swing_radius
        close_ranges = front_ranges[close_mask]
        close_angles = front_angles[close_mask]

        return list(zip(close_ranges, close_angles))

    def generate_avoidance_command(self, front_obstacles, side_obstacles):
        """Generate velocity command for obstacle avoidance"""
        cmd = Twist()

        if front_obstacles:
            # Adjust for humanoid-specific constraints
            min_distance = min([r for r, a in front_obstacles])

            if min_distance < self.min_obstacle_distance:
                # Stop or move laterally based on humanoid capabilities
                cmd.linear.x = 0.0
                # Calculate lateral movement considering balance
                cmd.linear.y = self.calculate_lateral_avoidance(front_obstacles)
            else:
                # Move forward with caution
                cmd.linear.x = 0.2  # Conservative speed for humanoid
        else:
            # Clear path, move forward
            cmd.linear.x = 0.4

        return cmd

    def calculate_lateral_avoidance(self, obstacles):
        """Calculate lateral movement considering humanoid balance"""
        # Implement humanoid-aware lateral avoidance
        # Consider foot placement and balance constraints
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Adapting Nav2 Concepts for Bipedal Humanoids

### Configuration Files for Humanoid Navigation

Nav2 configuration for humanoid robots requires specialized parameter files:

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    set_initial_pose: false
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_footprint"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific behavior tree
    plugin_lib_names: ["humanoid_navigate_to_pose"]

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controllers
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid FollowPath controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIC"
      debug_multithreading: false
      rate: 20.0
      transform_tolerance: 0.1
      look ahead distance: 0.6
      minimum look ahead distance: 0.3
      lateral_iterations: 3
      longitudinal_iterations: 3
      control_horizon: 10
      heading_threshold: 0.2
      # Humanoid-specific parameters
      max_linear_speed: 0.4  # Conservative for balance
      min_linear_speed: 0.05
      max_angular_speed: 0.5
      min_angular_speed: 0.1

local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: "odom"
    robot_base_frame: "base_footprint"
    use_sim_time: True
    rolling_window: true
    width: 6
    height: 6
    resolution: 0.05
    # Humanoid-specific costmap settings
    footprint: "[ [0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2] ]"
    footprint_padding: 0.01
    inflation_radius: 0.55  # Account for leg swing
    cost_scaling_factor: 5.0
    map_type: "costmap"
    always_send_full_costmap: true

global_costmap:
  ros__parameters:
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: "map"
    robot_base_frame: "base_footprint"
    use_sim_time: True
    robot_radius: 0.3  # Account for humanoid width
    resolution: 0.05
    track_unknown_space: true
    # Humanoid-specific settings
    inflation_radius: 0.7  # Larger for humanoid navigation
    cost_scaling_factor: 3.0
    map_type: "costmap"
    always_send_full_costmap: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    # Humanoid-specific path planner
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size_limit: 0.3  # Max step size for humanoid
      balance_margin: 0.1   # Safety margin for balance
```

### Custom Plugins for Humanoid Navigation

Creating custom Nav2 plugins for humanoid-specific functionality:

1. **HumanoidPathPlanner**: Custom global planner considering step constraints
2. **BalanceController**: Local controller maintaining stability during navigation
3. **StepPlannerServer**: Specialized action server for humanoid navigation

### Launch File Configuration

```xml
<!-- humanoid_navigation.launch.py -->
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Parameters file
    params_file = os.path.join(
        get_package_share_directory('your_humanoid_package'),
        'config', 'humanoid_nav2_params.yaml'
    )

    # Navigation launch
    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file
        }.items()
    )

    # Humanoid-specific nodes
    humanoid_controller = Node(
        package='your_humanoid_package',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/humanoid/cmd_vel'),
            ('/odom', '/humanoid/odom')
        ]
    )

    return launch.LaunchDescription([
        navigation_launch,
        humanoid_controller
    ])
```

## Practical Exercise: Implementing Humanoid Navigation System

### Exercise Objective
Create a complete Nav2-based navigation system adapted for bipedal humanoid constraints, implementing custom path planning and obstacle avoidance.

### Prerequisites
- ROS 2 Humble with Nav2 installed
- Isaac ROS perception pipeline running
- Simulated or real humanoid robot
- Sensor data (LiDAR, IMU, cameras)

### Steps

1. **Configuration Setup**:
   - Create humanoid-specific Nav2 parameter files
   - Configure costmap for humanoid dimensions
   - Set up behavior tree for humanoid navigation

2. **Custom Plugin Development**:
   - Implement humanoid-aware path planner
   - Create balance-preserving local controller
   - Develop step-aware obstacle avoidance

3. **Integration Testing**:
   - Launch complete navigation stack
   - Test with simulated humanoid robot
   - Validate path planning and obstacle avoidance

4. **Performance Validation**:
   - Measure navigation success rates
   - Verify balance preservation during navigation
   - Test recovery behaviors for humanoid-specific failures

### Expected Results
- Complete Nav2 navigation system adapted for humanoid robots
- Successful path planning considering step constraints
- Stable obstacle avoidance maintaining balance
- High success rate in navigation tasks

## Troubleshooting Humanoid Navigation

### Common Issues and Solutions

1. **Balance Loss During Navigation**:
   - Problem: Robot loses balance while following paths
   - Solution: Adjust path smoothing and velocity profiles

2. **Step Planning Failures**:
   - Problem: Unable to find valid foot placements
   - Solution: Increase costmap resolution and adjust inflation

3. **Oscillation in Local Planning**:
   - Problem: Robot oscillates near obstacles
   - Solution: Tune local planner parameters and increase hysteresis

4. **Recovery Behavior Failures**:
   - Problem: Recovery behaviors don't work for humanoid
   - Solution: Implement humanoid-specific recovery actions

### Performance Monitoring

Monitor these key metrics for humanoid navigation:

- **Balance maintenance**: Percentage of time robot maintains stability
- **Path following accuracy**: Deviation from planned path
- **Obstacle avoidance success**: Successful avoidance rate
- **Navigation completion**: Goal reaching success rate
- **Computation time**: Real-time performance of navigation stack

## Summary

Navigation for humanoid robots requires significant adaptations to the standard Nav2 framework to account for bipedal locomotion, balance constraints, and unique kinematic properties. By implementing humanoid-aware path planning, balance-preserving local control, and step-aware obstacle avoidance, we can create robust navigation systems for humanoid robots.

The combination of Isaac Sim for training, Isaac ROS for perception, and adapted Nav2 for navigation creates a complete AI brain for humanoid robots, enabling them to perceive, plan, and navigate in complex environments while maintaining stability and safety.

This completes the three-chapter module on the NVIDIA Isaac AI Robot Brain, providing comprehensive coverage of simulation, perception, and navigation for humanoid robots using NVIDIA's Isaac ecosystem.