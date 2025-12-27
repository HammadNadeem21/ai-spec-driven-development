---
sidebar_position: 4
---

# Chapter 3: Humanoid Modeling with URDF

## Overview

This chapter covers URDF (Unified Robot Description Format) for modeling humanoid robots. You'll learn about the purpose and structure of URDF, including links, joints, coordinate frames, and how to integrate these models with ROS 2 and simulators.

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its kinematic structure, inertial properties, and visual appearance.

### Purpose of URDF

URDF serves several important purposes in robotics:

1. **Robot Description**: Defines the physical structure of a robot
2. **Kinematic Chain**: Establishes relationships between different parts of the robot
3. **Simulation**: Provides models for robot simulators like Gazebo
4. **Visualization**: Enables visualization tools like RViz to display the robot
5. **Motion Planning**: Allows motion planning algorithms to understand robot structure

### URDF Structure

A URDF file is an XML document with the following main elements:

- `<robot>`: Root element containing the entire robot description
- `<link>`: Represents a rigid body part of the robot
- `<joint>`: Defines the connection between two links
- `<material>`: Defines visual materials (color, texture)
- `<gazebo>`: Simulator-specific extensions (optional)

## Links: The Building Blocks

A **link** represents a rigid body part of the robot. Each link can have:

- Visual properties (shape, color, mesh)
- Collision properties (shape for collision detection)
- Inertial properties (mass, center of mass, inertia tensor)

### Link Structure

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
      <!-- or <cylinder radius="0.5" length="1"/> -->
      <!-- or <sphere radius="0.5"/> -->
      <!-- or <mesh filename="package://path/to/mesh.stl"/> -->
    </geometry>
    <material name="color_name">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
  </inertial>
</link>
```

## Joints: Connecting the Parts

A **joint** defines how two links are connected. Joints specify:

- The type of connection (revolute, continuous, prismatic, etc.)
- The axis of motion
- Limits on motion (for revolute joints)
- The transformation between links

### Joint Types

1. **Fixed**: No movement between links
2. **Revolute**: Rotational movement with limits
3. **Continuous**: Rotational movement without limits
4. **Prismatic**: Linear movement with limits
5. **Floating**: 6 degrees of freedom
6. **Planar**: Motion in a plane

### Joint Structure

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- For revolute and prismatic joints -->
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>

  <!-- For continuous joints, no limits needed -->
</joint>
```

## Coordinate Frames

URDF uses coordinate frames to define the position and orientation of links relative to each other. The standard is the right-hand rule with:

- X: Forward
- Y: Left
- Z: Up

### Transformation

Transformations between frames are defined by:
- **xyz**: Translation vector (x, y, z position)
- **rpy**: Rotation vector (roll, pitch, yaw angles in radians)

## Complete URDF Example: Simple Humanoid Robot

Here's a complete example of a simple humanoid robot with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin_color">
        <color rgba="0.8 0.6 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left arm) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.25 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

## URDF Integration with ROS 2

### Loading URDF in ROS 2

URDF files can be loaded in ROS 2 using the `robot_state_publisher` package, which publishes the robot's joint states and transforms to the TF tree.

```xml
<!-- Launch file example -->
<launch>
  <param name="robot_description" command="xacro $(find-pkg-share my_robot_description)/urdf/robot.urdf.xacro" />
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>
</launch>
```

### URDF with Xacro

For complex robots, Xacro (XML Macros) is often used to simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.14159"/>

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${side == 'left' and 0.2 or -0.2} 0 0.6" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left"/>
  <xacro:arm side="right"/>
</robot>
```

## Integration with Simulators

### Gazebo Integration

To integrate URDF with Gazebo, add Gazebo-specific extensions:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- For adding plugins -->
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint_name</joint_name>
  </plugin>
</gazebo>
```

### RViz Visualization

RViz can visualize URDF models by subscribing to the TF tree published by `robot_state_publisher`. The robot will appear in the 3D view with all its links and joints properly positioned.

## Best Practices for URDF Modeling

### Structure and Organization

1. **Use consistent naming**: Follow a clear naming convention for links and joints
2. **Start simple**: Begin with a basic model and add complexity gradually
3. **Validate regularly**: Use URDF validation tools during development
4. **Document well**: Add comments to explain complex parts of the model

### Physical Accuracy

1. **Realistic inertial properties**: Use proper mass and inertia values
2. **Accurate dimensions**: Base dimensions on actual robot measurements
3. **Appropriate joint limits**: Set realistic limits based on physical constraints
4. **Collision vs. visual**: Use simplified geometry for collision detection

### Common Pitfalls to Avoid

1. **Floating joints**: Ensure all joints are properly connected to the robot tree
2. **Missing inertial properties**: Every link should have inertial properties defined
3. **Incorrect coordinate frames**: Follow the ROS coordinate frame conventions
4. **Complex meshes**: Simplify collision meshes for better performance

## Practical Examples

Now that you understand the concepts, here are practical examples of integrating URDF models with ROS 2:

### Loading URDF in ROS 2

The `robot_state_publisher` package publishes the state of the robot to the TF tree based on joint positions. Here's a basic launch file example:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ])
    ])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    # Joint State Publisher (for visualization)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="both",
    )

    # RViz2 node
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'rviz',
            'view_robot.rviz'
        ])],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])
```

More complete examples, including Gazebo integration and validation tools, are available in the [URDF Integration Examples](./urdf-integration-examples.md) section.

## Practical Exercise

Now that you understand the concepts, create a URDF model for a simple humanoid robot:

1. Define at least 10 links (torso, head, arms, legs)
2. Connect the links with appropriate joints
3. Add visual and collision properties
4. Validate your URDF model
5. Load the model in RViz to visualize it

Detailed implementation instructions are available in the [Practical Exercises](./practical-exercises.md) section.

## Summary

This chapter covered:
- The purpose and structure of URDF for robot modeling
- Links and joints as the fundamental building blocks
- Coordinate frames and transformations
- Complete example of a humanoid robot model
- Integration with ROS 2 and simulators
- Best practices for effective URDF modeling

URDF is essential for representing humanoid robots in ROS 2 systems, enabling simulation, visualization, and motion planning.

## References

For more detailed information about URDF modeling, refer to the official ROS 2 documentation:
- [URDF/XML Format](https://docs.ros.org/en/humble/p/urdf/)
- [Xacro Package](https://docs.ros.org/en/humble/p/xacro/)
- [Robot Modeling Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF.html)

## Next Steps

With the fundamentals of ROS 2, Python AI agents, and URDF modeling covered, you now have the knowledge to create comprehensive robotic systems that integrate AI with physical humanoid robots.