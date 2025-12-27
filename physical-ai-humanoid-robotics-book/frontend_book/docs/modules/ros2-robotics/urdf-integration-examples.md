# URDF Integration Examples

## Overview

This document provides practical examples of integrating URDF models with ROS 2 and simulators like Gazebo.

## Loading URDF in ROS 2

### Using robot_state_publisher

The `robot_state_publisher` package publishes the state of the robot to the TF tree based on joint positions.

**Launch file example (launch/robot.launch.py):**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description",
            default_value="",
            description="Robot description in URDF or XACRO format",
        )
    )

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

    # Joint State Publisher GUI (optional, for manual joint control)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
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

    return LaunchDescription(
        declared_arguments +
        [
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )
```

### URDF with Xacro

Xacro (XML Macros) allows you to create parameterized URDF files:

**Example URDF with Xacro (urdf/robot.urdf.xacro):**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.3"/>
  <xacro:property name="base_height" value="1.0"/>

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
      <material name="white"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side position_x">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="blue"/>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.15"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${position_x} 0 ${base_height*0.6}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
    </joint>

    <link name="${side}_lower_arm">
      <visual>
        <geometry>
          <cylinder radius="0.04" length="0.3"/>
        </geometry>
        <material name="blue"/>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="0.04" length="0.3"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.8"/>
        <origin xyz="0 0 -0.15"/>
        <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the arm macro -->
  <xacro:arm side="left" position_x="${base_width/2}"/>
  <xacro:arm side="right" position_x="${-base_width/2}"/>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 ${base_height}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="5" velocity="1"/>
  </joint>

</robot>
```

## Integration with Gazebo

### Adding Gazebo-Specific Elements

To make your URDF work with Gazebo, add Gazebo-specific tags:

**URDF with Gazebo extensions (urdf/robot.gazebo.xacro):**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot_gazebo">

  <!-- Include the basic robot definition -->
  <xacro:include filename="$(find my_robot_description)/urdf/robot.urdf.xacro"/>

  <!-- Gazebo Materials -->
  <gazebo reference="base_link">
    <material>Gazebo/White</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Gazebo Plugins -->
  <!-- Joint state publisher for Gazebo -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <joint_name>neck_joint</joint_name>
      <joint_name>left_shoulder_joint</joint_name>
      <joint_name>left_elbow_joint</joint_name>
      <joint_name>right_shoulder_joint</joint_name>
      <joint_name>right_elbow_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Diff drive controller for base movement (if applicable) -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.15</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Position controllers for arm joints -->
  <gazebo>
    <plugin name="position_controller" filename="libgazebo_ros_control.so">
      <robotNamespace>/robot</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

### Gazebo Launch File

**Launch file for simulation (launch/simulate.launch.py):**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'robot.gazebo.xacro'
        ])
    ])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": use_sim_time},
        ],
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            ])
        ]),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzclient.launch.py"
            ])
        ]),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "humanoid_robot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ],
        output="screen",
    )

    return LaunchDescription([
        *declared_arguments,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
    ])
```

## Validation Tools

### Checking URDF Validity

Use these command-line tools to validate your URDF:

```bash
# Check if URDF is valid
check_urdf $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/robot.urdf.xacro

# Or using xacro directly
ros2 run xacro xacro $(find-pkg-share my_robot_description)/urdf/robot.urdf.xacro

# View the robot in RViz
ros2 launch my_robot_description robot.launch.py
```

### Creating a URDF Validation Script

**Python script for URDF validation (scripts/validate_urdf.py):**

```python
#!/usr/bin/env python3

import sys
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def validate_urdf(urdf_file_path):
    """
    Validate a URDF file for basic syntax and structure
    """
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_file_path)
        root = tree.getroot()

        # Check if it's a valid robot file
        if root.tag != 'robot':
            print(f"ERROR: Root element is not 'robot', got '{root.tag}'")
            return False

        robot_name = root.get('name')
        if not robot_name:
            print("ERROR: Robot element missing 'name' attribute")
            return False

        print(f"✓ Robot name: {robot_name}")

        # Count links and joints
        links = root.findall('link')
        joints = root.findall('joint')

        print(f"✓ Found {len(links)} links")
        print(f"✓ Found {len(joints)} joints")

        # Check for common issues
        link_names = [link.get('name') for link in links]
        joint_names = [joint.get('name') for joint in joints]

        # Check for duplicate names
        if len(link_names) != len(set(link_names)):
            print("ERROR: Duplicate link names found")
            return False

        if len(joint_names) != len(set(joint_names)):
            print("ERROR: Duplicate joint names found")
            return False

        # Check joint parent-child relationships
        for joint in joints:
            parent = joint.find('parent')
            child = joint.find('child')

            if parent is not None and child is not None:
                parent_link = parent.get('link')
                child_link = child.get('link')

                if parent_link not in link_names:
                    print(f"ERROR: Joint '{joint.get('name')}' references non-existent parent link '{parent_link}'")
                    return False

                if child_link not in link_names:
                    print(f"ERROR: Joint '{joint.get('name')}' references non-existent child link '{child_link}'")
                    return False

        print("✓ URDF structure validation passed")
        return True

    except ET.ParseError as e:
        print(f"ERROR: Invalid XML syntax - {e}")
        return False
    except FileNotFoundError:
        print(f"ERROR: File not found - {urdf_file_path}")
        return False
    except Exception as e:
        print(f"ERROR: Unexpected error - {e}")
        return False


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 validate_urdf.py <urdf_file_path>")
        sys.exit(1)

    urdf_file = sys.argv[1]
    print(f"Validating URDF file: {urdf_file}")
    print("-" * 40)

    if validate_urdf(urdf_file):
        print("-" * 40)
        print("SUCCESS: URDF validation passed!")
        return 0
    else:
        print("-" * 40)
        print("FAILURE: URDF validation failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main())
```

## Best Practices

### URDF Organization

1. **Use Xacro**: Makes complex URDFs more manageable
2. **Modular design**: Break robot into logical components
3. **Parameterization**: Use properties for dimensions that might change
4. **Clear naming**: Use consistent, descriptive names
5. **Documentation**: Comment complex sections

### Performance Considerations

1. **Simplified collision geometry**: Use simple shapes for collision detection
2. **Appropriate mesh resolution**: Balance detail with performance
3. **Realistic inertial properties**: Use actual physical values
4. **Efficient joint limits**: Set realistic constraints

These examples demonstrate how to properly structure URDF files for use with ROS 2 and simulators, ensuring your humanoid robot models work correctly in both visualization and simulation environments.