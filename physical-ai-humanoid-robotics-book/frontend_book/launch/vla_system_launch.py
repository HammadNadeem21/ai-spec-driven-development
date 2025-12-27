from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_robot',
            description='Name of the robot'
        ),

        # Voice Processing Node
        Node(
            package='vla_system',
            executable='voice_processor',
            name='voice_processor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
            output='screen'
        ),

        # Cognitive Planning Node
        Node(
            package='vla_system',
            executable='cognitive_planner',
            name='cognitive_planner',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
            output='screen'
        ),

        # VLA Orchestrator Node
        Node(
            package='vla_system',
            executable='vla_orchestrator',
            name='vla_orchestrator',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
            output='screen'
        ),

        # Action Executor Node
        Node(
            package='vla_system',
            executable='action_executor',
            name='action_executor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
            output='screen'
        ),

        # Safety Validator Node
        Node(
            package='vla_system',
            executable='safety_validator',
            name='safety_validator',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
            output='screen'
        )
    ])