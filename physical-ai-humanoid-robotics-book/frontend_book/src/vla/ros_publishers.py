"""
ROS 2 Publishers for Vision-Language-Action (VLA) System

This module handles publishing of VLA data to ROS 2 topics.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from typing import Dict, Any, Optional
from .models.voice_command import VoiceCommand
from .models.task_plan import TaskPlan
from .models.action_sequence import ActionSequence
from .models.simulation_state import SimulationState
import json
import logging

logger = logging.getLogger(__name__)


class VLAPublisher(Node):
    """
    ROS 2 publisher node for VLA system components
    """

    def __init__(self, node_name: str = 'vla_publisher'):
        """
        Initialize the VLA publisher node

        Args:
            node_name: Name of the ROS 2 node (default: 'vla_publisher')
        """
        super().__init__(node_name)

        # Publishers for different VLA components
        self.voice_command_publisher = self.create_publisher(
            String,
            'vla/voice_commands',
            10
        )

        self.task_plan_publisher = self.create_publisher(
            String,
            'vla/task_plans',
            10
        )

        self.action_status_publisher = self.create_publisher(
            String,
            'vla/action_status',
            10
        )

        self.simulation_state_publisher = self.create_publisher(
            String,
            'vla/simulation_state',
            10
        )

        self.execution_status_publisher = self.create_publisher(
            String,
            'vla/execution_status',
            10
        )

        self.audio_data_publisher = self.create_publisher(
            AudioData,
            'vla/audio_data',
            10
        )

        self.confidence_publisher = self.create_publisher(
            Float32,
            'vla/transcription_confidence',
            10
        )

        # Store active plans for tracking
        self.active_plans = {}

        logger.info(f"Initialized VLA publisher node: {node_name}")

    def publish_voice_command(self, voice_command: VoiceCommand) -> None:
        """
        Publish a voice command to the appropriate ROS 2 topic

        Args:
            voice_command: VoiceCommand object to publish
        """
        try:
            # Create message with command data
            msg = String()
            msg.data = json.dumps({
                "id": voice_command.id,
                "transcript": voice_command.transcript,
                "confidence": voice_command.confidence,
                "timestamp": voice_command.timestamp,
                "language": voice_command.language,
                "audio_data_ref": voice_command.audio_data
            })

            self.voice_command_publisher.publish(msg)
            self.get_logger().info(f'Published voice command: {voice_command.transcript[:50]}...')

        except Exception as e:
            self.get_logger().error(f'Error publishing voice command: {e}')
            raise

    def publish_task_plan(self, task_plan: TaskPlan) -> None:
        """
        Publish a task plan to the appropriate ROS 2 topic

        Args:
            task_plan: TaskPlan object to publish
        """
        try:
            # Create message with plan data
            plan_data = {
                "id": task_plan.id,
                "original_command": task_plan.original_command,
                "status": task_plan.status,
                "created_at": task_plan.created_at,
                "completed_at": task_plan.completed_at,
                "actions": [
                    {
                        "id": action.id,
                        "type": action.type,
                        "parameters": action.parameters,
                        "dependencies": action.dependencies,
                        "timeout": action.timeout,
                        "priority": action.priority,
                        "status": action.status
                    }
                    for action in task_plan.actions
                ],
                "constraints": [
                    {
                        "id": constraint.id,
                        "type": constraint.type,
                        "condition": constraint.condition,
                        "severity": constraint.severity,
                        "description": constraint.description
                    }
                    for constraint in task_plan.constraints
                ]
            }

            msg = String()
            msg.data = json.dumps(plan_data)

            self.task_plan_publisher.publish(msg)
            self.get_logger().info(f'Published task plan {task_plan.id} with {len(task_plan.actions)} actions')

            # Track the plan
            self.active_plans[task_plan.id] = task_plan

        except Exception as e:
            self.get_logger().error(f'Error publishing task plan: {e}')
            raise

    def publish_action_status(self, action_id: str, status: str, plan_id: Optional[str] = None,
                            result: Optional[Dict[str, Any]] = None) -> None:
        """
        Publish action status to the appropriate ROS 2 topic

        Args:
            action_id: ID of the action
            status: Status of the action (pending, executing, completed, failed)
            plan_id: ID of the parent plan (optional)
            result: Result data from action execution (optional)
        """
        try:
            status_data = {
                "action_id": action_id,
                "status": status,
                "timestamp": rclpy.clock.Clock().now().to_msg().sec,
                "plan_id": plan_id,
                "result": result or {}
            }

            msg = String()
            msg.data = json.dumps(status_data)

            self.action_status_publisher.publish(msg)
            self.get_logger().info(f'Published action status: {action_id} -> {status}')

        except Exception as e:
            self.get_logger().error(f'Error publishing action status: {e}')
            raise

    def publish_simulation_state(self, simulation_state: SimulationState) -> None:
        """
        Publish simulation state to the appropriate ROS 2 topic

        Args:
            simulation_state: SimulationState object to publish
        """
        try:
            state_data = {
                "id": simulation_state.id,
                "robot_pose": simulation_state.robot_pose,
                "environment_objects": simulation_state.environment_objects,
                "sensor_data": simulation_state.sensor_data,
                "timestamp": simulation_state.timestamp
            }

            msg = String()
            msg.data = json.dumps(state_data)

            self.simulation_state_publisher.publish(msg)
            self.get_logger().info(f'Published simulation state: {simulation_state.id}')

        except Exception as e:
            self.get_logger().error(f'Error publishing simulation state: {e}')
            raise

    def publish_execution_status(self, plan_id: str, status: str, message: str = "") -> None:
        """
        Publish execution status to the appropriate ROS 2 topic

        Args:
            plan_id: ID of the plan being executed
            status: Execution status
            message: Additional status message (optional)
        """
        try:
            status_data = {
                "plan_id": plan_id,
                "status": status,
                "message": message,
                "timestamp": rclpy.clock.Clock().now().to_msg().sec
            }

            msg = String()
            msg.data = json.dumps(status_data)

            self.execution_status_publisher.publish(msg)
            self.get_logger().info(f'Published execution status: {plan_id} -> {status}')

        except Exception as e:
            self.get_logger().error(f'Error publishing execution status: {e}')
            raise

    def publish_audio_data(self, audio_data: bytes) -> None:
        """
        Publish raw audio data to the appropriate ROS 2 topic

        Args:
            audio_data: Raw audio data to publish
        """
        try:
            msg = AudioData()
            msg.data = audio_data

            self.audio_data_publisher.publish(msg)
            self.get_logger().info(f'Published audio data: {len(audio_data)} bytes')

        except Exception as e:
            self.get_logger().error(f'Error publishing audio data: {e}')
            raise

    def publish_confidence(self, confidence: float) -> None:
        """
        Publish transcription confidence to the appropriate ROS 2 topic

        Args:
            confidence: Confidence value to publish (0.0 to 1.0)
        """
        try:
            msg = Float32()
            msg.data = float(confidence)

            self.confidence_publisher.publish(msg)
            self.get_logger().info(f'Published confidence: {confidence}')

        except Exception as e:
            self.get_logger().error(f'Error publishing confidence: {e}')
            raise

    def update_active_plan(self, plan_id: str, action_id: str, new_status: str) -> bool:
        """
        Update the status of an action in an active plan

        Args:
            plan_id: ID of the plan
            action_id: ID of the action to update
            new_status: New status for the action

        Returns:
            True if update was successful, False otherwise
        """
        if plan_id not in self.active_plans:
            self.get_logger().warning(f'Plan {plan_id} not found in active plans')
            return False

        plan = self.active_plans[plan_id]

        # Find and update the action
        action_updated = False
        for action in plan.actions:
            if action.id == action_id:
                action.status = new_status
                action_updated = True
                break

        if action_updated:
            # Check if the entire plan status needs updating
            if plan.is_complete():
                plan.status = "completed"
                plan.completed_at = rclpy.clock.Clock().now().to_msg().sec
                # Remove from active plans when completed
                del self.active_plans[plan_id]

        return action_updated

    def get_active_plan_status(self, plan_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the status of an active plan

        Args:
            plan_id: ID of the plan to check

        Returns:
            Dictionary with plan status, or None if plan not found
        """
        if plan_id not in self.active_plans:
            return None

        plan = self.active_plans[plan_id]
        completed_actions = sum(1 for action in plan.actions if action.status == "completed")
        total_actions = len(plan.actions)

        return {
            "plan_id": plan.id,
            "status": plan.status,
            "original_command": plan.original_command,
            "total_actions": total_actions,
            "completed_actions": completed_actions,
            "progress_percentage": (completed_actions / total_actions) * 100 if total_actions > 0 else 0,
            "created_at": plan.created_at,
            "completed_at": plan.completed_at
        }

    def cancel_active_plan(self, plan_id: str) -> bool:
        """
        Cancel an active plan

        Args:
            plan_id: ID of the plan to cancel

        Returns:
            True if plan was successfully canceled, False otherwise
        """
        if plan_id not in self.active_plans:
            return False

        plan = self.active_plans[plan_id]
        plan.status = "failed"
        plan.completed_at = rclpy.clock.Clock().now().to_msg().sec

        # Publish cancellation status
        self.publish_execution_status(plan_id, "failed", "Plan canceled by user")

        # Remove from active plans
        del self.active_plans[plan_id]

        self.get_logger().info(f'Canceled plan: {plan_id}')
        return True


def create_vla_publisher_node() -> VLAPublisher:
    """
    Factory function to create a VLA publisher node

    Returns:
        VLAPublisher instance
    """
    rclpy.init()
    return VLAPublisher()


# Example usage:
if __name__ == '__main__':
    # This would typically be run as part of a ROS 2 system
    print("VLAPublisher class ready for use in ROS 2 system")
    print("Run with: ros2 run your_package vla_publisher")