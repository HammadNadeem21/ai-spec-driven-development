"""
VLA Orchestrator Node for Vision-Language-Action (VLA) System

This module orchestrates the complete VLA pipeline: voice processing, cognitive planning,
and action execution in a unified system.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import AudioData
from typing import Dict, Any, Optional
from .models.voice_command import VoiceCommand
from .models.task_plan import TaskPlan
from .models.simulation_state import SimulationState
from .whisper_processor import WhisperProcessor
from .cognitive_planner import CognitivePlanner
from .safety_validator import SafetyConstraintValidator
from .llm_planning import LLMPlanning
from .ros_publishers import VLAPublisher
import json
import logging
import time

logger = logging.getLogger(__name__)


class VLAOrchestrator(Node):
    """
    Main orchestrator node that coordinates the complete VLA pipeline
    """

    def __init__(self, node_name: str = 'vla_orchestrator'):
        """
        Initialize the VLA orchestrator node

        Args:
            node_name: Name of the ROS 2 node (default: 'vla_orchestrator')
        """
        super().__init__(node_name)

        # Initialize components
        self.llm_planning = None  # Will be set by user or loaded from config
        self.safety_validator = SafetyConstraintValidator()
        self.cognitive_planner = None  # Will be set after LLM planning is configured
        self.whisper_processor = None  # Will be set after Whisper integration is configured
        self.vla_publisher = VLAPublisher(f"{node_name}_publisher")

        # Current state
        self.current_state = SimulationState.create_initial_state()
        self.active_plans = {}
        self.voice_command_buffer = []

        # ROS 2 interfaces
        self.setup_ros_interfaces()

        # Timers
        self.execution_timer = self.create_timer(0.1, self.execute_plan_step)  # 10 Hz execution
        self.state_update_timer = self.create_timer(1.0, self.update_simulation_state)  # 1 Hz state updates

        logger.info(f"Initialized VLA orchestrator node: {node_name}")

    def setup_ros_interfaces(self) -> None:
        """Setup ROS 2 publishers and subscribers"""
        # Publishers (using the VLAPublisher instance)
        self.status_publisher = self.create_publisher(String, 'vla/orchestrator_status', 10)

        # Subscribers
        self.voice_command_subscriber = self.create_subscription(
            String,
            'vla/voice_commands',
            self.voice_command_callback,
            10
        )

        self.audio_data_subscriber = self.create_subscription(
            AudioData,
            'vla/audio_data',
            self.audio_data_callback,
            10
        )

        self.action_status_subscriber = self.create_subscription(
            String,
            'vla/action_status',
            self.action_status_callback,
            10
        )

        self.simulation_state_subscriber = self.create_subscription(
            String,
            'vla/simulation_state',
            self.simulation_state_callback,
            10
        )

        self.confidence_subscriber = self.create_subscription(
            Float32,
            'vla/transcription_confidence',
            self.confidence_callback,
            10
        )

    def set_llm_planning(self, llm_planning: LLMPlanning) -> None:
        """
        Set the LLM planning component

        Args:
            llm_planning: LLMPlanning instance
        """
        self.llm_planning = llm_planning
        self.cognitive_planner = CognitivePlanner(self.llm_planning, self.safety_validator)

    def set_whisper_processor(self, whisper_processor: WhisperProcessor) -> None:
        """
        Set the Whisper processor component

        Args:
            whisper_processor: WhisperProcessor instance
        """
        self.whisper_processor = whisper_processor

    def voice_command_callback(self, msg: String) -> None:
        """
        Handle incoming voice commands

        Args:
            msg: ROS 2 String message containing voice command data
        """
        try:
            command_data = json.loads(msg.data)
            voice_command = VoiceCommand(
                id=command_data["id"],
                transcript=command_data["transcript"],
                confidence=command_data["confidence"],
                timestamp=command_data["timestamp"],
                language=command_data["language"],
                audio_data=command_data.get("audio_data_ref")
            )

            self.get_logger().info(f'Received voice command: {voice_command.transcript}')

            # Process the voice command through the VLA pipeline
            self.process_voice_command(voice_command)

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')

    def audio_data_callback(self, msg: AudioData) -> None:
        """
        Handle incoming audio data

        Args:
            msg: ROS 2 AudioData message containing raw audio
        """
        try:
            # Process audio through Whisper if processor is available
            if self.whisper_processor:
                # In a real system, we'd convert AudioData to bytes format
                # For this example, we'll just log the data
                self.get_logger().info(f'Received audio data: {len(msg.data)} bytes')
            else:
                self.get_logger().warn('Whisper processor not configured, cannot process audio')

        except Exception as e:
            self.get_logger().error(f'Error processing audio data: {e}')

    def action_status_callback(self, msg: String) -> None:
        """
        Handle incoming action status updates

        Args:
            msg: ROS 2 String message containing action status data
        """
        try:
            status_data = json.loads(msg.data)
            action_id = status_data["action_id"]
            status = status_data["status"]
            plan_id = status_data.get("plan_id")

            self.get_logger().info(f'Action {action_id} status: {status}')

            # Update plan status if it's an active plan
            if plan_id and plan_id in self.active_plans:
                self.vla_publisher.update_active_plan(plan_id, action_id, status)

                # Check if plan is complete
                plan_status = self.vla_publisher.get_active_plan_status(plan_id)
                if plan_status and plan_status["progress_percentage"] == 100:
                    self.get_logger().info(f'Plan {plan_id} completed successfully')
                    self.vla_publisher.publish_execution_status(plan_id, "completed", "Plan executed successfully")

        except Exception as e:
            self.get_logger().error(f'Error processing action status: {e}')

    def simulation_state_callback(self, msg: String) -> None:
        """
        Handle incoming simulation state updates

        Args:
            msg: ROS 2 String message containing simulation state data
        """
        try:
            state_data = json.loads(msg.data)
            self.current_state = SimulationState(
                id=state_data["id"],
                robot_pose=state_data["robot_pose"],
                environment_objects=state_data["environment_objects"],
                sensor_data=state_data["sensor_data"],
                timestamp=state_data["timestamp"]
            )

            self.get_logger().info(f'Updated simulation state: {self.current_state.id}')

        except Exception as e:
            self.get_logger().error(f'Error processing simulation state: {e}')

    def confidence_callback(self, msg: Float32) -> None:
        """
        Handle incoming confidence updates

        Args:
            msg: ROS 2 Float32 message containing confidence value
        """
        self.get_logger().info(f'Transcription confidence: {msg.data}')

    def process_voice_command(self, voice_command: VoiceCommand) -> None:
        """
        Process a voice command through the complete VLA pipeline

        Args:
            voice_command: VoiceCommand to process
        """
        try:
            self.get_logger().info(f'Processing voice command: {voice_command.transcript}')

            # Validate transcription quality
            if voice_command.confidence < 0.7:
                self.get_logger().warn(f'Low confidence transcription: {voice_command.confidence}')
                # Optionally, we could request repetition or use a different processing path
                # For now, we'll proceed but log the low confidence

            # Create a task plan using cognitive planning
            if not self.cognitive_planner:
                raise RuntimeError("Cognitive planner not configured")

            # Create the task plan
            task_plan = self.cognitive_planner.create_plan(
                voice_command.transcript,
                self.current_state
            )

            # Validate the plan for safety
            validation_result = self.cognitive_planner.validate_plan(task_plan.id, self.current_state)
            if not validation_result["is_valid"]:
                self.get_logger().error(f'Plan {task_plan.id} failed safety validation: {validation_result["violations"]}')
                # In a real system, we might want to refine the plan or request clarification
                # For now, we'll publish the plan but mark it as potentially unsafe
                self.vla_publisher.publish_execution_status(
                    task_plan.id,
                    "warning",
                    f"Plan has safety violations: {'; '.join(validation_result['violations'])}"
                )

            # Publish the task plan for execution
            self.vla_publisher.publish_task_plan(task_plan)
            self.active_plans[task_plan.id] = task_plan

            self.get_logger().info(f'Published task plan {task_plan.id} for execution')

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            # Publish error status
            error_msg = String()
            error_msg.data = json.dumps({
                "status": "error",
                "message": str(e),
                "command_id": voice_command.id
            })
            self.status_publisher.publish(error_msg)

    def execute_plan_step(self) -> None:
        """
        Execute one step of active plans
        """
        try:
            for plan_id, plan in list(self.active_plans.items()):
                if plan.status == "executing":
                    # Get the next action to execute
                    next_action = self.cognitive_planner.get_next_action(plan_id)
                    if next_action:
                        # Publish action for execution by downstream nodes
                        action_msg = String()
                        action_msg.data = json.dumps(next_action)
                        # In a real system, this would publish to an action execution topic
                        self.get_logger().info(f'Queuing action for execution: {next_action["action_id"]}')

                        # Update action status to executing
                        self.vla_publisher.publish_action_status(
                            next_action["action_id"],
                            "executing",
                            plan_id=plan_id
                        )

                    elif plan.is_complete():
                        # Plan is complete
                        plan.complete_execution()
                        del self.active_plans[plan_id]
                        self.vla_publisher.publish_execution_status(
                            plan_id,
                            "completed",
                            "Plan executed successfully"
                        )
                        self.get_logger().info(f'Plan {plan_id} completed')

        except Exception as e:
            self.get_logger().error(f'Error in plan execution step: {e}')

    def update_simulation_state(self) -> None:
        """
        Update and publish the current simulation state
        """
        try:
            # In a real system, this would get updated state from simulation or sensors
            # For this example, we'll just update the timestamp
            self.current_state.timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

            # Publish the updated state
            self.vla_publisher.publish_simulation_state(self.current_state)

        except Exception as e:
            self.get_logger().error(f'Error updating simulation state: {e}')

    def cancel_plan(self, plan_id: str) -> bool:
        """
        Cancel an active plan

        Args:
            plan_id: ID of the plan to cancel

        Returns:
            True if plan was successfully canceled, False otherwise
        """
        return self.vla_publisher.cancel_active_plan(plan_id)

    def get_system_status(self) -> Dict[str, Any]:
        """
        Get the current status of the VLA system

        Returns:
            Dictionary with system status information
        """
        return {
            "active_plans_count": len(self.active_plans),
            "current_state_id": self.current_state.id,
            "llm_planning_configured": self.llm_planning is not None,
            "whisper_processor_configured": self.whisper_processor is not None,
            "active_plan_ids": list(self.active_plans.keys()),
            "timestamp": time.time()
        }

    def shutdown(self) -> None:
        """
        Perform cleanup operations before shutdown
        """
        self.get_logger().info('Shutting down VLA orchestrator')
        # Cancel any active plans
        for plan_id in list(self.active_plans.keys()):
            self.cancel_plan(plan_id)


def main(args=None):
    """
    Main function to run the VLA orchestrator node
    """
    rclpy.init(args=args)

    orchestrator = VLAOrchestrator()

    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.shutdown()
        orchestrator.destroy_node()
        rclpy.shutdown()


# Example usage:
if __name__ == '__main__':
    print("VLAOrchestrator class ready for use in ROS 2 system")
    print("Run with: ros2 run your_package vla_orchestrator")
    print("Requires LLMPlanning and WhisperProcessor instances to be configured")