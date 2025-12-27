"""
Cognitive Planning System for Vision-Language-Action (VLA) Module

This module orchestrates the cognitive planning process using LLMs and safety validation.
"""
import json
from typing import Dict, Any, List, Optional
from .models.task_plan import TaskPlan
from .models.simulation_state import SimulationState
from .llm_planning import LLMPlanning
from .safety_validator import SafetyConstraintValidator
import logging
import time

logger = logging.getLogger(__name__)


class CognitivePlanner:
    """
    Main cognitive planning system that orchestrates LLM-based task decomposition
    with safety validation and constraint checking
    """

    def __init__(self, llm_planning: LLMPlanning, safety_validator: SafetyConstraintValidator):
        """
        Initialize the cognitive planner

        Args:
            llm_planning: Instance of LLMPlanning for task decomposition
            safety_validator: Instance of SafetyConstraintValidator for safety checks
        """
        self.llm_planning = llm_planning
        self.safety_validator = safety_validator
        self.active_plans = {}
        self.plan_history = []

    def create_plan(self, command_text: str, environment_state: Optional[SimulationState] = None,
                   robot_capabilities: Optional[Dict[str, Any]] = None) -> TaskPlan:
        """
        Create a task plan from a natural language command

        Args:
            command_text: Natural language command to process
            environment_state: Current simulation/environment state (optional)
            robot_capabilities: Robot capabilities description (optional)

        Returns:
            TaskPlan object with decomposed actions and safety constraints
        """
        logger.info(f"Creating plan for command: {command_text}")

        try:
            # Decompose the task using LLM
            task_plan = self.llm_planning.decompose_task(command_text, robot_capabilities)

            # Validate safety constraints if environment state is provided
            if environment_state:
                violations = self.llm_planning.validate_plan_safety(
                    task_plan,
                    {
                        "obstacles": getattr(environment_state, 'environment_objects', []),
                        "robot_pose": getattr(environment_state, 'robot_pose', {}),
                        "unsafe_objects": []  # Could be populated from environment state
                    }
                )

                if violations:
                    logger.warning(f"Plan has safety violations: {violations}")
                    # In a real system, we might want to refine the plan based on violations
                    # For now, we'll proceed but log the issues

            # Store the plan
            self.active_plans[task_plan.id] = task_plan
            self.plan_history.append(task_plan)

            logger.info(f"Created plan {task_plan.id} with {len(task_plan.actions)} actions")
            return task_plan

        except Exception as e:
            logger.error(f"Error creating plan for command '{command_text}': {e}")
            raise

    def refine_plan(self, plan_id: str, feedback: str) -> TaskPlan:
        """
        Refine an existing task plan based on feedback

        Args:
            plan_id: ID of the plan to refine
            feedback: Feedback about the plan that should be addressed

        Returns:
            Refined TaskPlan object
        """
        if plan_id not in self.active_plans:
            raise ValueError(f"Plan with ID {plan_id} not found")

        original_plan = self.active_plans[plan_id]

        try:
            # Refine the plan using LLM
            refined_plan = self.llm_planning.refine_plan(original_plan, feedback)

            # Update the active plans dictionary
            self.active_plans[plan_id] = refined_plan

            logger.info(f"Refined plan {plan_id}")
            return refined_plan

        except Exception as e:
            logger.error(f"Error refining plan {plan_id}: {e}")
            raise

    def validate_plan(self, plan_id: str, environment_state: SimulationState) -> Dict[str, Any]:
        """
        Validate a plan against the current environment state

        Args:
            plan_id: ID of the plan to validate
            environment_state: Current environment state for validation

        Returns:
            Dictionary with validation results
        """
        if plan_id not in self.active_plans:
            raise ValueError(f"Plan with ID {plan_id} not found")

        plan = self.active_plans[plan_id]

        # Validate safety constraints
        violations = self.llm_planning.validate_plan_safety(
            plan,
            {
                "obstacles": environment_state.environment_objects,
                "robot_pose": environment_state.robot_pose,
                "sensor_data": environment_state.sensor_data,
                "unsafe_objects": []  # Could be populated from sensor data
            }
        )

        # Check if plan is executable with current robot state
        is_executable = len(violations) == 0

        validation_result = {
            "plan_id": plan_id,
            "is_valid": is_executable,
            "violations": violations,
            "action_count": len(plan.actions),
            "constraint_count": len(plan.constraints)
        }

        return validation_result

    def get_plan_status(self, plan_id: str) -> Dict[str, Any]:
        """
        Get the current status of a plan

        Args:
            plan_id: ID of the plan to check

        Returns:
            Dictionary with plan status information
        """
        if plan_id not in self.active_plans:
            raise ValueError(f"Plan with ID {plan_id} not found")

        plan = self.active_plans[plan_id]

        completed_actions = sum(1 for action in plan.actions if action.status == "completed")
        total_actions = len(plan.actions)

        status = {
            "plan_id": plan_id,
            "status": plan.status,
            "original_command": plan.original_command,
            "total_actions": total_actions,
            "completed_actions": completed_actions,
            "progress_percentage": (completed_actions / total_actions) * 100 if total_actions > 0 else 0,
            "created_at": plan.created_at,
            "completed_at": plan.completed_at
        }

        return status

    def cancel_plan(self, plan_id: str) -> bool:
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
        plan.status = "failed"  # Mark as failed rather than completed
        plan.completed_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

        # Move from active to history
        del self.active_plans[plan_id]
        self.plan_history.append(plan)

        logger.info(f"Canceled plan {plan_id}")
        return True

    def get_active_plans(self) -> List[TaskPlan]:
        """
        Get all currently active plans

        Returns:
            List of active TaskPlan objects
        """
        return list(self.active_plans.values())

    def get_next_action(self, plan_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the next action to execute for a plan

        Args:
            plan_id: ID of the plan

        Returns:
            Dictionary with action information, or None if no action is ready
        """
        if plan_id not in self.active_plans:
            raise ValueError(f"Plan with ID {plan_id} not found")

        plan = self.active_plans[plan_id]

        # Get the next action that is ready to execute
        next_action = plan.get_next_action()

        if next_action:
            # Mark action as executing
            next_action.start_execution()

            return {
                "action_id": next_action.id,
                "action_type": next_action.type,
                "parameters": next_action.parameters,
                "timeout": next_action.timeout,
                "priority": next_action.priority,
                "plan_id": plan_id
            }

        return None

    def complete_action(self, plan_id: str, action_id: str, success: bool = True) -> bool:
        """
        Mark an action as completed

        Args:
            plan_id: ID of the plan containing the action
            action_id: ID of the action that completed
            success: Whether the action completed successfully

        Returns:
            True if action was successfully marked as completed
        """
        if plan_id not in self.active_plans:
            return False

        plan = self.active_plans[plan_id]

        # Find the action
        action = next((a for a in plan.actions if a.id == action_id), None)
        if not action:
            return False

        # Update action status
        if success:
            action.complete_execution()
        else:
            action.fail_execution()

        # Check if the entire plan is complete
        if plan.is_complete():
            plan.complete_execution()

            # Move from active to history
            del self.active_plans[plan_id]
            self.plan_history.append(plan)

        return True


# Example usage:
if __name__ == "__main__":
    # This would require the LLMPlanning and SafetyConstraintValidator classes
    # to be properly initialized with API keys
    print("CognitivePlanner class ready for use")
    print("Requires LLMPlanning and SafetyConstraintValidator instances")