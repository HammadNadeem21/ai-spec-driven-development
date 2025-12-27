"""
LLM Integration Module for Vision-Language-Action (VLA) Cognitive Planning

This module handles the integration with LLMs for cognitive planning and task decomposition.
"""
import openai
import json
from typing import List, Dict, Any, Optional
from .models.task_plan import TaskPlan
from .models.action_sequence import ActionSequence
from .models.safety_constraint import SafetyConstraint
import logging
import time

logger = logging.getLogger(__name__)


class LLMPlanning:
    """
    Handles integration with LLMs for cognitive planning and task decomposition
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-3.5-turbo"):
        """
        Initialize the LLM planning integration

        Args:
            api_key: OpenAI API key (if not set via environment variable)
            model: LLM model to use for planning (default: "gpt-3.5-turbo")
        """
        if api_key:
            openai.api_key = api_key
        self.model = model

    def decompose_task(self, command_text: str, robot_capabilities: Optional[Dict[str, Any]] = None) -> TaskPlan:
        """
        Decompose a natural language command into executable actions using LLM

        Args:
            command_text: Natural language command to decompose
            robot_capabilities: Dictionary describing robot capabilities (optional)

        Returns:
            TaskPlan object with decomposed actions
        """
        if robot_capabilities is None:
            robot_capabilities = {
                "navigation": True,
                "manipulation": True,
                "perception": True,
                "communication": True,
                "max_velocity": 1.0,
                "manipulation_reach": 1.0
            }

        # Create the prompt for the LLM
        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Decompose the following command into a sequence of executable actions.

        Robot capabilities:
        - Navigation: Can move to specified locations
        - Manipulation: Can grasp and move objects
        - Perception: Can detect and identify objects
        - Communication: Can speak and provide status updates
        - Max velocity: {robot_capabilities.get('max_velocity', 1.0)} m/s
        - Manipulation reach: {robot_capabilities.get('manipulation_reach', 1.0)} m

        Command: "{command_text}"

        Return a JSON object with the following structure:
        {{
            "actions": [
                {{
                    "type": "navigation|manipulation|perception|communication",
                    "parameters": {{"param1": "value1", ...}},
                    "dependencies": ["action_id_1", ...],
                    "timeout": 10.0,
                    "priority": 5
                }}
            ],
            "constraints": [
                {{
                    "type": "environmental|physical|operational",
                    "condition": "condition_to_check",
                    "severity": "warning|error",
                    "description": "constraint_description"
                }}
            ]
        }}

        Rules:
        1. Ensure actions are in logical order with proper dependencies
        2. Include safety constraints for all actions
        3. Set appropriate timeouts based on action complexity
        4. Set priorities (1-10) with higher numbers for critical actions
        5. Make sure all actions are achievable with the robot's capabilities
        """

        try:
            client = openai.OpenAI()
            response = client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"},
                temperature=0.3  # Lower temperature for more consistent outputs
            )

            result = json.loads(response.choices[0].message.content)

            # Create action sequences
            actions = []
            for action_data in result.get("actions", []):
                action = ActionSequence.create_action(
                    action_type=action_data["type"],
                    parameters=action_data["parameters"],
                    dependencies=action_data.get("dependencies", []),
                    timeout=action_data.get("timeout", 10.0),
                    priority=action_data.get("priority", 5)
                )
                actions.append(action)

            # Create safety constraints
            constraints = []
            for constraint_data in result.get("constraints", []):
                constraint = SafetyConstraint.create_constraint(
                    constraint_type=constraint_data["type"],
                    condition=constraint_data["condition"],
                    severity=constraint_data["severity"],
                    description=constraint_data["description"]
                )
                constraints.append(constraint)

            # Create and return the task plan
            return TaskPlan.create_from_command(
                original_command=command_text,
                actions=actions,
                constraints=constraints
            )

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse LLM response as JSON: {e}")
            raise Exception("LLM returned invalid JSON response")
        except openai.error.OpenAIError as e:
            logger.error(f"OpenAI API error: {e}")
            raise
        except Exception as e:
            logger.error(f"Error in task decomposition: {e}")
            raise

    def refine_plan(self, task_plan: TaskPlan, feedback: str) -> TaskPlan:
        """
        Refine an existing task plan based on feedback

        Args:
            task_plan: The original task plan to refine
            feedback: Feedback about the plan that should be addressed

        Returns:
            Refined TaskPlan object
        """
        prompt = f"""
        You are refining a robot task plan based on feedback. The original command was: "{task_plan.original_command}"

        Original plan actions:
        {json.dumps([{
            "id": action.id,
            "type": action.type,
            "parameters": action.parameters,
            "dependencies": action.dependencies,
            "timeout": action.timeout,
            "priority": action.priority
        } for action in task_plan.actions], indent=2)}

        Feedback: "{feedback}"

        Return a refined JSON plan with the same structure as before, addressing the feedback.
        """

        try:
            client = openai.OpenAI()
            response = client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"},
                temperature=0.3
            )

            result = json.loads(response.choices[0].message.content)

            # Create refined action sequences
            refined_actions = []
            for action_data in result.get("actions", []):
                action = ActionSequence.create_action(
                    action_type=action_data["type"],
                    parameters=action_data["parameters"],
                    dependencies=action_data.get("dependencies", []),
                    timeout=action_data.get("timeout", 10.0),
                    priority=action_data.get("priority", 5)
                )
                refined_actions.append(action)

            # Create refined constraints
            refined_constraints = []
            for constraint_data in result.get("constraints", []):
                constraint = SafetyConstraint.create_constraint(
                    constraint_type=constraint_data["type"],
                    condition=constraint_data["condition"],
                    severity=constraint_data["severity"],
                    description=constraint_data["description"]
                )
                refined_constraints.append(constraint)

            # Create and return the refined task plan
            refined_plan = TaskPlan.create_from_command(
                original_command=task_plan.original_command,
                actions=refined_actions,
                constraints=refined_constraints
            )

            return refined_plan

        except Exception as e:
            logger.error(f"Error refining plan: {e}")
            raise

    def validate_plan_safety(self, task_plan: TaskPlan, environment_state: Dict[str, Any]) -> List[str]:
        """
        Validate a task plan against safety constraints in the current environment

        Args:
            task_plan: The task plan to validate
            environment_state: Current state of the environment

        Returns:
            List of safety violations found
        """
        violations = []

        for constraint in task_plan.constraints:
            if not constraint.is_satisfied(environment_state):
                violations.append(f"Constraint '{constraint.description}' not satisfied")

        # Additional safety checks based on action sequence
        for i, action in enumerate(task_plan.actions):
            # Check for potential collisions in navigation actions
            if action.type == "navigation":
                target_location = action.parameters.get("target", {})
                if self._would_cause_collision(target_location, environment_state):
                    violations.append(f"Navigation to {target_location} would cause collision")

            # Check for unsafe manipulation
            elif action.type == "manipulation":
                object_id = action.parameters.get("object_id", "")
                if self._unsafe_manipulation(object_id, environment_state):
                    violations.append(f"Manipulation of {object_id} is unsafe")

        return violations

    def _would_cause_collision(self, target_location: Dict[str, Any], environment_state: Dict[str, Any]) -> bool:
        """
        Check if navigation to a target location would cause a collision

        Args:
            target_location: Target navigation location
            environment_state: Current environment state

        Returns:
            True if collision would occur, False otherwise
        """
        # This is a simplified implementation
        # In a real system, this would involve path planning and collision detection
        obstacles = environment_state.get("obstacles", [])
        robot_position = environment_state.get("robot_pose", {"x": 0, "y": 0})

        # Simple distance check - if target is too close to an obstacle
        for obstacle in obstacles:
            distance_to_obstacle = self._calculate_distance(target_location, obstacle)
            if distance_to_obstacle < 0.5:  # 0.5m safety margin
                return True

        return False

    def _unsafe_manipulation(self, object_id: str, environment_state: Dict[str, Any]) -> bool:
        """
        Check if manipulation of an object would be unsafe

        Args:
            object_id: ID of the object to manipulate
            environment_state: Current environment state

        Returns:
            True if manipulation is unsafe, False otherwise
        """
        # This is a simplified implementation
        # In a real system, this would check object properties, location, etc.
        unsafe_objects = environment_state.get("unsafe_objects", [])
        return object_id in unsafe_objects

    def _calculate_distance(self, pos1: Dict[str, Any], pos2: Dict[str, Any]) -> float:
        """
        Calculate 2D distance between two positions

        Args:
            pos1: First position with x, y coordinates
            pos2: Second position with x, y coordinates

        Returns:
            Distance between the positions
        """
        dx = pos1.get("x", 0) - pos2.get("x", 0)
        dy = pos1.get("y", 0) - pos2.get("y", 0)
        return (dx**2 + dy**2)**0.5


# Example usage:
if __name__ == "__main__":
    import os

    # Initialize LLM planning
    # Note: In a real system, you would set OPENAI_API_KEY environment variable
    planner = LLMPlanning()

    # Example: Decompose a simple command
    try:
        task_plan = planner.decompose_task("Go to the kitchen and bring me a cup")
        print(f"Decomposed command: {task_plan.original_command}")
        print(f"Number of actions: {len(task_plan.actions)}")
        print(f"Number of constraints: {len(task_plan.constraints)}")

        for i, action in enumerate(task_plan.actions):
            print(f"  Action {i+1}: {action.type} - {action.parameters}")

    except Exception as e:
        print(f"Task decomposition failed: {e}")
        print("Make sure OPENAI_API_KEY environment variable is set")

    # Example: Show the class structure
    print("\nLLMPlanning class ready for use with OpenAI API")
    print("Requires OPENAI_API_KEY environment variable to be set")