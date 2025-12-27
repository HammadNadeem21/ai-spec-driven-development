"""
Safety Constraint Validation System for Vision-Language-Action (VLA) Module

This module validates actions and plans against safety constraints before execution.
"""
from typing import Dict, Any, List, Union
from .models.action_sequence import ActionSequence
from .models.task_plan import TaskPlan
from .models.simulation_state import SimulationState
from .models.safety_constraint import SafetyConstraint
import logging

logger = logging.getLogger(__name__)


class SafetyConstraintValidator:
    """
    Validates actions and plans against safety constraints before execution
    """

    def __init__(self):
        """
        Initialize the safety constraint validator
        """
        self.constraint_rules = {
            "navigation": self._validate_navigation_safety,
            "manipulation": self._validate_manipulation_safety,
            "perception": self._validate_perception_safety,
            "communication": self._validate_communication_safety
        }

    def validate_action(self, action: ActionSequence, current_state: SimulationState) -> Dict[str, Any]:
        """
        Validate a single action against safety constraints

        Args:
            action: ActionSequence to validate
            current_state: Current simulation/environment state

        Returns:
            Dictionary with validation results
        """
        logger.debug(f"Validating action {action.id} of type {action.type}")

        # Check if action type is supported
        if action.type not in self.constraint_rules:
            logger.warning(f"Unknown action type '{action.type}', skipping validation")
            return {
                "action_id": action.id,
                "is_safe": True,
                "violations": [],
                "warnings": [f"Unknown action type '{action.type}', proceed with caution"]
            }

        # Run type-specific validation
        type_validation = self.constraint_rules[action.type](action, current_state)

        # Check general safety constraints
        general_validation = self._validate_general_safety(action, current_state)

        # Combine results
        is_safe = type_validation["is_safe"] and general_validation["is_safe"]
        violations = type_validation["violations"] + general_validation["violations"]
        warnings = type_validation["warnings"] + general_validation["warnings"]

        result = {
            "action_id": action.id,
            "is_safe": is_safe,
            "violations": violations,
            "warnings": warnings,
            "action_type": action.type
        }

        if not is_safe:
            logger.warning(f"Action {action.id} failed safety validation: {violations}")

        return result

    def validate_task_plan(self, task_plan: TaskPlan, current_state: SimulationState) -> Dict[str, Any]:
        """
        Validate an entire task plan against safety constraints

        Args:
            task_plan: TaskPlan to validate
            current_state: Current simulation/environment state

        Returns:
            Dictionary with validation results for the entire plan
        """
        logger.info(f"Validating task plan {task_plan.id} with {len(task_plan.actions)} actions")

        plan_violations = []
        plan_warnings = []
        action_results = []

        for action in task_plan.actions:
            action_validation = self.validate_action(action, current_state)
            action_results.append(action_validation)

            if not action_validation["is_safe"]:
                plan_violations.extend([
                    f"Action {action.id}: {v}" for v in action_validation["violations"]
                ])

            plan_warnings.extend([
                f"Action {action.id}: {w}" for w in action_validation["warnings"]
            ])

        # Validate plan-level constraints
        for constraint in task_plan.constraints:
            if not constraint.is_satisfied(current_state.__dict__):
                plan_violations.append(f"Plan constraint violated: {constraint.description}")

        is_safe = len(plan_violations) == 0

        result = {
            "plan_id": task_plan.id,
            "is_safe": is_safe,
            "violations": plan_violations,
            "warnings": plan_warnings,
            "action_results": action_results,
            "total_actions": len(task_plan.actions),
            "unsafe_actions": len([r for r in action_results if not r["is_safe"]])
        }

        if not is_safe:
            logger.warning(f"Plan {task_plan.id} failed safety validation: {plan_violations}")

        return result

    def _validate_navigation_safety(self, action: ActionSequence, state: SimulationState) -> Dict[str, Any]:
        """
        Validate navigation action safety

        Args:
            action: Navigation action to validate
            state: Current simulation state

        Returns:
            Validation results
        """
        violations = []
        warnings = []

        # Check target position safety
        target_pose = action.parameters.get("target", {})
        if not self._is_safe_navigation_target(target_pose, state):
            violations.append("Navigation target is in unsafe location")

        # Check path safety
        if not self._is_path_clear(target_pose, state):
            violations.append("Path to target has obstacles or hazards")

        # Check velocity constraints
        max_velocity = action.parameters.get("max_velocity", 1.0)
        if max_velocity > 2.0:  # Arbitrary safety limit
            warnings.append("High navigation velocity requested")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations,
            "warnings": warnings
        }

    def _validate_manipulation_safety(self, action: ActionSequence, state: SimulationState) -> Dict[str, Any]:
        """
        Validate manipulation action safety

        Args:
            action: Manipulation action to validate
            state: Current simulation state

        Returns:
            Validation results
        """
        violations = []
        warnings = []

        # Check if target object is safe to manipulate
        object_id = action.parameters.get("object_id", "")
        if not self._is_safe_to_manipulate(object_id, state):
            violations.append(f"Object {object_id} is unsafe to manipulate")

        # Check if target pose is safe for manipulation
        target_pose = action.parameters.get("target_pose", {})
        if not self._is_safe_manipulation_target(target_pose, state):
            violations.append("Manipulation target pose is unsafe")

        # Check grasp type safety
        grasp_type = action.parameters.get("grasp_type", "")
        unsafe_grasps = ["power_grasp_hot", "pinch_grasp_fragile_with_force"]
        if grasp_type in unsafe_grasps:
            violations.append(f"Unsafe grasp type requested: {grasp_type}")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations,
            "warnings": warnings
        }

    def _validate_perception_safety(self, action: ActionSequence, state: SimulationState) -> Dict[str, Any]:
        """
        Validate perception action safety

        Args:
            action: Perception action to validate
            state: Current simulation state

        Returns:
            Validation results
        """
        violations = []
        warnings = []

        # Check if perception is safe to perform in current environment
        sensor_name = action.parameters.get("sensor", "default")
        if sensor_name == "lidar" and state.sensor_data.get("lidar_blocked", False):
            violations.append("Lidar sensor is blocked, perception unsafe")

        # Check for hazardous environmental conditions during perception
        if state.sensor_data.get("hazard_detected", False):
            warnings.append("Hazard detected, proceed with caution during perception")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations,
            "warnings": warnings
        }

    def _validate_communication_safety(self, action: ActionSequence, state: SimulationState) -> Dict[str, Any]:
        """
        Validate communication action safety

        Args:
            action: Communication action to validate
            state: Current simulation state

        Returns:
            Validation results
        """
        violations = []
        warnings = []

        # Check if communication is appropriate given the state
        message = action.parameters.get("text", "")
        if "emergency" in message.lower() and not state.sensor_data.get("emergency_mode", False):
            warnings.append("Emergency communication without emergency state detected")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations,
            "warnings": warnings
        }

    def _validate_general_safety(self, action: ActionSequence, state: SimulationState) -> Dict[str, Any]:
        """
        Validate general safety constraints not specific to action type

        Args:
            action: Action to validate
            state: Current simulation state

        Returns:
            Validation results
        """
        violations = []
        warnings = []

        # Check battery level
        battery_level = state.sensor_data.get("battery_level", 100.0)
        if battery_level < 10.0:
            violations.append("Battery level too low for safe operation")

        # Check for emergency conditions
        if state.sensor_data.get("emergency_stop", False):
            violations.append("Emergency stop condition detected")

        # Check if robot is in safe operational state
        robot_status = state.sensor_data.get("robot_status", "normal")
        if robot_status == "error":
            violations.append("Robot in error state, unsafe to proceed")

        # Check time limits
        timeout = action.timeout
        if timeout > 300:  # 5 minutes max for safety
            warnings.append("Action timeout is very long, consider reducing")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations,
            "warnings": warnings
        }

    def _is_safe_navigation_target(self, target_pose: Dict[str, Any], state: SimulationState) -> bool:
        """
        Check if navigation target is in a safe location

        Args:
            target_pose: Target position for navigation
            state: Current simulation state

        Returns:
            True if target is safe, False otherwise
        """
        # Check if target is near hazards
        for obj in state.environment_objects:
            if obj.get("type") == "hazard" or obj.get("type") == "unsafe_area":
                distance = self._calculate_distance(target_pose, obj)
                if distance < 1.0:  # 1 meter safety margin
                    return False

        # Check if target is accessible
        if target_pose.get("z", 0) > 2.0:  # Too high for ground robot
            return False

        return True

    def _is_path_clear(self, target_pose: Dict[str, Any], state: SimulationState) -> bool:
        """
        Check if path to target is clear of obstacles

        Args:
            target_pose: Target position for navigation
            state: Current simulation state

        Returns:
            True if path is clear, False otherwise
        """
        # This is a simplified check - in reality, this would involve path planning
        # For now, we'll check if there are any obstacles very close to the target
        for obj in state.environment_objects:
            if obj.get("type") == "obstacle":
                distance = self._calculate_distance(target_pose, obj)
                if distance < 0.5:  # 0.5 meter clearance
                    return False

        return True

    def _is_safe_to_manipulate(self, object_id: str, state: SimulationState) -> bool:
        """
        Check if an object is safe to manipulate

        Args:
            object_id: ID of object to manipulate
            state: Current simulation state

        Returns:
            True if object is safe to manipulate, False otherwise
        """
        # Find the object in the environment
        target_obj = None
        for obj in state.environment_objects:
            if obj.get("id") == object_id:
                target_obj = obj
                break

        if not target_obj:
            logger.warning(f"Object {object_id} not found in environment")
            return False

        # Check if object is marked as unsafe
        if target_obj.get("unsafe", False):
            return False

        # Check object properties
        obj_type = target_obj.get("type", "")
        if obj_type in ["hazard", "fragile", "hot", "sharp"]:
            return False

        # Check object weight if available
        weight = target_obj.get("weight", 1000)  # Default to very heavy if unknown
        max_safe_weight = 5.0  # 5kg max for safe manipulation
        if weight > max_safe_weight:
            return False

        return True

    def _is_safe_manipulation_target(self, target_pose: Dict[str, Any], state: SimulationState) -> bool:
        """
        Check if manipulation target pose is safe

        Args:
            target_pose: Target pose for manipulation
            state: Current simulation state

        Returns:
            True if pose is safe for manipulation, False otherwise
        """
        # Check if target is within robot's reach
        robot_pose = state.robot_pose
        distance = self._calculate_distance(target_pose, robot_pose)

        # Assume 1m max reach for safety
        max_reach = state.sensor_data.get("manipulation_reach", 1.0)
        if distance > max_reach:
            return False

        # Check if target is in a safe position (not too high, low, etc.)
        if target_pose.get("z", 0) < 0.1:  # Too low, might be unstable
            return False
        if target_pose.get("z", 0) > 2.0:  # Too high for safe manipulation
            return False

        return True

    def _calculate_distance(self, pos1: Dict[str, Any], pos2: Dict[str, Any]) -> float:
        """
        Calculate 3D distance between two positions

        Args:
            pos1: First position with x, y, z coordinates
            pos2: Second position with x, y, z coordinates

        Returns:
            Distance between the positions
        """
        dx = pos1.get("x", 0) - pos2.get("x", 0)
        dy = pos1.get("y", 0) - pos2.get("y", 0)
        dz = pos1.get("z", 0) - pos2.get("z", 0)
        return (dx**2 + dy**2 + dz**2)**0.5

    def get_safety_report(self, state: SimulationState) -> Dict[str, Any]:
        """
        Generate a comprehensive safety report for the current state

        Args:
            state: Current simulation state

        Returns:
            Safety report with various safety metrics
        """
        hazards = [obj for obj in state.environment_objects if obj.get("type") in ["hazard", "unsafe_area"]]
        obstacles = [obj for obj in state.environment_objects if obj.get("type") == "obstacle"]

        battery_level = state.sensor_data.get("battery_level", 100.0)
        robot_status = state.sensor_data.get("robot_status", "normal")
        emergency_active = state.sensor_data.get("emergency_stop", False)

        return {
            "timestamp": state.timestamp,
            "hazards_detected": len(hazards),
            "obstacles_count": len(obstacles),
            "battery_level": battery_level,
            "robot_status": robot_status,
            "emergency_active": emergency_active,
            "safe_operational": battery_level > 20.0 and robot_status == "normal" and not emergency_active
        }


# Example usage:
if __name__ == "__main__":
    from .models.action_sequence import ActionSequence
    from .models.simulation_state import SimulationState

    # Create a validator instance
    validator = SafetyConstraintValidator()

    # Create a sample action
    action = ActionSequence.create_action(
        action_type="navigation",
        parameters={"target": {"x": 1.0, "y": 2.0, "z": 0.0}},
        timeout=10.0
    )

    # Create a sample state
    state = SimulationState.create_initial_state()

    # Validate the action
    result = validator.validate_action(action, state)
    print(f"Action validation result: {result}")