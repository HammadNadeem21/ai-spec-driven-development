"""
Action Sequence Model for Vision-Language-Action (VLA) System

Based on the data model defined in the VLA specification.
"""
from dataclasses import dataclass
from typing import List, Dict, Any
import time
import uuid


@dataclass
class ActionSequence:
    """
    Represents a single action in a task plan that can be executed by the robot

    Fields:
    - id: Unique identifier for the action
    - type: Type of action (navigation, manipulation, perception, etc.)
    - parameters: Parameters required for the action
    - dependencies: List of action IDs that must complete first
    - timeout: Maximum time allowed for the action
    - priority: Priority level for execution
    - status: Current status of the action (pending, executing, completed, failed)
    """

    id: str
    type: str  # navigation, manipulation, perception, communication, etc.
    parameters: Dict[str, Any]
    dependencies: List[str]
    timeout: float
    priority: int  # 0-10 scale
    status: str = "pending"  # pending, executing, completed, failed

    def __post_init__(self):
        """Validate the ActionSequence instance after initialization"""
        valid_types = ["navigation", "manipulation", "perception", "communication"]
        if self.type not in valid_types:
            raise ValueError(f"Type must be one of {valid_types}")

        if self.timeout <= 0:
            raise ValueError("Timeout must be positive")

        if not (0 <= self.priority <= 10):
            raise ValueError("Priority must be between 0 and 10")

        valid_statuses = ["pending", "executing", "completed", "failed"]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")

    @classmethod
    def create_action(cls, action_type: str, parameters: Dict[str, Any], dependencies: List[str] = None,
                     timeout: float = 10.0, priority: int = 5):
        """
        Create an ActionSequence instance

        Args:
            action_type: Type of action (navigation, manipulation, perception, etc.)
            parameters: Parameters required for the action
            dependencies: List of action IDs that must complete first
            timeout: Maximum time allowed for the action (default: 10.0 seconds)
            priority: Priority level for execution (default: 5)
        """
        if dependencies is None:
            dependencies = []

        return cls(
            id=f"act_{uuid.uuid4().hex[:8]}",
            type=action_type,
            parameters=parameters,
            dependencies=dependencies,
            timeout=timeout,
            priority=priority,
            status="pending"
        )

    def start_execution(self):
        """Update the action status to executing"""
        self.status = "executing"

    def complete_execution(self):
        """Update the action status to completed"""
        self.status = "completed"

    def fail_execution(self):
        """Update the action status to failed"""
        self.status = "failed"

    def is_ready(self, completed_actions: List[str]) -> bool:
        """
        Check if this action is ready to execute based on its dependencies

        Args:
            completed_actions: List of action IDs that have been completed

        Returns:
            True if all dependencies are satisfied, False otherwise
        """
        for dep_id in self.dependencies:
            if dep_id not in completed_actions:
                return False
        return True


# Example usage:
if __name__ == "__main__":
    # Create a sample navigation action
    nav_action = ActionSequence.create_action(
        action_type="navigation",
        parameters={"target_location": "kitchen", "speed": 0.5},
        dependencies=[],
        timeout=30.0,
        priority=7
    )

    # Create a sample manipulation action that depends on navigation
    manip_action = ActionSequence.create_action(
        action_type="manipulation",
        parameters={"object_id": "cup", "grasp_type": "top_grasp"},
        dependencies=[nav_action.id],
        timeout=20.0,
        priority=8
    )

    print(f"Navigation Action ID: {nav_action.id}")
    print(f"Navigation Action Type: {nav_action.type}")
    print(f"Navigation Action Parameters: {nav_action.parameters}")
    print(f"Navigation Action Status: {nav_action.status}")

    print(f"\nManipulation Action ID: {manip_action.id}")
    print(f"Manipulation Action Dependencies: {manip_action.dependencies}")
    print(f"Is Manipulation Action Ready (no completed actions): {manip_action.is_ready([])}")
    print(f"Is Manipulation Action Ready (with nav completed): {manip_action.is_ready([nav_action.id])}")