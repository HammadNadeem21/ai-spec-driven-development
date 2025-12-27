"""
Task Plan Model for Vision-Language-Action (VLA) System

Based on the data model defined in the VLA specification.
"""
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import time
import uuid
from .action_sequence import ActionSequence


@dataclass
class TaskPlan:
    """
    Represents a decomposed plan generated from a natural language command

    Fields:
    - id: Unique identifier for the plan
    - original_command: The original voice command text
    - actions: Array of ordered actions to execute
    - constraints: Safety and operational constraints
    - status: Current status of the plan (pending, executing, completed, failed)
    - created_at: Timestamp when the plan was created
    """

    id: str
    original_command: str
    actions: List[ActionSequence]
    constraints: List[Dict[str, Any]]
    status: str  # pending, executing, completed, failed, interrupted
    created_at: str
    completed_at: Optional[str] = None

    def __post_init__(self):
        """Validate the TaskPlan instance after initialization"""
        valid_statuses = ["pending", "executing", "completed", "failed", "interrupted"]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")

    @classmethod
    def create_from_command(cls, original_command: str, actions: List[ActionSequence], constraints: List[Dict[str, Any]] = None):
        """
        Create a TaskPlan instance from a natural language command

        Args:
            original_command: The original voice command text
            actions: List of ActionSequence objects to execute
            constraints: List of safety and operational constraints
        """
        if constraints is None:
            constraints = []

        return cls(
            id=f"tp_{uuid.uuid4().hex[:8]}",
            original_command=original_command,
            actions=actions,
            constraints=constraints,
            status="pending",
            created_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        )

    def start_execution(self):
        """Update the plan status to executing"""
        self.status = "executing"

    def complete_execution(self):
        """Update the plan status to completed and set completion time"""
        self.status = "completed"
        self.completed_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def fail_execution(self):
        """Update the plan status to failed"""
        self.status = "failed"
        self.completed_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def interrupt_execution(self):
        """Update the plan status to interrupted"""
        self.status = "interrupted"
        self.completed_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def get_next_action(self) -> Optional[ActionSequence]:
        """
        Get the next action to execute based on dependencies and completion status

        Returns:
            The next ActionSequence to execute, or None if no action is ready
        """
        for action in self.actions:
            if action.status == "pending":
                # Check if all dependencies are completed
                all_deps_satisfied = True
                for dep_id in action.dependencies:
                    dep_action = next((a for a in self.actions if a.id == dep_id), None)
                    if dep_action and dep_action.status != "completed":
                        all_deps_satisfied = False
                        break

                if all_deps_satisfied:
                    return action

        return None

    def is_complete(self) -> bool:
        """Check if all actions in the plan are completed"""
        completed_count = sum(1 for action in self.actions if action.status == "completed")
        return completed_count == len(self.actions)


# Example usage:
if __name__ == "__main__":
    # Create sample actions (assuming ActionSequence is properly defined)
    from .action_sequence import ActionSequence

    sample_actions = [
        ActionSequence(
            id="act_1",
            type="navigation",
            parameters={"target": "kitchen"},
            dependencies=[],
            timeout=10.0,
            priority=5,
            status="pending"
        ),
        ActionSequence(
            id="act_2",
            type="manipulation",
            parameters={"object": "cup"},
            dependencies=["act_1"],
            timeout=15.0,
            priority=5,
            status="pending"
        )
    ]

    sample_constraints = [
        {
            "id": "constraint_1",
            "type": "safety",
            "condition": "avoid_stairs",
            "severity": "error",
            "description": "Do not navigate near stairs"
        }
    ]

    # Create a sample task plan
    plan = TaskPlan.create_from_command(
        original_command="Go to the kitchen and bring me a cup",
        actions=sample_actions,
        constraints=sample_constraints
    )

    print(f"Task Plan ID: {plan.id}")
    print(f"Original Command: {plan.original_command}")
    print(f"Status: {plan.status}")
    print(f"Number of Actions: {len(plan.actions)}")
    print(f"Number of Constraints: {len(plan.constraints)}")