"""
Safety Constraint Model for Vision-Language-Action (VLA) System

Based on the data model defined in the VLA specification.
"""
from dataclasses import dataclass
from typing import Dict, Any
import uuid


@dataclass
class SafetyConstraint:
    """
    Represents a safety constraint that must be validated before executing actions

    Fields:
    - id: Unique identifier for the constraint
    - type: Type of constraint (environmental, physical, operational)
    - condition: Condition that must be satisfied
    - severity: Severity level (warning, error)
    - description: Human-readable description of the constraint
    """

    id: str
    type: str  # environmental, physical, operational
    condition: str
    severity: str  # warning, error
    description: str

    def __post_init__(self):
        """Validate the SafetyConstraint instance after initialization"""
        valid_types = ["environmental", "physical", "operational"]
        if self.type not in valid_types:
            raise ValueError(f"Type must be one of {valid_types}")

        valid_severities = ["warning", "error"]
        if self.severity not in valid_severities:
            raise ValueError(f"Severity must be one of {valid_severities}")

    @classmethod
    def create_constraint(cls, constraint_type: str, condition: str, severity: str, description: str):
        """
        Create a SafetyConstraint instance

        Args:
            constraint_type: Type of constraint (environmental, physical, operational)
            condition: Condition that must be satisfied
            severity: Severity level (warning, error)
            description: Human-readable description of the constraint
        """
        return cls(
            id=f"sc_{uuid.uuid4().hex[:8]}",
            type=constraint_type,
            condition=condition,
            severity=severity,
            description=description
        )

    def is_satisfied(self, current_state: Dict[str, Any]) -> bool:
        """
        Check if the safety constraint is satisfied given the current state

        Args:
            current_state: Current state of the system/environment

        Returns:
            True if constraint is satisfied, False otherwise
        """
        # This is a simplified implementation - in a real system, this would contain
        # complex logic to evaluate the constraint against the current state
        if self.condition == "avoid_stairs":
            return not current_state.get("near_stairs", False)
        elif self.condition == "avoid_hazards":
            return not current_state.get("hazard_detected", False)
        elif self.condition == "maintain_battery_above_threshold":
            battery_level = current_state.get("battery_level", 100.0)
            return battery_level > current_state.get("battery_threshold", 20.0)
        elif self.condition == "limit_velocity":
            target_velocity = current_state.get("max_velocity", 1.0)
            current_velocity = current_state.get("current_velocity", 0.0)
            return current_velocity <= target_velocity
        else:
            # For other conditions, implement specific logic
            return True  # Default to satisfied for unknown conditions

    def should_block_execution(self) -> bool:
        """
        Check if this constraint should block action execution

        Returns:
            True if severity is 'error', False if 'warning'
        """
        return self.severity == "error"


# Example usage:
if __name__ == "__main__":
    # Create sample safety constraints
    avoid_stairs_constraint = SafetyConstraint.create_constraint(
        constraint_type="environmental",
        condition="avoid_stairs",
        severity="error",
        description="Do not navigate near stairs to prevent falls"
    )

    battery_threshold_constraint = SafetyConstraint.create_constraint(
        constraint_type="operational",
        condition="maintain_battery_above_threshold",
        severity="warning",
        description="Maintain battery level above 20% for safe operation"
    )

    print(f"Avoid Stairs Constraint ID: {avoid_stairs_constraint.id}")
    print(f"Avoid Stairs Constraint Type: {avoid_stairs_constraint.type}")
    print(f"Avoid Stairs Condition: {avoid_stairs_constraint.condition}")
    print(f"Avoid Stairs Severity: {avoid_stairs_constraint.severity}")
    print(f"Avoid Stairs Description: {avoid_stairs_constraint.description}")
    print(f"Should block execution: {avoid_stairs_constraint.should_block_execution()}")

    print(f"\nBattery Threshold Constraint ID: {battery_threshold_constraint.id}")
    print(f"Should block execution: {battery_threshold_constraint.should_block_execution()}")

    # Test constraint evaluation
    test_state_safe = {"near_stairs": False, "hazard_detected": False, "battery_level": 80.0}
    test_state_unsafe = {"near_stairs": True, "hazard_detected": False, "battery_level": 80.0}

    print(f"\nSafe state passes avoid_stairs constraint: {avoid_stairs_constraint.is_satisfied(test_state_safe)}")
    print(f"Unsafe state passes avoid_stairs constraint: {avoid_stairs_constraint.is_satisfied(test_state_unsafe)}")