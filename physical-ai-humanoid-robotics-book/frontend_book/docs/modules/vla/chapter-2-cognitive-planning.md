---
sidebar_position: 2
title: "Chapter 2: Language-Driven Cognitive Planning"
---

# Chapter 2: Language-Driven Cognitive Planning

## Overview

This chapter focuses on using Large Language Models (LLMs) for cognitive planning in humanoid robotics. We'll explore how to translate natural language commands into executable ROS 2 action sequences while implementing safety and constraint-aware planning mechanisms.

## Using LLMs for Task Decomposition

Large Language Models have revolutionized the ability to understand and decompose complex natural language commands into structured action sequences. In robotics, this capability enables more intuitive human-robot interaction by allowing users to express complex tasks in natural language.

### Cognitive Planning Architecture

```python
import openai
import time
from dataclasses import dataclass
from typing import List, Dict, Any
import json

@dataclass
class ActionSequence:
    """Represents a single action in a task plan that can be executed by the robot"""
    id: str
    type: str  # navigation, manipulation, perception, etc.
    parameters: Dict[str, Any]
    dependencies: List[str]
    timeout: float
    priority: int

@dataclass
class TaskPlan:
    """Represents a decomposed plan generated from a natural language command"""
    id: str
    original_command: str
    actions: List[ActionSequence]
    constraints: List[Dict[str, Any]]
    status: str  # pending, executing, completed, failed
    created_at: str

@dataclass
class SafetyConstraint:
    """Represents a safety constraint that must be validated before executing actions"""
    id: str
    type: str  # environmental, physical, operational
    condition: str
    severity: str  # warning, error
    description: str

class CognitivePlanningSystem:
    def __init__(self, model_name="gpt-3.5-turbo"):
        self.client = openai.OpenAI()
        self.model_name = model_name

    def decompose_task(self, command_text: str) -> TaskPlan:
        """Decompose a natural language command into executable actions"""
        prompt = f"""
        Decompose the following command into a sequence of ROS 2 actions for a humanoid robot:
        Command: "{command_text}"

        Return a JSON object with the following structure:
        {{
            "actions": [
                {{
                    "id": "action_id",
                    "type": "action_type",
                    "parameters": {{"param1": "value1"}},
                    "dependencies": ["dependency_id"],
                    "timeout": 10.0,
                    "priority": 5
                }}
            ],
            "constraints": [
                {{
                    "id": "constraint_id",
                    "type": "constraint_type",
                    "condition": "condition_string",
                    "severity": "error",
                    "description": "constraint_description"
                }}
            ]
        }}

        Action types should be from: navigation, manipulation, perception, communication.
        Ensure all actions are safe and executable by a humanoid robot.
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)

            # Create action sequences
            actions = []
            for action_data in result.get("actions", []):
                action = ActionSequence(
                    id=action_data["id"],
                    type=action_data["type"],
                    parameters=action_data["parameters"],
                    dependencies=action_data.get("dependencies", []),
                    timeout=action_data.get("timeout", 10.0),
                    priority=action_data.get("priority", 5)
                )
                actions.append(action)

            # Create task plan
            task_plan = TaskPlan(
                id=f"plan_{int(time.time())}",
                original_command=command_text,
                actions=actions,
                constraints=result.get("constraints", []),
                status="pending",
                created_at=time.time()
            )

            return task_plan

        except Exception as e:
            print(f"Error in task decomposition: {e}")
            # Return a default task plan indicating failure
            return TaskPlan(
                id=f"plan_{int(time.time())}",
                original_command=command_text,
                actions=[],
                constraints=[],
                status="failed",
                created_at=time.time()
            )
```

### LLM Prompt Engineering for Robotics

Effective task decomposition requires careful prompt engineering to ensure the LLM understands robotics-specific constraints and capabilities:

- **Context Provision**: Provide information about the robot's capabilities and limitations
- **Safety Constraints**: Include safety guidelines in the prompt to prevent dangerous actions
- **Action Vocabulary**: Define a standardized set of action types the robot can perform
- **Format Consistency**: Ensure consistent JSON output for reliable parsing

## Translating Natural Language into ROS 2 Action Sequences

The translation from natural language to executable actions involves several steps:

1. **Intent Recognition**: Understanding the user's high-level goal
2. **Task Decomposition**: Breaking down the goal into sequential actions
3. **Action Mapping**: Converting abstract actions to specific ROS 2 services/topics
4. **Constraint Validation**: Ensuring actions comply with safety and operational constraints

### Example Translation Process

```python
class NaturalLanguageTranslator:
    def __init__(self):
        self.planning_system = CognitivePlanningSystem()
        self.action_mapper = ROS2ActionMapper()

    def translate_command(self, command: str) -> List[ActionSequence]:
        """Translate natural language command to ROS 2 action sequences"""
        # Step 1: Decompose task using LLM
        task_plan = self.planning_system.decompose_task(command)

        # Step 2: Validate safety constraints
        if not self.validate_constraints(task_plan.constraints):
            raise ValueError("Safety constraints violated")

        # Step 3: Map to ROS 2 actions
        ros_actions = []
        for action in task_plan.actions:
            ros_action = self.action_mapper.map_to_ros2(action)
            ros_actions.append(ros_action)

        return ros_actions

    def validate_constraints(self, constraints: List[Dict[str, Any]]) -> bool:
        """Validate that all safety constraints are satisfied"""
        for constraint in constraints:
            if constraint["severity"] == "error":
                # Implement constraint validation logic
                if not self.check_constraint(constraint):
                    return False
        return True
```

### ROS 2 Action Mapping

```python
class ROS2ActionMapper:
    def __init__(self):
        self.action_mapping = {
            "navigation": self._map_navigation,
            "manipulation": self._map_manipulation,
            "perception": self._map_perception,
            "communication": self._map_communication
        }

    def map_to_ros2(self, action: ActionSequence) -> Dict[str, Any]:
        """Map action to ROS 2 service call or topic message"""
        if action.type in self.action_mapping:
            return self.action_mapping[action.type](action)
        else:
            raise ValueError(f"Unknown action type: {action.type}")

    def _map_navigation(self, action: ActionSequence) -> Dict[str, Any]:
        """Map navigation action to ROS 2 navigation2 interface"""
        return {
            "service": "/navigate_to_pose",
            "request": {
                "pose": action.parameters.get("pose", {}),
                "behavior_tree": action.parameters.get("behavior_tree", "default")
            }
        }

    def _map_manipulation(self, action: ActionSequence) -> Dict[str, Any]:
        """Map manipulation action to ROS 2 manipulation interface"""
        return {
            "service": "/manipulation/plan",
            "request": {
                "target_pose": action.parameters.get("target_pose", {}),
                "object_id": action.parameters.get("object_id", "")
            }
        }

    def _map_perception(self, action: ActionSequence) -> Dict[str, Any]:
        """Map perception action to ROS 2 perception interface"""
        return {
            "service": "/perception/detect_objects",
            "request": {
                "camera_name": action.parameters.get("camera", "head_camera"),
                "object_types": action.parameters.get("object_types", [])
            }
        }

    def _map_communication(self, action: ActionSequence) -> Dict[str, Any]:
        """Map communication action to ROS 2 communication interface"""
        return {
            "topic": "/tts/text",
            "message": {
                "text": action.parameters.get("text", ""),
                "voice": action.parameters.get("voice", "default")
            }
        }
```

## Safety and Constraint-Aware Planning

Safety is paramount in humanoid robotics, especially when autonomous systems interpret natural language commands. Constraint-aware planning ensures that all planned actions comply with safety requirements.

### Safety Validation Framework

```python
class SafetyConstraintValidator:
    def __init__(self):
        self.constraint_rules = {
            "navigation": self._validate_navigation_safety,
            "manipulation": self._validate_manipulation_safety,
            "environmental": self._validate_environmental_safety
        }

    def validate_action(self, action: ActionSequence, current_state: Dict[str, Any]) -> bool:
        """Validate an action against safety constraints"""
        if action.type in self.constraint_rules:
            return self.constraint_rules[action.type](action, current_state)
        return True  # If no specific rule, assume safe

    def _validate_navigation_safety(self, action: ActionSequence, state: Dict[str, Any]) -> bool:
        """Validate navigation safety constraints"""
        target_pose = action.parameters.get("pose", {})

        # Check if target is in safe zone
        if not self._is_safe_navigation_target(target_pose, state):
            return False

        # Check path for obstacles
        if not self._is_path_clear(target_pose, state):
            return False

        return True

    def _validate_manipulation_safety(self, action: ActionSequence, state: Dict[str, Any]) -> bool:
        """Validate manipulation safety constraints"""
        target_pose = action.parameters.get("target_pose", {})

        # Check if target is within safe reach
        if not self._is_safe_manipulation_target(target_pose, state):
            return False

        # Check if object is safe to manipulate
        object_id = action.parameters.get("object_id", "")
        if not self._is_safe_to_manipulate(object_id, state):
            return False

        return True

    def _validate_environmental_safety(self, action: ActionSequence, state: Dict[str, Any]) -> bool:
        """Validate environmental safety constraints"""
        # Check environmental conditions
        environment = state.get("environment", {})

        # Check for hazardous conditions
        if environment.get("hazard_detected", False):
            return False

        # Check for safety-critical situations
        if environment.get("emergency", False):
            return False

        return True

    def _is_safe_navigation_target(self, target_pose: Dict, state: Dict) -> bool:
        """Check if navigation target is safe"""
        # Implementation of safety checks
        return True

    def _is_path_clear(self, target_pose: Dict, state: Dict) -> bool:
        """Check if path to target is clear of obstacles"""
        # Implementation of path validation
        return True

    def _is_safe_manipulation_target(self, target_pose: Dict, state: Dict) -> bool:
        """Check if manipulation target is within safe reach"""
        # Implementation of reachability and safety checks
        return True

    def _is_safe_to_manipulate(self, object_id: str, state: Dict) -> bool:
        """Check if object is safe to manipulate"""
        # Implementation of object safety checks
        return True
```

### Integration with ROS 2 Safety Systems

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

class SafeCognitivePlanner(Node):
    def __init__(self):
        super().__init__('safe_cognitive_planner')

        # Publishers and subscribers
        self.plan_publisher = self.create_publisher(
            String,
            'cognitive_plans',
            10
        )

        self.safety_status_sub = self.create_subscription(
            String,
            'safety_status',
            self.safety_callback,
            10
        )

        # Initialize components
        self.planning_system = CognitivePlanningSystem()
        self.safety_validator = SafetyConstraintValidator()
        self.current_state = {}

    def safety_callback(self, msg):
        """Update current state based on safety system"""
        try:
            self.current_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid safety status message")

    def plan_and_execute(self, command: str):
        """Plan and execute command with safety validation"""
        # Decompose task
        task_plan = self.planning_system.decompose_task(command)

        if task_plan.status == "failed":
            self.get_logger().error("Task decomposition failed")
            return

        # Validate safety for each action
        for action in task_plan.actions:
            if not self.safety_validator.validate_action(action, self.current_state):
                self.get_logger().error(f"Action {action.id} failed safety validation")
                # Publish safety violation
                safety_msg = String()
                safety_msg.data = f"safety_violation:{action.id}"
                self.plan_publisher.publish(safety_msg)
                return

        # If all actions pass safety validation, publish the plan
        plan_msg = String()
        plan_msg.data = json.dumps({
            "plan_id": task_plan.id,
            "actions": [action.__dict__ for action in task_plan.actions],
            "original_command": task_plan.original_command
        })

        self.plan_publisher.publish(plan_msg)
        self.get_logger().info(f"Published safe task plan: {task_plan.id}")
```

## Implementation Best Practices

### Error Handling and Fallbacks

```python
class RobustCognitivePlanner:
    def __init__(self):
        self.fallback_strategies = [
            self._use_rule_based_planning,
            self._request_user_clarification,
            self._execute_safe_default
        ]

    def plan_with_fallback(self, command: str):
        """Plan with multiple fallback strategies"""
        try:
            # Try LLM-based planning first
            return self.planning_system.decompose_task(command)
        except Exception as e:
            self.get_logger().warn(f"LLM planning failed: {e}")

            # Try fallback strategies
            for fallback in self.fallback_strategies:
                try:
                    result = fallback(command)
                    if result:
                        return result
                except Exception as fe:
                    self.get_logger().warn(f"Fallback failed: {fe}")

        return self._safe_failure_response(command)
```

### Performance Optimization

- **Caching**: Cache common command patterns and their decompositions
- **Parallel Processing**: Process multiple commands in parallel when safe
- **Model Optimization**: Use appropriate model sizes for real-time requirements
- **Response Validation**: Validate LLM responses before execution

## Summary

In this chapter, we've explored how to use LLMs for cognitive planning in humanoid robotics:

1. Task decomposition using LLMs to convert natural language to action sequences
2. ROS 2 integration patterns for translating commands to executable actions
3. Safety and constraint-aware planning to ensure safe robot operation
4. Best practices for robust and reliable cognitive planning

In the next chapter, we'll combine these components into a complete end-to-end VLA architecture that orchestrates speech, planning, and execution in simulation environments.