---
sidebar_position: 4
title: "Practical Exercises: VLA System"
---

# VLA Practical Exercises

This section provides hands-on exercises to reinforce the concepts learned in the Vision-Language-Action (VLA) module. These exercises will help you implement and test the complete VLA pipeline.

## Exercise 1: Basic Voice Command Processing

### Objective
Implement a basic voice command processing pipeline that captures audio, converts it to text, and publishes it to a ROS 2 topic.

### Steps
1. Set up the OpenAI Whisper API configuration
2. Create an audio capture node that listens for voice commands
3. Process the audio through Whisper to convert to text
4. Publish the text command to a ROS 2 topic
5. Test with simple commands like "move forward" or "stop"

### Code Template
```python
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import openai
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, 'voice_commands', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Start listening loop
        self.timer = self.create_timer(1.0, self.listen_for_command)

    def listen_for_command(self):
        """Listen for and process voice commands"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                audio = self.recognizer.listen(source, timeout=5.0)
                # Process with Whisper
                result = openai.Audio.transcribe("whisper-1", audio)

                # Publish the command
                msg = String()
                msg.data = result.text
                self.publisher.publish(msg)
                self.get_logger().info(f'Published command: {result.text}')

            except sr.WaitTimeoutError:
                self.get_logger().info('No audio detected')
            except Exception as e:
                self.get_logger().error(f'Error processing audio: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Cognitive Planning with LLMs

### Objective
Create a cognitive planning system that takes natural language commands and decomposes them into action sequences.

### Steps
1. Set up OpenAI API for LLM integration
2. Create a prompt template for task decomposition
3. Implement the task decomposition logic
4. Validate actions against safety constraints
5. Test with complex commands like "Go to the kitchen and bring me a cup"

### Code Template
```python
import openai
import json
from dataclasses import dataclass
from typing import List, Dict, Any

@dataclass
class Action:
    type: str
    parameters: Dict[str, Any]
    dependencies: List[str]

class CognitivePlanner:
    def __init__(self):
        self.client = openai.OpenAI()

    def decompose_task(self, command: str) -> List[Action]:
        """Decompose a natural language command into actions"""
        prompt = f"""
        Convert the following command into a sequence of robot actions:
        Command: "{command}"

        Return a JSON array of actions with type and parameters.
        Valid action types: navigation, manipulation, perception, communication

        Example format:
        [
            {{
                "type": "navigation",
                "parameters": {{"target": "kitchen"}},
                "dependencies": []
            }},
            {{
                "type": "perception",
                "parameters": {{"object": "cup"}},
                "dependencies": ["navigation"]
            }}
        ]
        """

        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)
        actions = []

        for action_data in result:
            action = Action(
                type=action_data["type"],
                parameters=action_data["parameters"],
                dependencies=action_data.get("dependencies", [])
            )
            actions.append(action)

        return actions

# Test the planner
planner = CognitivePlanner()
actions = planner.decompose_task("Go to the kitchen and bring me a red cup")
print(f"Decomposed {len(actions)} actions")
for i, action in enumerate(actions):
    print(f"{i+1}. {action.type}: {action.parameters}")
```

## Exercise 3: Complete VLA Pipeline Integration

### Objective
Integrate voice processing, cognitive planning, and action execution into a complete system.

### Steps
1. Create a VLA orchestrator node
2. Connect voice input to cognitive planning
3. Connect planning output to action execution
4. Implement safety validation between components
5. Test the complete pipeline with end-to-end scenarios

### Code Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VLAOrchestrator(Node):
    def __init__(self):
        super().__init__('vla_orchestrator')

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10
        )

        self.action_status_sub = self.create_subscription(
            String, 'action_status', self.action_status_callback, 10
        )

        # Publishers
        self.plan_pub = self.create_publisher(String, 'action_plans', 10)
        self.status_pub = self.create_publisher(String, 'vla_status', 10)

        # Initialize components
        self.planner = CognitivePlanner()

        # Active plan tracking
        self.active_plan = None
        self.plan_status = {}

    def voice_callback(self, msg: String):
        """Process incoming voice commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Decompose task
        try:
            actions = self.planner.decompose_task(command)

            # Create plan
            plan = {
                "command": command,
                "actions": [
                    {
                        "id": f"action_{i}",
                        "type": action.type,
                        "parameters": action.parameters,
                        "dependencies": action.dependencies
                    }
                    for i, action in enumerate(actions)
                ]
            }

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Track plan
            self.active_plan = plan
            self.plan_status = {action["id"]: "pending" for action in plan["actions"]}

            self.get_logger().info(f'Published plan with {len(actions)} actions')

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def action_status_callback(self, msg: String):
        """Update plan status based on action execution"""
        try:
            status = json.loads(msg.data)
            action_id = status.get("action_id")
            status_value = status.get("status")

            if action_id in self.plan_status:
                self.plan_status[action_id] = status_value

                # Check if plan is complete
                if all(s == "completed" for s in self.plan_status.values()):
                    self.get_logger().info('Plan completed successfully!')
                    self.active_plan = None
                    self.plan_status = {}

        except Exception as e:
            self.get_logger().error(f'Error updating status: {e}')

def main(args=None):
    rclpy.init(args=args)
    orchestrator = VLAOrchestrator()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: Simulation Integration

### Objective
Integrate the VLA system with a simulation environment to test in a safe, controlled setting.

### Steps
1. Set up a basic simulation environment (Gazebo or Isaac Sim)
2. Connect VLA system to simulation state publisher
3. Implement simulation-specific action execution
4. Test complete scenarios in simulation
5. Validate safety constraints in simulated environment

### Testing Scenarios
1. **Simple Navigation**: "Go to the table" - Test basic navigation planning
2. **Object Interaction**: "Pick up the red block" - Test perception and manipulation
3. **Complex Task**: "Go to kitchen, find a cup, and bring it to me" - Test full pipeline
4. **Safety Validation**: "Go near the stairs" - Test constraint validation
5. **Error Recovery**: "Go to unknown location" - Test fallback mechanisms

## Exercise 5: Performance Optimization

### Objective
Optimize the VLA system for real-time performance and reliability.

### Steps
1. Implement caching for common command patterns
2. Add timeout handling for LLM calls
3. Optimize audio processing for real-time performance
4. Implement graceful degradation when components fail
5. Add comprehensive logging and monitoring

### Key Metrics to Track
- Voice processing latency (< 2 seconds)
- Planning response time (< 5 seconds)
- Action execution success rate (> 80%)
- System uptime (> 95% during testing)
- Safety constraint violation rate (< 1%)

## Exercise 6: Advanced Features

### Objective
Extend the VLA system with advanced capabilities.

### Extensions to Implement
1. **Multi-turn Conversations**: Handle follow-up commands and context
2. **Learning from Corrections**: Adapt behavior based on user feedback
3. **Multi-modal Input**: Combine voice with visual or gesture input
4. **Distributed Processing**: Handle processing across multiple nodes
5. **Adaptive Planning**: Modify plans based on real-time feedback

### Assessment Criteria
- Successful implementation of basic voice processing pipeline
- Correct decomposition of natural language into action sequences
- Proper safety validation and constraint checking
- End-to-end functionality demonstration
- Performance optimization and error handling
- Clean, well-documented code following ROS 2 conventions