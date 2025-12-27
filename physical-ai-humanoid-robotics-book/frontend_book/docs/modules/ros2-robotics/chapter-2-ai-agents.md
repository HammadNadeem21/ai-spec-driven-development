---
sidebar_position: 3
---

# Chapter 2: Python AI Agents with rclpy

## Overview

This chapter explores how to build intelligent control nodes using Python and rclpy to bridge AI logic to robot controllers. You'll learn about the ROS 2 execution and messaging model, and how to create Python-based AI agents that can control robots through ROS 2 messaging.

## Introduction to rclpy

**rclpy** is the Python client library for ROS 2. It provides a Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, make service calls, and provide services. rclpy is built on top of the ROS 2 client library (rcl) and the DDS (Data Distribution Service) implementation.

### Why Python for AI Agents?

Python is the dominant language in AI and machine learning due to its rich ecosystem of libraries such as TensorFlow, PyTorch, scikit-learn, and OpenCV. Using rclpy allows you to leverage these AI capabilities while seamlessly integrating with the ROS 2 ecosystem for robotics.

## ROS 2 Execution Model

### Node Lifecycle

In ROS 2, nodes have a well-defined lifecycle that includes several states:

1. **Unconfigured**: The node is created but not yet configured
2. **Inactive**: The node is configured but not active
3. **Active**: The node is running and can communicate
4. **Finalized**: The node is shutting down

The lifecycle allows for better resource management and coordination between nodes.

### Threading Model

ROS 2 provides several execution models:

- **Single-threaded executor**: All callbacks are processed sequentially in a single thread
- **Multi-threaded executor**: Callbacks are processed in multiple threads from a thread pool
- **Custom executors**: You can implement your own execution strategies

### Example Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Create publisher and subscriber
        self.publisher = self.create_publisher(String, 'ai_commands', 10)
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Create a timer for periodic AI processing
        self.timer = self.create_timer(0.1, self.ai_processing_callback)

        self.get_logger().info('AI Agent Node initialized')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Process sensor data and make AI decisions
        self.process_sensor_data(msg.data)

    def ai_processing_callback(self):
        # Implement your AI logic here
        ai_decision = self.make_ai_decision()
        if ai_decision:
            self.publish_command(ai_decision)

    def process_sensor_data(self, data):
        # Implement sensor data processing logic
        pass

    def make_ai_decision(self):
        # Implement AI decision-making logic
        return "example_command"

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Building Intelligent Control Nodes

### AI Integration Patterns

When building AI agents for robotics, several patterns are commonly used:

1. **Reactive Agents**: Respond directly to sensor inputs
2. **Deliberative Agents**: Plan actions based on goals and world models
3. **Learning Agents**: Adapt behavior based on experience
4. **Hybrid Agents**: Combine multiple approaches

### Sensor Data Processing

AI agents typically need to process various types of sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import numpy as np


class IntelligentController(Node):
    def __init__(self):
        super().__init__('intelligent_controller')

        # Subscriptions for different sensor types
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.obstacle_detected = False
        self.target_detected = False

    def laser_callback(self, msg):
        # Process laser scan data for obstacle detection
        min_distance = min(msg.ranges)
        if min_distance < 1.0:  # 1 meter threshold
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def camera_callback(self, msg):
        # Process camera data for target detection
        # This is a simplified example
        # In practice, you'd use CV/AI libraries
        self.target_detected = self.detect_target_in_image(msg)

    def detect_target_in_image(self, image_msg):
        # Placeholder for actual image processing
        # This would typically use OpenCV or other CV libraries
        return False

    def make_navigation_decision(self):
        cmd = Twist()

        if self.obstacle_detected:
            # Stop or turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right
        elif self.target_detected:
            # Move toward target
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        else:
            # Continue forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        return cmd

    def publish_control_command(self):
        cmd = self.make_navigation_decision()
        self.cmd_publisher.publish(cmd)
```

## Bridging AI Logic to Robot Controllers

### Integration Strategies

There are several approaches to bridge AI logic with robot controllers:

1. **Direct Integration**: AI logic runs in the same node as the controller
2. **Separate Nodes**: AI and control logic in separate nodes communicating via topics
3. **Service-Based**: AI provides decisions via services
4. **Action-Based**: Long-running AI tasks with feedback

### Example: AI Decision Node with Controller

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool


class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Publisher for AI decisions
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)

        # Subscription for sensor fusion data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_fusion',
            self.sensor_callback,
            10
        )

        # Service to enable/disable AI
        self.enable_service = self.create_service(
            SetBool,
            'enable_ai',
            self.enable_ai_callback
        )

        self.ai_enabled = True
        self.decision_timer = self.create_timer(0.2, self.make_decision)

    def sensor_callback(self, msg):
        # Process sensor fusion data
        self.process_sensor_data(msg.data)

    def process_sensor_data(self, sensor_data):
        # Implement your AI processing logic here
        # This could involve neural networks, planning algorithms, etc.
        pass

    def make_decision(self):
        if not self.ai_enabled:
            return

        # Implement AI decision-making logic
        decision = self.ai_decision_algorithm()

        if decision:
            decision_msg = String()
            decision_msg.data = decision
            self.decision_publisher.publish(decision_msg)

    def ai_decision_algorithm(self):
        # Placeholder for actual AI algorithm
        # This could be a neural network, planning algorithm, etc.
        return "move_forward"

    def enable_ai_callback(self, request, response):
        self.ai_enabled = request.data
        response.success = True
        response.message = f"AI {'enabled' if self.ai_enabled else 'disabled'}"
        return response


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Subscription for AI decisions
        self.decision_subscription = self.create_subscription(
            String,
            'ai_decisions',
            self.decision_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def decision_callback(self, msg):
        # Convert AI decision to robot command
        cmd = self.convert_decision_to_command(msg.data)
        if cmd:
            self.cmd_publisher.publish(cmd)

    def convert_decision_to_command(self, decision):
        cmd = Twist()

        if decision == "move_forward":
            cmd.linear.x = 0.5
        elif decision == "turn_left":
            cmd.angular.z = 0.5
        elif decision == "turn_right":
            cmd.angular.z = -0.5
        elif decision == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Unknown decision, stop for safety
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd
```

## Best Practices for AI Agents

### Performance Considerations

1. **Efficient Message Handling**: Process messages efficiently to avoid bottlenecks
2. **Resource Management**: Monitor CPU and memory usage of AI algorithms
3. **Real-time Constraints**: Consider timing requirements for robot control
4. **Threading**: Use appropriate threading models for your application

### Safety and Reliability

1. **Fail-Safe Mechanisms**: Implement fallback behaviors when AI fails
2. **Validation**: Validate AI outputs before sending to robot controllers
3. **Monitoring**: Monitor AI agent performance and decision quality
4. **Logging**: Log AI decisions for debugging and analysis

## Practical Examples

Now that you understand the concepts, here are some practical Python examples that demonstrate implementing AI agents:

### Simple Obstacle Avoidance Agent

The following example shows a basic AI agent that avoids obstacles using laser scan data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_agent')

        # Create subscription for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        # Parameters
        self.min_distance_threshold = 1.0  # meters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.8  # rad/s

        self.latest_scan = None
        self.get_logger().info('Obstacle Avoidance Agent initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.latest_scan = msg

    def ai_decision_callback(self):
        """Main AI decision-making function"""
        if self.latest_scan is None:
            return

        # Process scan data to detect obstacles
        min_distance = min(self.latest_scan.ranges)

        cmd = Twist()

        if min_distance < self.min_distance_threshold:
            # Obstacle detected - turn away
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Obstacle detected - turning')
        else:
            # Path clear - move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Path clear - moving forward')

        # Publish command
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    agent = ObstacleAvoidanceAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

More complete examples, including path planning and machine learning integration, are available in the [Python AI Examples](./python-ai-examples.md) section.

## Practical Exercise

Implement an AI agent that:

1. Subscribes to sensor data (laser scan or camera feed)
2. Implements a simple AI algorithm (e.g., obstacle avoidance, path planning)
3. Publishes commands to control a simulated robot
4. Includes error handling and safety checks

Detailed implementation instructions are available in the [Practical Exercises](./practical-exercises.md) section.

## Summary

This chapter covered:
- The rclpy Python client library for ROS 2
- ROS 2 execution and messaging models
- How to build intelligent control nodes with Python
- Strategies for bridging AI logic to robot controllers
- Best practices for creating robust AI agents

These concepts are essential for creating AI-driven robotic systems that can operate autonomously while maintaining safety and reliability.

## References

For more detailed information about Python AI agents with ROS 2, refer to the official ROS 2 documentation:
- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [Tutorials: Python Client Library](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/api/)

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format) for modeling humanoid robots, including links, joints, coordinate frames, and integration with ROS 2 and simulators.