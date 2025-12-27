# Python AI Examples for Robotics

## Overview

This document provides practical Python code examples that demonstrate how to build intelligent control nodes using rclpy and bridge AI logic to robot controllers.

## Example 1: Simple Obstacle Avoidance AI Agent

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

## Example 2: AI Agent with Basic Path Planning

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import math


class PathPlanningAgent(Node):
    def __init__(self):
        super().__init__('path_planning_agent')

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        # Goal position (x, y)
        self.goal = Point()
        self.goal.x = 5.0
        self.goal.y = 5.0

        # Robot state
        self.current_position = Point()
        self.current_yaw = 0.0

        # AI parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.goal_tolerance = 0.5
        self.obstacle_threshold = 1.0

        self.latest_scan = None
        self.reached_goal = False

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Process odometry data to get robot position and orientation"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

    def calculate_distance_to_goal(self):
        """Calculate Euclidean distance to goal"""
        dx = self.goal.x - self.current_position.x
        dy = self.goal.y - self.current_position.y
        return math.sqrt(dx * dx + dy * dy)

    def calculate_angle_to_goal(self):
        """Calculate angle to goal in robot's frame"""
        dx = self.goal.x - self.current_position.x
        dy = self.goal.y - self.current_position.y
        goal_angle = math.atan2(dy, dx)
        return goal_angle - self.current_yaw

    def ai_decision_callback(self):
        """Path planning AI decision making"""
        if self.latest_scan is None:
            return

        distance_to_goal = self.calculate_distance_to_goal()

        if distance_to_goal < self.goal_tolerance:
            self.reached_goal = True
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)
            self.get_logger().info('Goal reached!')
            return

        # Check for obstacles
        min_distance = min(self.latest_scan.ranges)

        cmd = Twist()

        if min_distance < self.obstacle_threshold:
            # Obstacle detected - turn away
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Obstacle detected - turning')
        else:
            # Move toward goal
            angle_to_goal = self.calculate_angle_to_goal()

            # Proportional controller for angle
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_to_goal))
            cmd.linear.x = self.linear_speed * max(0.0, math.cos(angle_to_goal))

            self.get_logger().info(f'Moving toward goal, angle: {math.degrees(angle_to_goal):.2f}°')

        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    agent = PathPlanningAgent()

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

## Example 3: Integration with Machine Learning

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class MLVisionAgent(Node):
    def __init__(self):
        super().__init__('ml_vision_agent')

        # Create subscription for camera images
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI processing
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # State variables
        self.latest_image = None
        self.object_detected = False
        self.object_position = None  # Normalized position (-1.0 to 1.0)

        self.get_logger().info('ML Vision Agent initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image to detect objects (simplified example)
            self.object_detected, self.object_position = self.detect_object(cv_image)

            # Store the image for potential use in AI processing
            self.latest_image = cv_image

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_object(self, image):
        """Simple object detection using color thresholding"""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    # Normalize position to -1.0 (left) to 1.0 (right)
                    normalized_x = (cx - image.shape[1] / 2) / (image.shape[1] / 2)
                    return True, normalized_x

        return False, None

    def ai_decision_callback(self):
        """AI decision making based on vision input"""
        cmd = Twist()

        if self.object_detected and self.object_position is not None:
            # Object detected - move toward it
            cmd.linear.x = 0.3  # Move forward

            # Turn toward object
            cmd.angular.z = -0.5 * self.object_position  # Negative because of coordinate system

            self.get_logger().info(f'Object detected at position: {self.object_position:.2f}')
        else:
            # No object detected - search pattern
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3  # Slow turn to search

            self.get_logger().info('Searching for object...')

        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    agent = MLVisionAgent()

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

## Example 4: Service-Based AI Agent

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
import random


class ServiceBasedAIAgent(Node):
    def __init__(self):
        super().__init__('service_based_ai_agent')

        # Publisher for commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Service to request AI decisions
        self.decision_service = self.create_service(
            Trigger,
            'request_ai_decision',
            self.decision_callback
        )

        # Publisher for AI status
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        # AI state
        self.ai_enabled = True
        self.behavior_mode = "explore"  # explore, avoid, follow

        self.get_logger().info('Service-Based AI Agent initialized')

    def decision_callback(self, request, response):
        """Handle AI decision requests"""
        if not self.ai_enabled:
            response.success = False
            response.message = "AI agent is disabled"
            return response

        # Generate a decision based on current mode
        cmd = self.generate_decision()
        self.cmd_publisher.publish(cmd)

        # Publish status
        status_msg = String()
        status_msg.data = f"Decision made in {self.behavior_mode} mode"
        self.status_publisher.publish(status_msg)

        response.success = True
        response.message = f"Decision executed: {self.behavior_mode}"
        return response

    def generate_decision(self):
        """Generate a decision based on current behavior mode"""
        cmd = Twist()

        if self.behavior_mode == "explore":
            cmd.linear.x = 0.3
            cmd.angular.z = random.uniform(-0.5, 0.5)
        elif self.behavior_mode == "avoid":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8
        elif self.behavior_mode == "follow":
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        return cmd

    def change_behavior(self, new_mode):
        """Change the AI agent's behavior mode"""
        if new_mode in ["explore", "avoid", "follow"]:
            self.behavior_mode = new_mode
            self.get_logger().info(f'Changed behavior mode to: {new_mode}')
        else:
            self.get_logger().warn(f'Invalid behavior mode: {new_mode}')


def main(args=None):
    rclpy.init(args=args)
    agent = ServiceBasedAIAgent()

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

## Running the Examples

To run these examples:

1. **Create a new ROS 2 package**:
   ```bash
   mkdir -p ~/ros2_ws/src/my_ai_agents
   cd ~/ros2_ws/src/my_ai_agents
   ros2 pkg create --build-type ament_python my_ai_agents
   ```

2. **Copy the example code** into appropriate Python files in your package

3. **Update setup.py** to include the executables:
   ```python
   from setuptools import find_packages, setup

   package_name = 'my_ai_agents'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='AI agents for robotics',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'obstacle_avoidance = my_ai_agents.obstacle_avoidance:main',
               'path_planning = my_ai_agents.path_planning:main',
               'ml_vision = my_ai_agents.ml_vision:main',
               'service_agent = my_ai_agents.service_agent:main',
           ],
       },
   )
   ```

4. **Build and run**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_ai_agents
   source install/setup.bash
   ros2 run my_ai_agents obstacle_avoidance
   ```

These examples demonstrate different approaches to implementing AI agents with rclpy, from simple reactive behaviors to more complex path planning and machine learning integration.