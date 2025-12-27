# Test: Basic Node Communication

## Purpose
This document serves as a validation checklist to verify understanding of basic node communication in ROS 2 after completing Chapter 1.

## Validation Checklist

After completing Chapter 1: ROS 2 Fundamentals, verify that you can:

- [ ] Explain what a ROS 2 node is and its role in the system
- [ ] Describe the difference between topics, services, and actions
- [ ] Set up a basic ROS 2 environment
- [ ] Create a simple publisher node that publishes messages to a topic
- [ ] Create a simple subscriber node that subscribes to messages from a topic
- [ ] Verify that messages are successfully transmitted between nodes
- [ ] Understand the distributed communication model in ROS 2
- [ ] Explain how nodes discover each other in the ROS 2 network

## Practical Exercise

Complete the following practical exercise to validate your understanding:

1. Create a publisher node that publishes "Hello, ROS 2!" messages to a topic called "chatter"
2. Create a subscriber node that listens to the "chatter" topic
3. Run both nodes and verify that messages are transmitted successfully
4. Use `ros2 topic list` and `ros2 node list` to verify the nodes and topics exist
5. Use `ros2 topic echo /chatter` to verify the message content

## Expected Outcome

After completing this validation, you should be able to successfully implement a basic ROS 2 node that communicates with other nodes, demonstrating understanding of the core concepts covered in Chapter 1.