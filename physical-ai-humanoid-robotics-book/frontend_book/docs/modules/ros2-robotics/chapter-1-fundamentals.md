---
sidebar_position: 2
---

# Chapter 1: ROS 2 Fundamentals

## Overview

Welcome to the fundamentals of ROS 2 (Robot Operating System 2). This chapter introduces you to ROS 2 as a robotic nervous system, covering the essential concepts of nodes, topics, services, and actions that form the distributed communication model for robots.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### ROS 2 as a Robotic Nervous System

Think of ROS 2 as the nervous system of a robot. Just as the nervous system allows different parts of a biological organism to communicate and coordinate, ROS 2 allows different components of a robot to communicate and work together.

- **Sensors** (like eyes and ears) send information through the nervous system
- **Processing centers** (like the brain) interpret this information
- **Actuators** (like muscles) execute commands based on processed information

In ROS 2 terms:
- **Nodes** represent different components of the robot
- **Topics** are the communication channels between nodes
- **Messages** carry the actual data

## Core Concepts

### Nodes

A **node** is a process that performs computation. ROS 2 is designed to be modular at the level of a node: a robot control system typically uses multiple nodes to implement things like localization, mapping, path planning, and user interfaces.

Key characteristics of nodes:
- Nodes are the fundamental building blocks of a ROS 2 system
- Each node can perform specific tasks
- Nodes communicate with other nodes using topics, services, or actions
- Multiple nodes can run on the same device or across multiple devices

### Topics and Messages

**Topics** are named buses over which nodes exchange messages. Topics are designed for many-to-many communication: any number of nodes can subscribe to a topic, and any number of nodes can publish to a topic. This allows for flexible network topologies.

**Messages** are the data packets sent via topics. Each message has a specific data structure with typed fields.

Example communication pattern:
```
[Publisher Node] ----(Message)----> [Topic] ----(Message)----> [Subscriber Node]
```

### Services

**Services** provide a request/reply communication pattern. A service client sends a request message and waits for a response. Services are designed for remote procedure calls that have a direct reply.

Service characteristics:
- Request/Response pattern
- Synchronous communication
- One-to-one communication
- Request message and response message types are defined in service description files

### Actions

**Actions** are a more complex communication pattern that combines features of topics and services. Actions are designed for long-running tasks that have the following characteristics:

- Goal: A request that can be accepted or rejected
- Feedback: Optional, sent regularly while the goal is being processed
- Result: A final message indicating the outcome

Actions are ideal for tasks like navigation, where you want to send a goal (navigate to x,y coordinates), receive feedback (current progress), and get a result (success/failure).

## Distributed Communication Model

ROS 2 uses a distributed communication model based on the Data Distribution Service (DDS) standard. This allows nodes to communicate across different machines and operating systems.

### Key Features:

1. **Decentralized**: No master node required (unlike ROS 1)
2. **Multi-platform**: Works across different operating systems
3. **Language Agnostic**: Supports multiple programming languages
4. **Real-time Capable**: Designed for real-time applications
5. **Secure**: Built-in security features

### Communication Patterns in Practice

The distributed model means that nodes can be:
- On the same computer
- On different computers on the same network
- On computers across the internet (with proper configuration)

This flexibility is essential for robotic systems where different components may have different computational requirements or physical locations.

## Practical Exercise

Now that you understand the fundamental concepts, try implementing a basic publisher-subscriber example:

1. Create a publisher node that publishes "Hello, ROS 2!" messages to a topic called "chatter"
2. Create a subscriber node that listens to the "chatter" topic
3. Run both nodes and verify that messages are transmitted successfully
4. Use `ros2 topic list` and `ros2 node list` to verify the nodes and topics exist
5. Use `ros2 topic echo /chatter` to verify the message content

Detailed implementation instructions are available in the [Practical Exercises](./practical-exercises.md) section.

## Summary

This chapter introduced the fundamental concepts of ROS 2:
- Nodes as computational processes
- Topics for many-to-many communication
- Services for request/reply communication
- Actions for long-running tasks with feedback
- The distributed communication model for flexible robot architectures

Understanding these concepts is crucial for working with humanoid robots and connecting AI agents to physical systems, which we'll explore in the following chapters.

## References

For more detailed information about ROS 2 fundamentals, refer to the official ROS 2 documentation:
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Concepts: About ROS 2](https://docs.ros.org/en/humble/Concepts.html)
- [Tutorials: Beginners: CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

## Next Steps

In the next chapter, we'll explore how to use Python and rclpy to build intelligent control nodes that bridge AI logic to robot controllers.