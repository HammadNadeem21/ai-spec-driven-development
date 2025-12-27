# Data Model: Vision-Language-Action (VLA) Module

**Feature**: 004-vla | **Date**: 2025-12-21

## Overview

Data model for the Vision-Language-Action module focusing on voice command processing, cognitive planning, and autonomous humanoid simulation.

## Core Entities

### VoiceCommand

**Description**: Represents a voice command captured from the user
**Fields**:
- `id`: Unique identifier for the command
- `audio_data`: Raw audio data or reference to audio file
- `transcript`: Text transcript of the voice command
- `confidence`: Confidence score of the speech-to-text conversion
- `timestamp`: When the command was captured
- `language`: Detected language of the command

**Validation rules**:
- `transcript` must not be empty
- `confidence` must be between 0.0 and 1.0
- `timestamp` must be in ISO 8601 format

### TaskPlan

**Description**: Represents a decomposed plan generated from a natural language command
**Fields**:
- `id`: Unique identifier for the plan
- `original_command`: The original voice command text
- `actions`: Array of ordered actions to execute
- `constraints`: Safety and operational constraints
- `status`: Current status of the plan (pending, executing, completed, failed)
- `created_at`: Timestamp when the plan was created

**State transitions**:
- `pending` → `executing` when plan execution begins
- `executing` → `completed` when all actions are finished
- `executing` → `failed` when an action cannot be completed
- `executing` → `interrupted` when safety constraints are violated

### ActionSequence

**Description**: Represents a single action in a task plan that can be executed by the robot
**Fields**:
- `id`: Unique identifier for the action
- `type`: Type of action (navigation, manipulation, perception, etc.)
- `parameters`: Parameters required for the action
- `dependencies`: List of action IDs that must complete first
- `timeout`: Maximum time allowed for the action
- `priority`: Priority level for execution

**Validation rules**:
- `type` must be one of the predefined action types
- `timeout` must be positive

### SafetyConstraint

**Description**: Represents a safety constraint that must be validated before executing actions
**Fields**:
- `id`: Unique identifier for the constraint
- `type`: Type of constraint (environmental, physical, operational)
- `condition`: Condition that must be satisfied
- `severity`: Severity level (warning, error)
- `description`: Human-readable description of the constraint

**Validation rules**:
- `type` must be one of the predefined constraint types
- `severity` must be either 'warning' or 'error'

### SimulationState

**Description**: Represents the current state of the simulation environment
**Fields**:
- `id`: Unique identifier for the simulation state
- `robot_pose`: Current position and orientation of the robot
- `environment_objects`: List of objects in the environment
- `sensor_data`: Current sensor readings
- `timestamp`: When the state was captured

**Validation rules**:
- `robot_pose` must include position (x, y, z) and orientation (quaternion)
- `timestamp` must be in ISO 8601 format

## Relationships

### VoiceCommand → TaskPlan
- One VoiceCommand generates one TaskPlan
- Relationship: 1 to 1 (one-to-one)

### TaskPlan → ActionSequence
- One TaskPlan contains multiple ActionSequences
- Relationship: 1 to many (one-to-many)

### ActionSequence → SafetyConstraint
- One ActionSequence may be subject to multiple SafetyConstraints
- Relationship: many to many (many-to-many)

### TaskPlan → SimulationState
- TaskPlan execution occurs within a SimulationState context
- Relationship: many to one (many-to-one)

## Data Flow

### Voice Processing Flow
1. VoiceCommand is captured with audio_data
2. Audio_data is processed to generate transcript and confidence
3. Transcript is validated and stored

### Planning Flow
1. TaskPlan is created from VoiceCommand transcript
2. Natural language is decomposed into ActionSequences
3. SafetyConstraints are applied to each ActionSequence
4. ActionSequences are ordered based on dependencies

### Execution Flow
1. ActionSequences are executed in dependency order
2. SimulationState is updated after each action
3. SafetyConstraints are validated before each action
4. TaskPlan status is updated based on execution results