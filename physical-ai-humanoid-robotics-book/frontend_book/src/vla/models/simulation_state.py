"""
Simulation State Model for Vision-Language-Action (VLA) System

Based on the data model defined in the VLA specification.
"""
from dataclasses import dataclass
from typing import List, Dict, Any
import time
import uuid


@dataclass
class SimulationState:
    """
    Represents the current state of the simulation environment

    Fields:
    - id: Unique identifier for the simulation state
    - robot_pose: Current position and orientation of the robot
    - environment_objects: List of objects in the environment
    - sensor_data: Current sensor readings
    - timestamp: When the state was captured
    """

    id: str
    robot_pose: Dict[str, float]  # Contains x, y, z, qx, qy, qz, qw values
    environment_objects: List[Dict[str, Any]]
    sensor_data: Dict[str, Any]
    timestamp: str

    def __post_init__(self):
        """Validate the SimulationState instance after initialization"""
        # Validate robot_pose structure
        required_pose_keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        for key in required_pose_keys:
            if key not in self.robot_pose:
                raise ValueError(f"robot_pose must contain '{key}' key")

        # Validate timestamp format (should be ISO 8601)
        # Basic check - in a real system, we'd validate the format more rigorously
        if not self.timestamp or len(self.timestamp) < 10:
            raise ValueError("timestamp must be in ISO 8601 format")

    @classmethod
    def create_initial_state(cls):
        """
        Create an initial simulation state with default values
        """
        return cls(
            id=f"ss_{uuid.uuid4().hex[:8]}",
            robot_pose={
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0
            },
            environment_objects=[],
            sensor_data={},
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        )

    @classmethod
    def create_from_simulation_data(cls, robot_pose: Dict[str, float], environment_objects: List[Dict[str, Any]],
                                   sensor_data: Dict[str, Any]):
        """
        Create a SimulationState instance from simulation data

        Args:
            robot_pose: Current position and orientation of the robot
            environment_objects: List of objects in the environment
            sensor_data: Current sensor readings
        """
        return cls(
            id=f"ss_{uuid.uuid4().hex[:8]}",
            robot_pose=robot_pose,
            environment_objects=environment_objects,
            sensor_data=sensor_data,
            timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        )

    def update_robot_pose(self, new_pose: Dict[str, float]):
        """
        Update the robot pose in the simulation state

        Args:
            new_pose: New position and orientation of the robot
        """
        self.robot_pose = new_pose
        self.timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def add_environment_object(self, obj: Dict[str, Any]):
        """
        Add an object to the environment

        Args:
            obj: Object to add to the environment
        """
        self.environment_objects.append(obj)
        self.timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def update_sensor_data(self, new_sensor_data: Dict[str, Any]):
        """
        Update the sensor data in the simulation state

        Args:
            new_sensor_data: New sensor readings
        """
        self.sensor_data.update(new_sensor_data)
        self.timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def get_object_by_id(self, obj_id: str) -> Dict[str, Any]:
        """
        Get an object from the environment by its ID

        Args:
            obj_id: ID of the object to find

        Returns:
            The object with the specified ID, or None if not found
        """
        for obj in self.environment_objects:
            if obj.get("id") == obj_id:
                return obj
        return None

    def get_objects_by_type(self, obj_type: str) -> List[Dict[str, Any]]:
        """
        Get all objects of a specific type from the environment

        Args:
            obj_type: Type of objects to find

        Returns:
            List of objects of the specified type
        """
        return [obj for obj in self.environment_objects if obj.get("type") == obj_type]


# Example usage:
if __name__ == "__main__":
    # Create an initial simulation state
    initial_state = SimulationState.create_initial_state()
    print(f"Initial State ID: {initial_state.id}")
    print(f"Initial Robot Pose: {initial_state.robot_pose}")
    print(f"Initial Timestamp: {initial_state.timestamp}")

    # Create a simulation state with custom data
    custom_pose = {
        "x": 1.5,
        "y": 2.0,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.1,
        "qw": 0.99
    }

    custom_objects = [
        {"id": "table_1", "type": "furniture", "x": 2.0, "y": 2.0, "z": 0.0},
        {"id": "cup_1", "type": "object", "x": 2.1, "y": 2.1, "z": 0.8}
    ]

    custom_sensors = {
        "camera_data": {"objects_detected": ["cup_1"]},
        "lidar_data": {"obstacles": []},
        "battery_level": 85.0
    }

    state = SimulationState.create_from_simulation_data(
        robot_pose=custom_pose,
        environment_objects=custom_objects,
        sensor_data=custom_sensors
    )

    print(f"\nCustom State ID: {state.id}")
    print(f"Robot Pose: {state.robot_pose}")
    print(f"Environment Objects: {state.environment_objects}")
    print(f"Sensor Data: {state.sensor_data}")

    # Find a specific object
    cup = state.get_object_by_id("cup_1")
    print(f"\nFound cup object: {cup}")

    # Find all objects of a type
    furniture = state.get_objects_by_type("furniture")
    print(f"Furniture objects: {furniture}")