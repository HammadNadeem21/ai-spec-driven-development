# Practical Exercises: Chapter 1 - ROS 2 Fundamentals

## Exercise 1: Basic Publisher-Subscriber

### Objective
Create a simple publisher node that publishes "Hello, ROS 2!" messages to a topic called "chatter", and a subscriber node that listens to this topic.

### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic Python knowledge
- Docusaurus environment set up

### Steps

1. **Create a new ROS 2 package** (if not already done):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_tutorials
   cd my_robot_tutorials
   ```

2. **Create the publisher node** (`my_robot_tutorials/my_robot_tutorials/talker.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class TalkerNode(Node):

       def __init__(self):
           super().__init__('talker')
           self.publisher_ = self.create_publisher(String, 'chatter', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello, ROS 2! {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       talker = TalkerNode()
       rclpy.spin(talker)
       talker.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. **Create the subscriber node** (`my_robot_tutorials/my_robot_tutorials/listener.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class ListenerNode(Node):

       def __init__(self):
           super().__init__('listener')
           self.subscription = self.create_subscription(
               String,
               'chatter',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.data}"')


   def main(args=None):
       rclpy.init(args=args)
       listener = ListenerNode()
       rclpy.spin(listener)
       listener.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

4. **Update the setup.py file** to include the executables:
   ```python
   from setuptools import find_packages, setup

   package_name = 'my_robot_tutorials'

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
       description='Simple ROS 2 publisher/subscriber example',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'talker = my_robot_tutorials.talker:main',
               'listener = my_robot_tutorials.listener:main',
           ],
       },
   )
   ```

5. **Build and run the nodes**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorials
   source install/setup.bash
   ros2 run my_robot_tutorials talker
   # In another terminal:
   ros2 run my_robot_tutorials listener
   ```

### Verification
- Verify that messages are successfully transmitted between nodes
- Use `ros2 topic list` and `ros2 node list` to verify the nodes and topics exist
- Use `ros2 topic echo /chatter` to verify the message content

## Exercise 2: Service Client-Server

### Objective
Create a simple service that adds two integers, with a client that calls this service.

### Steps

1. **Create the service definition** (`my_robot_tutorials/my_robot_tutorials/add_two_ints.srv`):
   ```
   int64 a
   int64 b
   ---
   int64 sum
   ```

2. **Create the service server** (`my_robot_tutorials/my_robot_tutorials/add_server.py`):
   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import AddTwoInts


   class AddTwoIntsServer(Node):

       def __init__(self):
           super().__init__('add_two_ints_server')
           self.srv = self.create_service(
               AddTwoInts,
               'add_two_ints',
               self.add_two_ints_callback
           )

       def add_two_ints_callback(self, request, response):
           response.sum = request.a + request.b
           self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
           return response


   def main(args=None):
       rclpy.init(args=args)
       add_two_ints_server = AddTwoIntsServer()
       rclpy.spin(add_two_ints_server)
       add_two_ints_server.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. **Create the service client** (`my_robot_tutorials/my_robot_tutorials/add_client.py`):
   ```python
   import sys
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import AddTwoInts


   class AddTwoIntsClient(Node):

       def __init__(self):
           super().__init__('add_two_ints_client')
           self.cli = self.create_client(AddTwoInts, 'add_two_ints')
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')
           self.req = AddTwoInts.Request()

       def send_request(self, a, b):
           self.req.a = a
           self.req.b = b
           self.future = self.cli.call_async(self.req)
           rclpy.spin_until_future_complete(self, self.future)
           return self.future.result()


   def main():
       rclpy.init()
       add_two_ints_client = AddTwoIntsClient()
       response = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
       add_two_ints_client.get_logger().info(
           f'Result of add_two_ints: {response.sum}'
       )
       add_two_ints_client.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

4. **Run the service**:
   ```bash
   ros2 run my_robot_tutorials add_server
   # In another terminal:
   ros2 run my_robot_tutorials add_client 2 3
   ```

### Expected Output
The client should return the sum of the two numbers passed as arguments.

## Exercise 3: Exploring ROS 2 Tools

### Objective
Use ROS 2 command-line tools to explore the communication patterns.

### Steps

1. **List all available topics**:
   ```bash
   ros2 topic list
   ```

2. **Get information about a specific topic**:
   ```bash
   ros2 topic info /chatter
   ```

3. **Echo messages on a topic**:
   ```bash
   ros2 topic echo /chatter
   ```

4. **List all available services**:
   ```bash
   ros2 service list
   ```

5. **Call a service directly**:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
   ```

6. **List all running nodes**:
   ```bash
   ros2 node list
   ```

7. **Get information about a specific node**:
   ```bash
   ros2 node info /talker
   ```

## Validation Checklist

After completing these exercises, verify that you can:
- [ ] Create and run a publisher node
- [ ] Create and run a subscriber node
- [ ] Verify message transmission between nodes
- [ ] Create and run a service server
- [ ] Create and run a service client
- [ ] Use ROS 2 command-line tools to explore the system
- [ ] Understand the differences between topics, services, and actions