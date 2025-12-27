# Quickstart Guide: Physical AI Humanoid Robotics Book

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Python 3.8+ with ROS 2 Humble Hawksbill installed (for examples)
- Basic knowledge of command line operations

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Navigate to the frontend book directory**
   ```bash
   cd frontend_book
   ```

3. **Install Docusaurus dependencies**
   ```bash
   npm install
   # OR
   yarn install
   ```

4. **Verify ROS 2 installation (for running examples)**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 --version
   ```

5. **Start the development server**
   ```bash
   npm run start
   # OR
   yarn start
   ```

6. **Access the documentation**
   Open your browser to `http://localhost:3000` to view the Physical AI Humanoid Robotics Book.

## Building for Production

```bash
npm run build
# OR
yarn build
```

The built site will be available in the `build/` directory and can be deployed to any static hosting service.

## Running the Examples

The ROS 2 examples in the documentation require a working ROS 2 environment. Follow the official ROS 2 installation guide for your platform before attempting to run the code examples.

## Troubleshooting

- If you encounter issues with ROS 2 commands, ensure your ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
- If Docusaurus fails to start, check that Node.js and npm are properly installed
- For Python-related issues in ROS 2 examples, verify that the ROS 2 Python packages are installed