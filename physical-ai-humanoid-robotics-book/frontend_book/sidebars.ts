import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Robotics Module',
      items: [
        'modules/ros2-robotics/chapter-1-fundamentals',
        'modules/ros2-robotics/chapter-2-ai-agents',
        'modules/ros2-robotics/chapter-3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Module (Gazebo & Unity)',
      items: [
        'modules/digital-twin/chapter-1-physics-simulation',
        'modules/digital-twin/chapter-2-unity-visualization',
        'modules/digital-twin/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/isaac-ai-brain/chapter-1-isaac-sim',
        'modules/isaac-ai-brain/chapter-2-isaac-ros',
        'modules/isaac-ai-brain/chapter-3-nav2-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Module',
      items: [
        'modules/vla/chapter-1-voice-to-action',
        'modules/vla/chapter-2-cognitive-planning',
        'modules/vla/chapter-3-autonomous-humanoid',
      ],
    },
    // Add more categories and items as needed
  ],
};

export default sidebars;