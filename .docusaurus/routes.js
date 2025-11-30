import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', 'b2f'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/blog/authors/all-sebastien-lorber-articles', '4a1'),
    exact: true
  },
  {
    path: '/blog/authors/yangshun',
    component: ComponentCreator('/blog/authors/yangshun', 'a68'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '89a'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '9ad'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', 'e9f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '704'),
    exact: true
  },
  {
    path: '/blog/tags/facebook',
    component: ComponentCreator('/blog/tags/facebook', '858'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '299'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '00d'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'd2b'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'eab'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '1dd'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'c92'),
            routes: [
              {
                path: '/docs/ai/imitation-learning',
                component: ComponentCreator('/docs/ai/imitation-learning', '9cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai/rl-for-robotics',
                component: ComponentCreator('/docs/ai/rl-for-robotics', '46d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai/training-data',
                component: ComponentCreator('/docs/ai/training-data', '3ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/project-overview',
                component: ComponentCreator('/docs/capstone/project-overview', '483'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/control/pid',
                component: ComponentCreator('/docs/control/pid', 'd09'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/gazebo/gazebo-ros',
                component: ComponentCreator('/docs/gazebo/gazebo-ros', 'c92'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/gazebo/ros2-integration',
                component: ComponentCreator('/docs/gazebo/ros2-integration', 'd0a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/gazebo/sensors-simulation',
                component: ComponentCreator('/docs/gazebo/sensors-simulation', '755'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/gazebo/simulation',
                component: ComponentCreator('/docs/gazebo/simulation', 'f25'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/isaac-basics',
                component: ComponentCreator('/docs/isaac/isaac-basics', '57a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/isaac-nav',
                component: ComponentCreator('/docs/isaac/isaac-nav', '357'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/isaac-ros',
                component: ComponentCreator('/docs/isaac/isaac-ros', 'c48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/isaac-synthetic-data',
                component: ComponentCreator('/docs/isaac/isaac-synthetic-data', '96d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/simulation',
                component: ComponentCreator('/docs/isaac/simulation', 'ed0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac/vision',
                component: ComponentCreator('/docs/isaac/vision', '9b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai/embodied-intelligence',
                component: ComponentCreator('/docs/physical-ai/embodied-intelligence', '3a1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai/sensors',
                component: ComponentCreator('/docs/physical-ai/sensors', '124'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai/what-is-ai',
                component: ComponentCreator('/docs/physical-ai/what-is-ai', '4f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/robotics/control',
                component: ComponentCreator('/docs/robotics/control', 'da2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/robotics/kinematics',
                component: ComponentCreator('/docs/robotics/kinematics', '1ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/actions',
                component: ComponentCreator('/docs/ros2/actions', 'a9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/launch-files',
                component: ComponentCreator('/docs/ros2/launch-files', 'a4d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/navigation2-basics',
                component: ComponentCreator('/docs/ros2/navigation2-basics', 'f01'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/nodes-topics',
                component: ComponentCreator('/docs/ros2/nodes-topics', 'c7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/packages',
                component: ComponentCreator('/docs/ros2/packages', 'ac6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/parameters',
                component: ComponentCreator('/docs/ros2/parameters', '920'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/ros-basics',
                component: ComponentCreator('/docs/ros2/ros-basics', '0cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/rviz',
                component: ComponentCreator('/docs/ros2/rviz', 'ad9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/services',
                component: ComponentCreator('/docs/ros2/services', 'd53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2/urdf',
                component: ComponentCreator('/docs/ros2/urdf', 'e5e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/slam/slam-basics',
                component: ComponentCreator('/docs/slam/slam-basics', '783'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/slam/visual-slam',
                component: ComponentCreator('/docs/slam/visual-slam', 'af9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/system/robot-operating-systems',
                component: ComponentCreator('/docs/system/robot-operating-systems', 'c67'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/vision',
                component: ComponentCreator('/docs/vla/vision', '768'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/vla-basics',
                component: ComponentCreator('/docs/vla/vla-basics', '676'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/vla-robot-control',
                component: ComponentCreator('/docs/vla/vla-robot-control', '0d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
