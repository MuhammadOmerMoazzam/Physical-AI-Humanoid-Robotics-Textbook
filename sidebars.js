// sidebars.js
// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: '00 – Setup & Development Environment',
      link: {
        type: 'doc',
        id: '00-setup/index',
      },
      items: [
        '00-setup/01-hardware',
        '00-setup/02-docker-devcontainer', 
        '00-setup/03-nvidia-isaac-sim',
        '00-setup/04-ros2-iron',
        '00-setup/05-vscode',
        '00-setup/06-first-run',
        '00-setup/07-troubleshooting'
      ],
    },
    {
      type: 'category',
      label: '01 – Foundations of Physical AI & Embodied Intelligence',
      link: {
        type: 'doc',
        id: '01-foundations/index',
      },
      items: [
        '01-foundations/01-intro',
        '01-foundations/02-paradox',
        '01-foundations/03-data-engines',
        '01-foundations/04-humanoid-economics',
        '01-foundations/05-taxonomy',
        '01-foundations/06-timeline',
        '01-foundations/07-platforms-2025',
        '01-foundations/08-summary'
      ],
    },
    {
      type: 'category',
      label: '02 – ROS 2: The Robotic Nervous System',
      link: {
        type: 'doc',
        id: '02-ros2/index',
      },
      items: [
        '02-ros2/01-intro',
        '02-ros2/02-core-concepts',
        '02-ros2/03-rclpy-crash-course',
        '02-ros2/04-package-layout',
        '02-ros2/05-launch-systems',
        '02-ros2/06-safety-nodes',
        '02-ros2/07-isaac-sim-bridge',
        '02-ros2/08-debugging-tools',
        '02-ros2/09-full-example-unitree-g1'
      ],
    },
    {
      type: 'category',
      label: '03 – Modeling Humanoids: URDF, SRDF & MoveIt 2',
      link: {
        type: 'doc',
        id: '03-modeling/index',
      },
      items: [
        '03-modeling/01-intro',
        '03-modeling/02-urdf-fundamentals',
        '03-modeling/03-floating-base-humanoids',
        '03-modeling/04-srdf-and-collision',
        '03-modeling/05-moveit2-setup',
        '03-modeling/06-planning-scene',
        '03-modeling/07-whole-body-planning',
        '03-modeling/08-usd-export',
        '03-modeling/09-full-example-g1'
      ],
    },
    {
      type: 'category',
      label: '04 – Physics Simulation: Gazebo & NVIDIA Isaac Sim',
      link: {
        type: 'doc',
        id: '04-simulation/index',
      },
      items: [
        '04-simulation/01-intro',
        '04-simulation/02-gazebo-vs-isaac-sim',
        '04-simulation/03-usd-workflow',
        '04-simulation/04-domain-randomization',
        '04-simulation/05-isaac-lab',
        '04-simulation/06-synthetic-data-pipeline',
        '04-simulation/07-physics-benchmarks',
        '04-simulation/08-full-humanoid-scene'
      ],
    },
    {
      type: 'category',
      label: '05 – Perception Stack for Humanoids',
      link: {
        type: 'doc',
        id: '05-perception/index',
      },
      items: [
        '05-perception/01-intro',
        '05-perception/02-sensor-suite-2025',
        '05-perception/03-isaac-ros-gems',
        '05-perception/04-stereo-vslam',
        '05-perception/05-3d-scene-reconstruction',
        '05-perception/06-people-and-object-tracking',
        '05-perception/07-realsense-d455-setup',
        '05-perception/08-jetson-deployment',
        '05-perception/09-full-perception-pipeline'
      ],
    },
    {
      type: 'category',
      label: '06 – Bipedal Locomotion & Whole-Body Control',
      link: {
        type: 'doc',
        id: '06-locomotion/index',
      },
      items: [
        '06-locomotion/01-intro',
        '06-locomotion/02-theory-zmp-capture-point',
        '06-locomotion/03-convex-mpc',
        '06-locomotion/04-rl-walking-sota-2025',
        '06-locomotion/05-whole-body-qp',
        '06-locomotion/06-raise-controller',
        '06-locomotion/07-dreamerv3-locomotion',
        '06-locomotion/08-open-controllers-comparison',
        '06-locomotion/09-full-walking-demo'
      ],
    },
    {
      type: 'category',
      label: '07 – Dexterous Manipulation & Grasp Synthesis',
      link: {
        type: 'doc',
        id: '07-manipulation/index',
      },
      items: [
        '07-manipulation/01-intro',
        '07-manipulation/02-hand-kinematics',
        '07-manipulation/03-contact-modeling',
        '07-manipulation/04-isaac-lab-manipulation',
        '07-manipulation/05-diffusion-policies',
        '07-manipulation/06-octo-and-rdt1b',
        '07-manipulation/07-in-hand-reorientation',
        '07-manipulation/08-grasp-synthesis-sota',
        '07-manipulation/09-full-dexterous-demo'
      ],
    },
    {
      type: 'category',
      label: '08 – Vision-Language-Action Models (VLA)',
      link: {
        type: 'doc',
        id: '08-vla/index',
      },
      items: [
        '08-vla/01-intro',
        '08-vla/02-vla-landscape-2025',
        '08-vla/03-openvla',
        '08-vla/04-octo',
        '08-vla/05-rdt1b-vla',
        '08-vla/06-prompt-engineering',
        '08-vla/07-action-chunking',
        '08-vla/08-ros2-vla-bridge',
        '08-vla/09-full-voice-to-action-demo'
      ],
    },
    {
      type: 'category',
      label: '09 – Sim-to-Real Transfer & Domain Randomization',
      link: {
        type: 'doc',
        id: '09-sim2real/index',
      },
      items: [
        '09-sim2real/01-intro',
        '09-sim2real/02-the-gap-is-dead',
        '09-sim2real/03-domain-randomization-2025',
        '09-sim2real/04-system-identification',
        '09-sim2real/05-real2sim-pipeline',
        '09-sim2real/06-actuator-modeling',
        '09-sim2real/07-latency-compensation',
        '09-sim2real/08-2025-success-cases',
        '09-sim2real/09-full-sim2real-deploy'
      ],
    },
    {
      type: 'category',
      label: '10 – Safety, Ethics & Human-Robot Interaction',
      link: {
        type: 'doc',
        id: '10-safety-ethics/index',
      },
      items: [
        '10-safety-ethics/01-intro',
        '10-safety-ethics/02-iso-ts-standards',
        '10-safety-ethics/03-emergency-stop',
        '10-safety-ethics/04-speed-separation',
        '10-safety-ethics/05-force-limiting',
        '10-safety-ethics/06-privacy-and-data',
        '10-safety-ethics/07-bias-fairness',
        '10-safety-ethics/08-regulatory-2025',
        '10-safety-ethics/09-responsible-deployment',
        '10-safety-ethics/10-checklist'
      ],
    },
    {
      type: 'category',
      label: '11 – Capstone: Autonomous Conversational Humanoid',
      link: {
        type: 'doc',
        id: '11-capstone/index',
      },
      items: [
        '11-capstone/01-overview',
        '11-capstone/02-scene-and-task',
        '11-capstone/03-full-pipeline',
        '11-capstone/04-voice-to-action',
        '11-capstone/05-planning-and-control',
        '11-capstone/06-manipulation-sequence',
        '11-capstone/07-safety-wrapper',
        '11-capstone/08-one-click-demo',
        '11-capstone/09-extensions'
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/hardware-guide',
        'appendices/bibliography',
        'appendices/glossary',
        'appendices/license'
      ],
    }
  ],
};

module.exports = sidebars;