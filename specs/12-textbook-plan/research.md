# Research Notes - Physical AI & Humanoid Robotics Textbook

## Chapter 00: Setup & Development Environment
- Hardware requirements based on recent benchmarks (RTX 4070 Ti vs 4090 vs 5090 performance)
- Docker containerization best practices for robotics development
- Isaac Sim 2024.2+ installation and licensing workflows
- ROS 2 Iron installation with all required packages

## Chapter 01: Foundations of Physical AI & Embodied Intelligence
- Moravec's Paradox analysis in the context of foundation models (2023-2025)
- Three data engines: Internet video, teleoperation, simulation
- Economic analysis of humanoid deployment ROI (Goldman Sachs 2025 report)
- Figure 02, Tesla Optimus, Unitree G1/H1 capabilities comparison

## Chapter 02: ROS 2: The Robotic Nervous System
- ROS 2 Iron vs Humble vs Rolling comparison for humanoid applications
- rclpy best practices for humanoid control
- ROS 2 + Isaac Sim bridge architecture
- Safety node implementation patterns

## Chapter 03: Modeling Humanoids: URDF, SRDF & MoveIt 2
- 36-DoF humanoid kinematic chain design
- URDF.xacro best practices for complex robots
- MoveIt 2 configuration for 36+ DoF systems
- USD export workflows for Isaac Sim

## Chapter 04: Physics Simulation: Gazebo & NVIDIA Isaac Sim
- Gazebo Harmonic vs Isaac Sim 2024.2 performance comparison
- USD workflow advantages for humanoid simulation
- Domain randomization techniques and parameters
- Isaac Lab reinforcement learning environments

## Chapter 05: Perception Stack for Humanoids
- Isaac ROS 2 GEMs (AprilTag, Visual SLAM, People Segmentation)
- RealSense D455 vs alternatives (ZED 2i, Azure Kinect) comparison
- FoundationPose and OpenVLA for 6D object pose estimation
- Jetson Orin deployment performance

## Chapter 06: Bipedal Locomotion & Whole-Body Control
- ZMP and Capture Point theory implementation
- Convex MPC walking controllers
- RAISE (2025) and DreamerV3 for humanoid locomotion
- Whole-body QP control for 36+ DoF systems

## Chapter 07: Dexterous Manipulation & Grasp Synthesis
- 16-24 DoF hand kinematics (Allegro, Shadow, Unitree Z1)
- Contact-rich simulation in Isaac Lab
- Diffusion policies and transformer-based grasping
- In-hand reorientation with Octo/RDT-1B

## Chapter 08: Vision-Language-Action Models (VLA)
- OpenVLA, RT-2-X, Octo, π0, RDT-1B comparison
- Fine-tuning workflows on custom datasets
- Action chunking and tokenization techniques
- Integration with ROS 2 control systems

## Chapter 09: Sim-to-Real Transfer & Domain Randomization
- 2025 domain randomization recipes for humanoid success
- System identification for dynamics parameter estimation
- Real→Sim asset pipeline using iPhone LiDAR
- Latency compensation techniques

## Chapter 10: Safety, Ethics & Human-Robot Interaction
- ISO/TS 15066 and ISO 10218 compliance
- Emergency stop and speed/separation monitoring
- EU AI Act and robotics regulations
- Bias audit and fairness evaluation

## Chapter 11: Capstone: Autonomous Conversational Humanoid
- End-to-end integration of all components
- Voice-to-action pipeline with Whisper + RDT-1B
- Safety supervisor implementation
- Real-world deployment considerations