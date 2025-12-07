# Agent Context - Physical AI & Humanoid Robotics Textbook

## Project Mission
Create a comprehensive, hands-on, simulation-first textbook on Physical AI and humanoid robotics using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) paradigms. The textbook bridges theory to deployable systems through extensive code examples, simulations, and a capstone-style simulated humanoid project.

## Technical Stack
- **Framework**: Docusaurus v3.x with MDX v3
- **Robotics Framework**: ROS 2 Iron (2025-native)
- **Simulation**: NVIDIA Isaac Sim 2024.2+ with Gazebo Harmonic
- **Vision-Language-Action**: OpenVLA, RT-2, Octo, RDT-1B
- **Programming Languages**: Python, C++, YAML for configurations
- **Build System**: Colcon for ROS 2 packages
- **Containerization**: Docker with devcontainers

## Target Audience
Upper-undergraduate to graduate students, researchers, and professionals in computer science, robotics, electrical/mechanical engineering, and AI who want a modern, hands-on, simulation-first textbook on embodied intelligence and humanoid robotics.

## Key Technologies and Concepts

### ROS 2 Ecosystem
- **Nodes, Topics, Services, Actions**: Core ROS 2 communication patterns
- **rclpy**: Python ROS 2 client library for humanoid control
- **Launch Systems**: Python, XML, and composable nodes
- **Parameter Management**: Dynamic reconfiguration and YAML overrides
- **Safety Nodes**: E-stop, watchdog, heartbeat mechanisms

### Physics Simulation
- **Isaac Sim**: USD-based workflows, PhysX 5.x, domain randomization
- **Gazebo Harmonic**: Comparative analysis with Isaac Sim strengths/weaknesses
- **Domain Randomization**: Mass, friction, sensor noise, lighting techniques

### Vision-Language-Action Models
- **OpenVLA**: Open Vision-Language-Action models for robotic manipulation
- **RT-2/RT-X**: Robot Transformer models from Google
- **Octo**: Multimodal robot manipulation from Berkeley
- **RDT-1B**: Reinforcement Diffusion Transformer models

### Humanoid Control Systems
- **Locomotion**: Bipedal walking with ZMP, Capture Point, MPC
- **Manipulation**: Dexterous grasping with 16+ DoF hands
- **Whole-Body Control**: Task-space control with MoveIt 2 integration
- **Sim-to-Real Transfer**: Domain randomization and system identification

## Development Workflow
1. **Specification Phase**: Define chapters and requirements using Spec-Kit Plus
2. **Implementation Phase**: Create Docusaurus content with executable code
3. **Integration Phase**: Connect all components in the capstone project
4. **Testing Phase**: Validate on simulated and real hardware where possible

## Repository Structure
- `docs/` - All Docusaurus Markdown/MDX content organized by chapter
- `src/` - Executable ROS 2 packages, Isaac Lab environments, policies
- `assets/` - URDFs, USDs, calibration files, images
- `static/` - Videos, large PDFs, model weights (git-lfs)
- `capstone/` - Full end-to-end demo scene + launch files
- `.devcontainer/` - 100% reproducible development environment

## Quality Standards
- All code examples must be executable and tested in the devcontainer
- Citations must follow IEEE numeric style with minimum 60% peer-reviewed sources
- All claims must be verifiable and reproducible
- Code must work on the specified hardware requirements
- Zero plagiarism with verification via Copyleaks and manual review

## Specific Requirements
- Minimum 12 chapters with learning objectives and success criteria
- Every chapter includes executable ROS 2 code examples
- Mermaid/PlantUML diagrams for all system architectures
- Embedded videos/GIFs of working simulations
- End-of-chapter exercises with solutions
- Full open-source licensing (MIT/Apache 2.0)
- Complete documentation deployed to GitHub Pages