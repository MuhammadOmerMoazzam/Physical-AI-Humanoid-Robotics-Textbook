# Chapter 03 Detailed Specification: Modeling Humanoids: URDF, SRDF & MoveIt 2

## Required Sections

### 1. URDF for Floating-Base Humanoids (36–40 DoF)
Complete guide to creating URDF files for complex floating-base humanoid robots with proper joint definitions and kinematic chains.

### 2. SRDF Semantics & Collision Checking
Comprehensive coverage of SRDF files, semantic definitions, and collision checking configurations for humanoid robots.

### 3. MoveIt 2 Setup from Scratch (2025 config generator)
Complete MoveIt 2 configuration process using updated 2025 tools, including setup wizard and configuration files.

### 4. Planning Scene & Dynamic Collision Objects
Implementation of dynamic planning scenes with real-time collision object updates and management.

### 5. Task-Space Control & Interactive Markers
Setup of task-space control systems and interactive markers for intuitive robot control in RViz2.

### 6. Whole-Body Planning with MoveIt 2 + Pilz/CHOMP
Advanced planning techniques using multiple planners with coordination between different robot parts.

### 7. Exporting to USD for Isaac Sim
Complete workflow for exporting humanoid models from URDF/SRDF to USD format for Isaac Sim simulation.

## Mandatory Requirements

**Full open-source URDF + SRDF for a 2025-class humanoid**: Complete model based on Unitree H1 or Figure 02 kinematics with proper licensing and documentation.

## Success Criteria

- **Model Creation**: Reader can create a complete URDF/SRDF for a 30+ DoF humanoid robot
- **MoveIt Configuration**: Reader can configure MoveIt 2 for any humanoid with custom kinematics
- **Simulation Integration**: Reader can export their model to USD and use it in Isaac Sim
- **Planning Performance**: Achieved planning success rates and computation times meet stated benchmarks

## Additional Requirements

- Include proper inertial parameter calculation and validation methods
- Cover both fixed-base and floating-base kinematic configurations
- Address collision checking optimization for real-time applications
- Include validation techniques for URDF/SRDF correctness
- Provide debugging strategies for kinematic and dynamic issues
- Document coordinate frame conventions and best practices

## Quality Standards

- All URDF/SRDF examples must be validated and functional in Gazebo and MoveIt
- Model must include proper visual and collision meshes
- Inertial parameters must be physically realistic and properly computed
- Export workflow to USD must be fully automated and reliable
- All examples must work with the textbook's target simulation environment
- Collision checking must be optimized for real-time performance