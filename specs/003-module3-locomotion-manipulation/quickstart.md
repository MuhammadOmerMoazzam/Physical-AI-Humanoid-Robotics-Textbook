# Quickstart Guide: Physical AI & Humanoid Robotics Textbook - Module 3

**Feature**: Module 3: Locomotion & Dexterous Manipulation (Chapters 06-07)
**Created**: 2025-12-09
**Target Audience**: Contributors and readers

## Prerequisites: Complete Modules 1 & 2

Before starting Module 3, ensure you have:
- Completed Module 1 (foundations & setup)
- Completed Module 2 (simulation & perception)
- Unitree G1 or equivalent humanoid model ready
- Isaac Sim properly configured with ROS 2 bridge
- Basic ROS 2 Iron knowledge

## Setup for Chapter 06 (Locomotion)

### 1. Install Locomotion Dependencies
```bash
# Install MPC and whole-body control libraries
pip install crocoddyl-torch towr-python

# Install Isaac Lab locomotion environments
git submodule update --init --recursive src/isaac_lab_locomotion/
```

### 2. Download RAISE Controller
```bash
pip install raise-controller==2025.12
```

### 3. Verify Simulation Environment
```bash
# Test Isaac Sim with basic walking scene
./isaac-sim.sh assets/scenes/basic_locomotion.usd
```

## Executing Locomotion Examples

### Running MPC Walking Controller
```bash
# Terminal 1 - Launch Isaac Sim with humanoid
./isaac-sim.sh assets/scenes/humanoid_locomotion.usd

# Terminal 2 - Launch MPC walking controller
ros2 launch locmotion mpc_walking.launch.py robot:=g1 terrain:=flat

# Terminal 3 - Send walking commands
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

### Running RL-Based Walking (RAISE)
```bash
# Launch with RAISE controller
ros2 launch locmotion raise_walking.launch.py robot:=g1 policy:=robust

# The policy will adapt to terrain automatically
```

## Executing Manipulation Examples

### Running Dexterous Manipulation
```bash
# Terminal 1 - Launch Isaac Sim with manipulation scene
./isaac-sim.sh assets/scenes/humanoid_manipulation.usd

# Terminal 2 - Launch manipulation pipeline
ros2 launch manipulation manipulation_bringup.launch.py robot:=g1 task:=grasp_and_place

# Terminal 3 - Send manipulation commands
ros2 topic pub /manipulation_goal std_msgs/String '{data: "grasp the red cup and place on table"}'
```

## Understanding the Module 3 Structure

### Documentation (docs/)
- `/06-locomotion/` - Bipedal locomotion content (MPC, RL, controllers)
- `/07-manipulation/` - Dexterous manipulation content (grasping, in-hand manipulation)

### Code Organization (src/)
- `/locomotion/` - Walking controller implementations
- `/manipulation/` - Grasping and manipulation implementations
- `/whole_body_control/` - Integration of locomotion and manipulation

### Assets (assets/)
- `/controllers/` - Pre-trained controller parameters and configurations
- `/locomotion_scenes/` - Isaac Sim locomotion test environments
- `/manipulation_scenes/` - Isaac Sim manipulation test environments

## Key Commands and Scripts

### Locomotion Development
- `ros2 launch locmotion test_all_controllers.launch.py` - Test all walking controllers
- `python benchmark_walking.py` - Run walking performance benchmarks
- `ros2 run locmotion visualize_capture_point` - Visualize capture point dynamics

### Manipulation Testing
- `ros2 launch manipulation test_grasping.launch.py` - Run grasping experiments
- `python evaluate_manipulation_success.py` - Assess manipulation performance
- `ros2 run manipulation visualize_grasp_approaches` - Show optimal grasp approaches

### Integration Validation
- `bash scripts/module3_integration_test.sh` - Full locomotion + manipulation pipeline test
- `python compare_controller_performance.py` - Compare different walking controllers
- `ros2 launch integration_test full_humanoid_demo.launch.py` - End-to-end humanoid demo

## Troubleshooting

### Common Issues

**Walking controller unstable**
- Check CoM initialization: Ensure robot is balanced before starting
- Verify ZMP calculation: Check that feet are properly contacting ground
- Adjust MPC parameters for specific robot dynamics

**Grasping fails repeatedly**
- Verify camera calibration: Use `ros2 run camera_calibration mono` to recalibrate
- Check lighting conditions: Manipulation requires good illumination
- Validate end-effector configuration: Confirm gripper/hand parameters are correct

**Whole-body control conflicts**
- Check for joint limit violations in trajectory planning
- Verify force control vs position control modes
- Confirm proper task prioritization in the QP controller

For additional help, see `/docs/module3-troubleshooting-full.md` or open an issue in the GitHub repository.