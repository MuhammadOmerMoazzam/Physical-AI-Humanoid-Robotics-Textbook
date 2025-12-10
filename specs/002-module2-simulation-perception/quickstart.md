# Quickstart Guide: Physical AI & Humanoid Robotics Textbook - Module 2

**Feature**: Module 2: Simulation & Perception (Chapters 04-05)
**Created**: 2025-12-09
**Target Audience**: Contributors and readers

## Prerequisites Module 1 Completion

Before starting Module 2, ensure you have:
- Completed Module 1 setup (Isaac Sim, ROS 2 Iron, devcontainer)
- Basic understanding of ROS 2 concepts
- Valid Isaac Sim installation with GPU acceleration
- Unitree G1 or similar humanoid model ready

## Setup for Chapter 04 (Simulation)

### 1. Isaac Sim Configuration
```bash
# Verify Isaac Sim is working properly
./isaac-sim.sh --/log/level=info
# Look for: "Isaac Sim initialized successfully"
```

### 2. USD Workflow Setup
```bash
# Install USD tools if not already done
pip install pxr-usd-widgets

# Verify USD export capability
python -c "from pxr import Usd; print('USD OK')"
```

### 3. Domain Randomization Environment
```bash
# Navigate to sim2real directory
cd src/sim2real/domain_randomization
# Verify Python dependencies
pip install -r requirements.txt
```

## Executing Simulation Examples

### Running Physics Simulation (Chapter 04)
```bash
# Terminal 1 - Launch Isaac Sim
./isaac-sim.sh assets/scenes/humanoid_isaac_sim/humanoid_g1.usd

# Terminal 2 - Launch simulation environment
ros2 launch simulation_examples physics_demo.launch.py

# Terminal 3 - Execute domain randomization
python run_domain_rand.py --config assets/scenes/humanoid_isaac_sim/randomization_script.py
```

### Running Perception Examples (Chapter 05)
```bash
# Launch Isaac Sim with perception scene
./isaac-sim.sh assets/scenes/perception_test_area.usd

# Run perception pipeline
ros2 launch perception_examples full_perception_pipeline.launch.py

# Test synthetic data generation
python generate_synthetic_data.py --num-scenes 1000 --output-path /data/synthetic_humanoids
```

## Understanding the Module 2 Structure

### Documentation (docs/)
- `/04-simulation/` - Physics simulation content (Isaac Sim, USD, domain randomization)
- `/05-perception/` - Perception pipeline content (Isaac ROS GEMs, FoundationPose, etc.)

### Code Organization (src/)
- `/sim2real/` - Sim-to-real transfer tools
- `/perception_pipeline/` - Full perception system implementation
- `/isaac_lab_examples/` - Isaac Lab environment implementations
- `/synthetic_data_generation/` - Tools for creating training data

### Assets (assets/)
- `/scenes/` - Isaac Sim scene files (USD format)
- `/synthetic_assets/` - Generated synthetic data and models
- `/perception_models/` - Pre-trained perception model files

## Key Commands and Scripts

### Simulation Development
- `npm run isaac-sim` - Launch Isaac Sim with project scenes
- `ros2 run simulation_examples create_usd_scene` - Generate new USD scenes from URDF
- `python -m sim2real.domain_randomization apply_randomization` - Apply domain randomization to scene

### Perception Testing
- `ros2 launch perception_examples perception_bringup.launch.py` - Launch full perception stack
- `ros2 run perception_examples test_segmentation` - Test semantic segmentation pipeline
- `python -m synthetic_data_generation.create_dataset` - Generate synthetic training data

### Performance Validation
- `python benchmark_simulation.py` - Run physics accuracy benchmarks
- `ros2 run perception_examples evaluate_pipeline` - Evaluate perception pipeline accuracy
- `bash scripts/module2_validation.sh` - Full Module 2 functionality test

## Troubleshooting

### Common Issues

**Isaac Sim performance is low**
- Verify GPU is being used: `nvidia-smi` during simulation
- Check VRAM: Complex scenes may require 24GB+ for smooth performance
- Ensure USD files are properly optimized

**Perception pipeline not detecting objects**
- Verify camera calibration: `ros2 topic echo /camera_info`
- Check lighting conditions in simulation scene
- Validate Isaac ROS bridge connection

**Domain randomization not working**
- Confirm randomization script syntax is correct (Python 3.11 format)
- Check that Isaac Lab is properly configured with the scene

For additional help, see `/docs/module2-troubleshooting-full.md` or open an issue in the GitHub repository.