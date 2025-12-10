# Quickstart Guide: Physical AI & Humanoid Robotics Textbook - Module 1

**Feature**: Module 1: Foundations & Infrastructure (Chapters 00-03)
**Created**: 2025-12-09
**Target Audience**: Contributors and readers

## Setup for Development

### Prerequisites
- Git installed (Git Bash recommended on Windows)
- Docker Desktop with WSL 2 backend (Windows users)
- VS Code with Remote Containers extension
- NVIDIA GPU with RTX 4080/4090 or similar for simulation
- Minimum: 32GB RAM (64GB recommended)

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-book
```

### 2. Open in DevContainer
```bash
code .
```
Then press `Ctrl+Shift+P` → "Remote-Containers: Reopen in Container"

### 3. Start the Documentation Site
```bash
npm start
```
This will launch the Docusaurus site at http://localhost:3000

## Executing Code Examples

### Prerequisites for Code Execution
- Ensure you're inside the devcontainer (terminal shows container name)
- All ROS 2 Iron and Isaac Sim dependencies are pre-installed

### Running Setup Examples (Chapter 00)
```bash
# Navigate to setup examples
cd src/setup_examples

# Run the first run demo
python first_run_demo.py
```

### Running Locomotion Examples (Chapter 01)
```bash
# Launch Isaac Sim with a humanoid
./isaac-sim.sh assets/scenes/basic_locomotion.usd

# Run the walking controller
ros2 launch locomotion_examples basic_walk_demo.launch.py
```

## Understanding the Structure

### Documentation Organization (docs/)
- `/00-setup/` - Setup and development environment (Chapter 00)
- `/01-foundations/` - Locomotion theory and controllers (Chapter 01) 
- `/02-ros2/` - ROS 2 for humanoid robotics (Chapter 02)
- `/03-modeling/` - Robot modeling with URDF/SRDF/MoveIt 2 (Chapter 03)

### Code Organization (src/)
- `/setup_examples/` - Setup and environment examples
- `/locomotion/` - Bipedal locomotion controllers and examples
- `/modeling_examples/` - Robot modeling tools and examples
- `/ros2_examples/` - ROS 2 specific implementations

### Assets (assets/)
- `/hardware_specs/` - Hardware comparison and requirements
- `/controllers/` - Controller parameters and configurations
- `/models/` - Robot models in various formats (URDF, USD)

## Key Commands and Scripts

### Development
- `npm start` - Start local development server with hot reloading
- `npm run build` - Build static site for production
- `npm run serve` - Serve built site locally for testing
- `npm run pdf` - Generate PDF version of textbook

### Testing
- `colcon test` - Run all ROS 2 packages' tests
- `npm test` - Run frontend unit tests
- `bash scripts/run_module1_demo.sh` - Execute Module 1 demonstrations

### Publication
- `git push origin master` - Push changes to trigger GitHub Pages deployment
- The site automatically updates at https://physical-ai.org

## Troubleshooting

### Common Issues

**Isaac Sim won't start**
- Ensure NVIDIA GPU drivers are up to date
- Check that Docker is running
- Verify CUDA installation with `nvidia-smi`

**Simulation runs too slow**
- Check VRAM: Complex scenes may require 24GB+ for smooth performance
- Use lower-quality settings in Isaac Sim config
- Close other GPU-intensive applications

**Code examples don't execute**
- Make sure you're in the devcontainer
- Run `source install/setup.bash` in the workspace root
- Check that ROS 2 Iron is properly sourced

For additional help, see `/docs/troubleshooting-full.md` or open an issue in the GitHub repository.