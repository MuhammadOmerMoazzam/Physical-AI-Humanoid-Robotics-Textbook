# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: Module 4: Intelligence, Transfer & Responsibility
**Created**: 2025-12-09
**Target Audience**: Contributors and readers

## Setup for Development

### Prerequisites
- Git installed (Git Bash recommended on Windows)
- Docker Desktop with WSL 2 backend (Windows users)
- VS Code with Remote Containers extension
- NVIDIA GPU with RTX 4080/4090 or similar for VLA model execution

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
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

### Running VLA Examples (Chapters 8-9)
```bash
# Navigate to VLA examples
cd src/vla/examples

# Run a basic VLA inference
python basic_vla_inference.py --model rdt1b-1.2b --prompt "pick up the red cup"
```

### Running Sim-to-Real Examples (Chapters 9-11)
```bash
# Start Isaac Sim
./isaac-sim.sh

# Launch sim-to-real pipeline
ros2 launch sim2real_example full_pipeline.launch.py
```

## Understanding the Structure

### Documentation Organization (docs/)
- `/08-vla/` - Vision-Language-Action models (Chapter 08)
- `/09-sim2real/` - Sim-to-real transfer (Chapter 09) 
- `/10-safety-ethics/` - Safety and ethical considerations (Chapter 10)
- `/11-capstone/` - Full integration project (Chapter 11)

### Code Organization (src/)
- `/vla/` - Vision-Language-Action model implementations
- `/sim2real/` - Domain randomization and sim-to-real tools
- `/safety/` - Safety protocol implementations
- `/capstone/` - Full autonomous humanoid system

### Assets (assets/)
- `/vla_models/` - Pre-trained VLA model files
- `/calibration/` - Robot dynamics and kinematics parameters

## Building the Full Capstone Project

### 1. Prepare Isaac Sim Environment
```bash
# Launch Isaac Sim with the capstone scene
./isaac-sim.sh assets/scenes/kitchen_conversational_humanoid.usd
```

### 2. Launch ROS 2 Stack
```bash
# Terminal 2: Launch the capstone system
ros2 launch capstone capstone_full.launch.py use_sim:=true
```

### 3. Issue Voice Command
```bash
# Terminal 3: Speak to the robot
python speak_to_robot.py
# Then say: "Please clean the table and bring me a water bottle"
```

## Key Scripts and Commands

### Development
- `npm start` - Start local development server with hot reloading
- `npm run build` - Build static site for production
- `npm run serve` - Serve built site locally for testing
- `npm run pdf` - Generate PDF version of textbook

### Testing
- `colcon test` - Run all ROS 2 packages' tests
- `npm test` - Run frontend unit tests
- `bash scripts/run_capstone_demo.sh` - Execute full capstone demo

### Publication
- `git push origin master` - Push changes to trigger GitHub Pages deployment
- The site automatically updates at https://physical-ai.org

## Troubleshooting

### Common Issues

**Isaac Sim won't start**
- Ensure NVIDIA GPU drivers are up to date
- Check that Docker is running
- Verify CUDA installation with `nvidia-smi`

**VLA models too slow**
- Ensure you're running on RTX 4080/4090 or similar
- Check VRAM: RDT-1B requires 24GB+ for 8-bit quantized

**Code examples don't execute**
- Make sure you're in the devcontainer
- Run `source install/setup.bash` in the workspace root
- Check that ROS 2 Iron is properly sourced

For additional help, see `/docs/troubleshooting-full.md` or open an issue in the GitHub repository.