# Quickstart Guide - Physical AI & Humanoid Robotics Textbook

## Prerequisites

### Hardware (Minimum Recommended)
- **CPU**: 12th Gen Intel i7 or AMD Ryzen 7 5800X
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or better
- **RAM**: 64 GB DDR5
- **Storage**: 2 TB NVMe SSD

### Software
- **OS**: Ubuntu 22.04 LTS (64-bit)
- **Docker**: Version 20.10 or newer
- **ROS 2**: Iron Irwini (native Ubuntu 22.04 package)
- **NVIDIA Drivers**: 535.129.03 or newer
- **CUDA**: 12.4 or newer

## Installation and Setup

### 1. Clone the Repository
```bash
git clone https://github.com/MuhammadOmerMoazzam/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Set up Docker Environment
```bash
# Use the provided devcontainer
code .
# When prompted, reopen in container
```

### 3. Install Isaac Sim
```bash
# Download Isaac Sim 2024.2+ from NVIDIA Developer portal
# Install to default location
# Activate development license (free for academic use)
```

### 4. Install ROS 2 Iron
```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Iron
sudo apt update
sudo apt install ros-iron-desktop ros-dev-tools python3-colcon-common-extensions
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Isaac ROS 2 Bridge
```bash
pip install "omni.isaac.ros2_bridge==2024.2.1"
```

### 6. Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

## Running the Book Content

### 1. Start the Website Locally
```bash
# Install Node.js dependencies
npm install

# Start the development server
npm start
```

### 2. Run a Chapter Example
```bash
# Navigate to the chapter's ROS 2 package
cd src/chapter-specific-package

# Build and run the example
colcon build --packages-select specific_example
source install/setup.bash
ros2 launch specific_example example.launch.py
```

### 3. Run Isaac Sim Integration
```bash
# Launch Isaac Sim with the textbook scene
./isaac-sim.sh --/app/window/fullscreen=false assets/scenes/kitchen_conversational_humanoid.usd

# In another terminal, run the ROS 2 bridge
ros2 launch ros2_humanoid_template spawn_humanoid.launch.py
```

## Running the Capstone Project

### 1. Start the Isaac Sim Environment
```bash
cd capstone
./isaac-sim.sh isaac_sim_scenes/kitchen_conversational_humanoid.usd
```

### 2. Launch the Full ROS 2 Stack
```bash
# Terminal 1
ros2 launch capstone_bringup capstone_full.launch.py use_sim:=true
```

### 3. Interact with the Humanoid
```bash
# Terminal 2
python3 capstone/scripts/voice_input_demo.py
# Speak: "Please tidy the table and bring me the water bottle"
```

## Development Guidelines

### Code Structure
- Code examples are located in `src/` directory
- Each chapter has its own ROS 2 package
- Simulation assets are in `assets/`
- Website content is in `docs/`

### Testing
- All code examples are tested in the devcontainer
- Use `colcon test` to run unit tests
- Integration tests are in the capstone section

### Contributing
- Follow the branching strategy: `feature/chapter-number-short-description`
- Submit pull requests to the `development` branch
- Ensure all tests pass before submitting