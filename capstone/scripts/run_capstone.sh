#!/bin/bash
# capstone/scripts/run_capstone.sh

echo "Starting Physical AI & Humanoid Robotics Capstone..."

echo "Step 1: Launch Isaac Sim Kitchen Scene"
echo "./isaac-sim.sh capstone/isaac_sim_scenes/kitchen_conversational_humanoid.usd"
# Uncomment the following line when Isaac Sim is available
# ./isaac-sim.sh capstone/isaac_sim_scenes/kitchen_conversational_humanoid.usd &

echo "Step 2: Build and Launch ROS 2 Stack"
cd capstone/ros2_ws
source /opt/ros/iron/setup.bash
colcon build --packages-select capstone_bringup
source install/setup.bash
ros2 launch capstone_bringup capstone_full.launch.py use_sim:=true

echo "Step 3: Listen for Voice Command"
echo "Speak: 'Good morning. Please tidy the table, throw away the trash, and bring me a fresh water bottle from the fridge.'"

echo "Capstone execution complete!"