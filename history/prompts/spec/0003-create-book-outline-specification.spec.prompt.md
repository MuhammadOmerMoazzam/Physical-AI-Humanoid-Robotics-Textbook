---
id: 0003
title: create-book-outline-specification
stage: spec
date: 2025-12-07
surface: agent
model: Claude
feature: 3-book-outline
branch: update-constitution-physical-ai-textbook
user: unknown
command: /sp.outline
labels: [specification, outline, chapters, textbook, curriculum]
links:
  spec: specs/3-book-outline/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/3-book-outline/spec.md
 - specs/3-book-outline/checklists/requirements.md
 - specs/3-book-outline/outline.md
tests:
 - none
---

## Prompt

/sp.outline Physical AI & Humanoid Robotics Textbook – High-Level Chapter Descriptions

00 – Setup & Development Environment
Your "zero-excuses" getting-started guide: Ubuntu 22.04 + ROS 2 Humble/Iron, Docker devcontainers, NVIDIA driver & CUDA, Isaac Sim installation, VS Code + ROS extensions, and the exact repo clone & build commands.

01 – Foundations of Physical AI & Embodied Intelligence
Why embodiment changes everything. Historical context (Moravec's paradox → 2025), the rise of foundation models for robotics, data engines, and the economic case for humanoids.

02 – ROS 2: The Robotic Nervous System
From zero to publishing joint commands on a real/simulated humanoid in <200 lines. Nodes, topics, services, actions, lifecycle, rclpy, parameters, launch files, and best practices for large humanoid projects.

03 – Modeling Humanoids: URDF, SRDF & MoveIt 2
Building a complete kinematic tree for a 30+ DoF humanoid. Fixed vs floating base, SRDF semantics, MoveIt 2 setup, planning scenes, and collision checking.

04 – Physics Simulation: Gazebo & NVIDIA Isaac Sim
Side-by-side comparison, then deep dive into Isaac Sim (Omniverse USD workflow, PhysX 5.x, domain randomization, synthetic data pipelines, and extension system).

05 – Perception Stack for Humanoids
Sensor simulation → real sensors. RGB-D, event cameras, LiDAR, IMUs, contact sensors. Isaac ROS GEMs (Stereo V-SLAM, AprilTag, PeopleSemSeg), calibration, and time synchronization.

06 – Bipedal Locomotion & Whole-Body Control
From centroidal dynamics to MPC and reinforcement learning. Zero-moment point, capture point, convex MPC, RL (DreamerV3, PPO on humanoids), and open-source walking controllers that actually work in 2025.

07 – Dexterous Manipulation & Grasp Synthesis
Hand kinematics, contact modeling, Isaac Gym → Isaac Lab, diffusion policies for grasping, in-hand reorientation, and the current best open VLA manipulation models.

08 – Vision-Language-Action Models (VLA)
The 2024–2025 revolution: OpenVLA, RT-2/RT-X, Octo, RDT-1B, π0. Training vs fine-tuning vs zero-shot, prompt engineering for robots, action chunking, and evaluation on RoboMimic / LIBERO benchmarks.

09 – Sim-to-Real Transfer & Domain Randomization
The gap is closing. System identification, dynamics randomization, sensor noise injection, real-to-sim asset pipelines, and case studies of 2025 deployments that actually worked.

10 – Safety, Ethics & Human-Robot Interaction
Force limiting, speed & separation monitoring, ethical frameworks, bias in training data, privacy with always-on perception, and regulatory landscape (EU AI Act implications for humanoids).

11 – Capstone: Autonomous Conversational Humanoid
End-to-end walkthrough you can run tonight: voice → Whisper → LLM planner → OpenVLA → low-level controller → Isaac Sim humanoid that walks, picks, and responds naturally. All code shipped and explained.

Appendices
A. Recommended Hardware 2025 (RTX 4070 Ti → 5090 class, Jetson Orin kits, Unitree G1/H1 access)
B. Complete Docker + devcontainer setup
C. Master bibliography (Zotero → BibTeX)
D. Glossary & acronym list

## Response snapshot

Created comprehensive chapter outline specification with 3 prioritized user stories, 13 functional requirements, and 10 measurable success criteria. Also created the detailed outline.md file with chapter descriptions and learning objectives for all 12 chapters.

## Outcome

- ✅ Impact: Created complete chapter outline for Physical AI textbook with descriptions and objectives for all 12 chapters
- 🧪 Tests: none
- 📁 Files: specs/3-book-outline/spec.md, requirements checklist, and outline.md
- 🔁 Next prompts: /sp.chapter-briefs to create briefs for each chapter
- 🧠 Reflection: The specification captures all essential chapter content with clear learning objectives.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): /sp.outline
- Next experiment (smallest change to try): none needed
