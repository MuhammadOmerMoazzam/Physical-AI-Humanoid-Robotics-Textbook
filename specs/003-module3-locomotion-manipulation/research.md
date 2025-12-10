# Research: Physical AI & Humanoid Robotics Textbook - Module 3

**Feature**: Module 3: Locomotion & Dexterous Manipulation (Chapters 06-07)
**Research Lead**: AI Assistant
**Status**: Completed
**Date**: 2025-12-09

## Completed Research Tasks

### Locomotion Theory Research

**Decision**: Focus on ZMP, Capture Point, and DCM as foundational concepts with modern MPC and RL approaches
**Rationale**: The combination of classical theory and modern learning-based approaches provides the most comprehensive understanding of 2025 humanoid locomotion.
**Additional findings**:
- ZMP remains essential for understanding balance
- Capture Point is critical for push recovery
- DCM formalizes divergent dynamics for stability analysis
- MPC controllers are the practical baseline for most systems
- RL approaches like RAISE provide state-of-the-art performance

### MPC Controller Research

**Decision**: Implement convex MPC approaches as the primary classical controller
**Rationale**: Convex MPC provides real-time performance with predictable behavior, making it ideal for safety-critical walking applications.
**Technical details determined**:
- Quadratic programming formulation for center of mass trajectory
- Preview control with 1.6 second horizon
- Integration with whole-body QP for joint-level control

### RL Walking Research

**Decision**: Cover RAISE and DreamerV3 as 2025 SOTA approaches
**Rationale**: These approaches represent the current state of the art in deep RL for humanoid locomotion, with RAISE specifically designed for humanoid applications.

### Manipulation Strategy Research

**Decision**: Focus on learning-based grasp synthesis combined with classical in-hand manipulation
**Rationale**: Modern approaches using diffusion policies, OpenVLA, and RDT-1B have transformed manipulation from a primarily classical problem to a learning-based one.
**Key approaches identified**:
- FoundationPose for 6D pose estimation without CAD models
- Diffusion Policies for grasp synthesis from visual observations
- In-hand reorientation using learned controllers

## Outstanding Issues

None. All research tasks for Module 3 have been completed and integrated into the implementation plan.