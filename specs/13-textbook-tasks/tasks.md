# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/12-textbook-plan/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md
**Status**: Ready for implementation

**Tasks**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

## Format: `[ID] [P?] [Story] Description with file path`

- [ ] [T001] [P] Create project structure per implementation plan
- [ ] [T002] [P] Initialize Docusaurus project with v3.5+ 
- [ ] [T003] [P] Set up GitHub Pages deployment configuration
- [ ] [T004] [P] Configure devcontainer for reproducible development environment
- [ ] [T005] [P] Add required dependencies to package.json

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] [T001] Create project structure per implementation plan
- [ ] [T002] Install and configure Docusaurus v3.5+ with classic preset
- [ ] [T003] [P] Configure GitHub Pages deployment settings

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY chapter can be implemented

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [ ] [T004] Set up devcontainer configuration for 100% reproducible environment
- [ ] [T005] [P] Configure docusaurus.config.js with proper site settings
- [ ] [T006] [P] Set up sidebars.js with all chapter categories (00-11)
- [ ] [T007] [P] Install and configure required dependencies (MDX, React, PDF plugin)
- [ ] [T008] Configure custom CSS and component structure
- [ ] [T009] Set up documentation structure (docs/00-setup/ through docs/11-capstone/)
- [ ] [T010] [P] Configure static assets directory for videos and large files

**Checkpoint**: Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: Chapter 00 - Setup & Development Environment (Priority: P1) 🎯 MVP

**Goal**: Complete the setup chapter with hardware requirements, Docker setup, ROS 2 installation, and first run

**Independent Test**: Complete hardware setup guide with verified ROS 2 and Isaac Sim installations

### Tests for Chapter 00 (OPTIONAL - only if tests requested) ⚠️

- [ ] [T011] [P] [US1] Unit tests for setup scripts in 00-setup/

### Implementation for Chapter 00

- [ ] [T012] [P] [US1] Create hardware requirements documentation in docs/00-setup/01-hardware.mdx
- [ ] [T013] [P] [US1] Create Docker + devcontainer setup guide in docs/00-setup/02-docker-devcontainer.mdx
- [ ] [T014] [US1] Create NVIDIA Isaac Sim installation guide in docs/00-setup/03-nvidia-isaac-sim.mdx
- [ ] [T015] [US1] Create ROS 2 Iron installation guide in docs/00-setup/04-ros2-iron.mdx
- [ ] [T016] [US1] Create VS Code configuration guide in docs/00-setup/05-vscode.mdx
- [ ] [T017] [US1] Create first run tutorial in docs/00-setup/06-first-run.mdx
- [ ] [T018] [US1] Create troubleshooting guide in docs/00-setup/07-troubleshooting.mdx
- [ ] [T019] [US1] Create _category_.json for 00-setup navigation

**Checkpoint**: At this point, Chapter 00 should be fully functional and testable independently

---

## Phase 4: Chapter 01 - Foundations of Physical AI & Embodied Intelligence (Priority: P2)

**Goal**: Complete foundational concepts with Moravec's paradox, data engines, and humanoid economics

**Independent Test**: Readers can explain why 2025 is the inflection point for humanoid robotics

### Implementation for Chapter 01

- [ ] [T020] [P] [US2] Create introduction and learning objectives in docs/01-foundations/01-intro.mdx
- [ ] [T021] [P] [US2] Create Moravec's paradox section in docs/01-foundations/02-paradox.mdx
- [ ] [T022] [US2] Create data engines section in docs/01-foundations/03-data-engines.mdx
- [ ] [T023] [US2] Create humanoid economics section in docs/01-foundations/04-humanoid-economics.mdx
- [ ] [T024] [US2] Create taxonomy section in docs/01-foundations/05-taxonomy.mdx
- [ ] [T025] [US2] Create timeline section in docs/01-foundations/06-timeline.mdx
- [ ] [T026] [US2] Create platform comparison in docs/01-foundations/07-platforms-2025.mdx
- [ ] [T027] [US2] Create summary section in docs/01-foundations/08-summary.mdx
- [ ] [T028] [US2] Create _category_.json for 01-foundations navigation

**Checkpoint**: At this point, Chapter 01 should be fully functional and testable independently

---

## Phase 5: Chapter 02 - ROS 2: The Robotic Nervous System (Priority: P3)

**Goal**: Complete ROS 2 fundamentals with core concepts, rclpy, and Isaac Sim bridge

**Independent Test**: Readers can create and run a basic ROS 2 node for humanoid control

### Implementation for Chapter 02

- [ ] [T029] [P] [US3] Create introduction and learning objectives in docs/02-ros2/01-intro.mdx
- [ ] [T030] [P] [US3] Create core concepts section in docs/02-ros2/02-core-concepts.mdx
- [ ] [T031] [US3] Create rclpy crash course in docs/02-ros2/03-rclpy-crash-course.mdx
- [ ] [T032] [US3] Create package layout section in docs/02-ros2/04-package-layout.mdx
- [ ] [T033] [US3] Create launch systems section in docs/02-ros2/05-launch-systems.mdx
- [ ] [T034] [US3] Create safety nodes section in docs/02-ros2/06-safety-nodes.mdx
- [ ] [T035] [US3] Create Isaac Sim bridge section in docs/02-ros2/07-isaac-sim-bridge.mdx
- [ ] [T036] [US3] Create debugging tools section in docs/02-ros2/08-debugging-tools.mdx
- [ ] [T037] [US3] Create full example section in docs/02-ros2/09-full-example-unitree-g1.mdx
- [ ] [T038] [US3] Create _category_.json for 02-ros2 navigation
- [ ] [T039] [US3] Create ROS 2 package template in src/ros2_humanoid_template/

**Checkpoint**: At this point, Chapter 02 should be fully functional and testable independently

---

## Phase 6: Chapter 03 - Modeling Humanoids: URDF, SRDF & MoveIt 2 (Priority: P4)

**Goal**: Complete humanoid modeling with kinematic trees and MoveIt 2

**Independent Test**: Readers can create a complete URDF/SRDF for a 36-DoF humanoid model

### Implementation for Chapter 03

- [ ] [T040] [P] [US4] Create introduction and learning objectives in docs/03-modeling/01-intro.mdx
- [ ] [T041] [P] [US4] Create URDF fundamentals section in docs/03-modeling/02-urdf-fundamentals.mdx
- [ ] [T042] [US4] Create floating base section in docs/03-modeling/03-floating-base-humanoids.mdx
- [ ] [T043] [US4] Create SRDF section in docs/03-modeling/04-srdf-and-collision.mdx
- [ ] [T044] [US4] Create MoveIt 2 setup section in docs/03-modeling/05-moveit2-setup.mdx
- [ ] [T045] [US4] Create planning scene section in docs/03-modeling/06-planning-scene.mdx
- [ ] [T046] [US4] Create whole body planning section in docs/03-modeling/07-whole-body-planning.mdx
- [ ] [T047] [US4] Create USD export section in docs/03-modeling/08-usd-export.mdx
- [ ] [T048] [US4] Create full example section in docs/03-modeling/09-full-example-g1.mdx
- [ ] [T049] [US4] Create _category_.json for 03-modeling navigation
- [ ] [T050] [US4] Create sample URDF files in assets/models/humanoid_g1/urdf/

**Checkpoint**: At this point, Chapter 03 should be fully functional and testable independently

---

## Phase 7: Chapter 04 - Physics Simulation: Gazebo & NVIDIA Isaac Sim (Priority: P5)

**Goal**: Complete simulation with Isaac Sim setup and domain randomization

**Independent Test**: Readers can launch Isaac Sim with a humanoid scene

### Implementation for Chapter 04

- [ ] [T051] [P] [US5] Create introduction and learning objectives in docs/04-simulation/01-intro.mdx
- [ ] [T052] [P] [US5] Create Gazebo vs Isaac Sim comparison in docs/04-simulation/02-gazebo-vs-isaac-sim.mdx
- [ ] [T053] [US5] Create USD workflow section in docs/04-simulation/03-usd-workflow.mdx
- [ ] [T054] [US5] Create domain randomization section in docs/04-simulation/04-domain-randomization.mdx
- [ ] [T055] [US5] Create Isaac Lab section in docs/04-simulation/05-isaac-lab.mdx
- [ ] [T056] [US5] Create synthetic data pipeline section in docs/04-simulation/06-synthetic-data-pipeline.mdx
- [ ] [T057] [US5] Create physics benchmarks section in docs/04-simulation/07-physics-benchmarks.mdx
- [ ] [T058] [US5] Create full scene section in docs/04-simulation/08-full-humanoid-scene.mdx
- [ ] [T059] [US5] Create _category_.json for 04-simulation navigation
- [ ] [T060] [US5] Create Isaac Sim scenes in assets/scenes/humanoid_isaac_sim/

**Checkpoint**: At this point, Chapter 04 should be fully functional and testable independently

---

## Phase 8: Chapter 05 - Perception Stack for Humanoids (Priority: P6)

**Goal**: Complete perception stack with Isaac ROS 2 GEMs and real-time processing

**Independent Test**: Readers can run Isaac ROS 2 GEMs for perception tasks

### Implementation for Chapter 05

- [ ] [T061] [P] [US6] Create introduction and learning objectives in docs/05-perception/01-intro.mdx
- [ ] [T062] [P] [US6] Create sensor suite section in docs/05-perception/02-sensor-suite-2025.mdx
- [ ] [T063] [US6] Create Isaac ROS GEMs section in docs/05-perception/03-isaac-ros-gems.mdx
- [ ] [T064] [US6] Create stereo VSLAM section in docs/05-perception/04-stereo-vslam.mdx
- [ ] [T065] [US6] Create 3D reconstruction section in docs/05-perception/05-3d-scene-reconstruction.mdx
- [ ] [T066] [US6] Create tracking section in docs/05-perception/06-people-and-object-tracking.mdx
- [ ] [T067] [US6] Create RealSense setup section in docs/05-perception/07-realsense-d455-setup.mdx
- [ ] [T068] [US6] Create Jetson deployment section in docs/05-perception/08-jetson-deployment.mdx
- [ ] [T069] [US6] Create full pipeline section in docs/05-perception/09-full-perception-pipeline.mdx
- [ ] [T070] [US6] Create _category_.json for 05-perception navigation
- [ ] [T071] [US6] Create perception pipeline in src/perception_pipeline/

**Checkpoint**: At this point, Chapter 05 should be fully functional and testable independently

---

## Phase 9: Chapter 06 - Bipedal Locomotion & Whole-Body Control (Priority: P7)

**Goal**: Complete walking controllers with MPC and RL approaches

**Independent Test**: Readers can implement a walking controller for a humanoid

### Implementation for Chapter 06

- [ ] [T072] [P] [US7] Create introduction and learning objectives in docs/06-locomotion/01-intro.mdx
- [ ] [T073] [P] [US7] Create ZMP/Capture Point theory in docs/06-locomotion/02-theory-zmp-capture-point.mdx
- [ ] [T074] [US7] Create convex MPC section in docs/06-locomotion/03-convex-mpc.mdx
- [ ] [T075] [US7] Create RL walking section in docs/06-locomotion/04-rl-walking-sota-2025.mdx
- [ ] [T076] [US7] Create whole-body QP section in docs/06-locomotion/05-whole-body-qp.mdx
- [ ] [T077] [US7] Create RAISE controller section in docs/06-locomotion/06-raise-controller.mdx
- [ ] [T078] [US7] Create DreamerV3 section in docs/06-locomotion/07-dreamerv3-locomotion.mdx
- [ ] [T079] [US7] Create controller comparison in docs/06-locomotion/08-open-controllers-comparison.mdx
- [ ] [T080] [US7] Create full walking demo in docs/06-locomotion/09-full-walking-demo.mdx
- [ ] [T081] [US7] Create _category_.json for 06-locomotion navigation
- [ ] [T082] [US7] Create locomotion controllers in src/locomotion/

**Checkpoint**: At this point, Chapter 06 should be fully functional and testable independently

---

## Phase 10: Chapter 07 - Dexterous Manipulation & Grasp Synthesis (Priority: P8)

**Goal**: Complete manipulation with dexterous hands and grasp synthesis

**Independent Test**: Readers can implement grasping with 16+ DoF hands

### Implementation for Chapter 07

- [ ] [T083] [P] [US8] Create introduction and learning objectives in docs/07-manipulation/01-intro.mdx
- [ ] [T084] [P] [US8] Create hand kinematics section in docs/07-manipulation/02-hand-kinematics.mdx
- [ ] [T085] [US8] Create contact modeling section in docs/07-manipulation/03-contact-modeling.mdx
- [ ] [T086] [US8] Create Isaac Lab manipulation in docs/07-manipulation/04-isaac-lab-manipulation.mdx
- [ ] [T087] [US8] Create diffusion policies section in docs/07-manipulation/05-diffusion-policies.mdx
- [ ] [T088] [US8] Create Octo and RDT1B section in docs/07-manipulation/06-octo-and-rdt1b.mdx
- [ ] [T089] [US8] Create in-hand reorientation in docs/07-manipulation/07-in-hand-reorientation.mdx
- [ ] [T090] [US8] Create grasp synthesis section in docs/07-manipulation/08-grasp-synthesis-sota.mdx
- [ ] [T091] [US8] Create full dexterous demo in docs/07-manipulation/09-full-dexterous-demo.mdx
- [ ] [T092] [US8] Create _category_.json for 07-manipulation navigation
- [ ] [T093] [US8] Create manipulation policies in src/manipulation/

**Checkpoint**: At this point, Chapter 07 should be fully functional and testable independently

---

## Phase 11: Chapter 08 - Vision-Language-Action Models (VLA) (Priority: P9)

**Goal**: Complete VLA models with OpenVLA, RT-2, Octo, and RDT-1B

**Independent Test**: Readers can run VLA models for robot tasks

### Implementation for Chapter 08

- [ ] [T094] [P] [US9] Create introduction and learning objectives in docs/08-vla/01-intro.mdx
- [ ] [T095] [P] [US9] Create VLA landscape in docs/08-vla/02-vla-landscape-2025.mdx
- [ ] [T096] [US9] Create OpenVLA section in docs/08-vla/03-openvla.mdx
- [ ] [T097] [US9] Create Octo section in docs/08-vla/04-octo.mdx
- [ ] [T098] [US9] Create RDT-1B VLA section in docs/08-vla/05-rdt1b-vla.mdx
- [ ] [T099] [US9] Create prompt engineering in docs/08-vla/06-prompt-engineering.mdx
- [ ] [T100] [US9] Create action chunking in docs/08-vla/07-action-chunking.mdx
- [ ] [T101] [US9] Create ROS2 VLA bridge in docs/08-vla/08-ros2-vla-bridge.mdx
- [ ] [T102] [US9] Create full voice-to-action demo in docs/08-vla/09-full-voice-to-action-demo.mdx
- [ ] [T103] [US9] Create _category_.json for 08-vla navigation
- [ ] [T104] [US9] Create VLA pipeline in src/vla/

**Checkpoint**: At this point, Chapter 08 should be fully functional and testable independently

---

## Phase 12: Chapter 09 - Sim-to-Real Transfer & Domain Randomization (Priority: P10)

**Goal**: Complete sim-to-real transfer with domain randomization techniques

**Independent Test**: Readers can deploy sim-trained policies to real hardware

### Implementation for Chapter 09

- [ ] [T105] [P] [US10] Create introduction and learning objectives in docs/09-sim2real/01-intro.mdx
- [ ] [T106] [P] [US10] Create the gap is dead section in docs/09-sim2real/02-the-gap-is-dead.mdx
- [ ] [T107] [US10] Create domain randomization section in docs/09-sim2real/03-domain-randomization-2025.mdx
- [ ] [T108] [US10] Create system identification in docs/09-sim2real/04-system-identification.mdx
- [ ] [T109] [US10] Create Real2Sim pipeline in docs/09-sim2real/05-real2sim-pipeline.mdx
- [ ] [T110] [US10] Create actuator modeling in docs/09-sim2real/06-actuator-modeling.mdx
- [ ] [T111] [US10] Create latency compensation in docs/09-sim2real/07-latency-compensation.mdx
- [ ] [T112] [US10] Create 2025 success cases in docs/09-sim2real/08-2025-success-cases.mdx
- [ ] [T113] [US10] Create full sim2real deploy in docs/09-sim2real/09-full-sim2real-deploy.mdx
- [ ] [T114] [US10] Create _category_.json for 09-sim2real navigation
- [ ] [T115] [US10] Create sim2real tools in src/sim2real/

**Checkpoint**: At this point, Chapter 09 should be fully functional and testable independently

---

## Phase 13: Chapter 10 - Safety, Ethics & Human-Robot Interaction (Priority: P11)

**Goal**: Complete safety compliance with ISO standards and ethics

**Independent Test**: Readers can implement emergency stops and safety protocols

### Implementation for Chapter 10

- [ ] [T116] [P] [US11] Create introduction and learning objectives in docs/10-safety-ethics/01-intro.mdx
- [ ] [T117] [P] [US11] Create ISO TS standards in docs/10-safety-ethics/02-iso-ts-standards.mdx
- [ ] [T118] [US11] Create emergency stop section in docs/10-safety-ethics/03-emergency-stop.mdx
- [ ] [T119] [US11] Create speed separation section in docs/10-safety-ethics/04-speed-separation.mdx
- [ ] [T120] [US11] Create force limiting section in docs/10-safety-ethics/05-force-limiting.mdx
- [ ] [T121] [US11] Create privacy and data section in docs/10-safety-ethics/06-privacy-and-data.mdx
- [ ] [T122] [US11] Create bias and fairness section in docs/10-safety-ethics/07-bias-fairness.mdx
- [ ] [T123] [US11] Create regulatory landscape in docs/10-safety-ethics/08-regulatory-2025.mdx
- [ ] [T124] [US11] Create responsible deployment in docs/10-safety-ethics/09-responsible-deployment.mdx
- [ ] [T125] [US11] Create final checklist in docs/10-safety-ethics/10-checklist.mdx
- [ ] [T126] [US11] Create _category_.json for 10-safety-ethics navigation

**Checkpoint**: At this point, Chapter 10 should be fully functional and testable independently

---

## Phase 14: Chapter 11 - Capstone: Autonomous Conversational Humanoid (Priority: P12)

**Goal**: Complete end-to-end capstone with voice-to-action pipeline

**Independent Test**: Readers can run the full conversational humanoid demo

### Implementation for Chapter 11

- [ ] [T127] [P] [US12] Create capstone overview in docs/11-capstone/01-overview.mdx
- [ ] [T128] [P] [US12] Create scene and task definition in docs/11-capstone/02-scene-and-task.mdx
- [ ] [T129] [US12] Create full pipeline architecture in docs/11-capstone/03-full-pipeline.mdx
- [ ] [T130] [US12] Create voice-to-action section in docs/11-capstone/04-voice-to-action.mdx
- [ ] [T131] [US12] Create planning and control in docs/11-capstone/05-planning-and-control.mdx
- [ ] [T132] [US12] Create manipulation sequence in docs/11-capstone/06-manipulation-sequence.mdx
- [ ] [T133] [US12] Create safety wrapper in docs/11-capstone/07-safety-wrapper.mdx
- [ ] [T134] [US12] Create one-click demo in docs/11-capstone/08-one-click-demo.mdx
- [ ] [T135] [US12] Create extensions section in docs/11-capstone/09-extensions.mdx
- [ ] [T136] [US12] Create _category_.json for 11-capstone navigation
- [ ] [T137] [US12] Create full capstone implementation in capstone/

**Checkpoint**: At this point, Chapter 11 should be fully functional and testable independently

---

## Phase 15: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple chapters

- [ ] [T138] [P] Documentation updates in docs/
- [ ] [T139] Code cleanup and refactoring
- [ ] [T140] Performance optimization across all chapters
- [ ] [T141] [P] Additional unit tests in test/ directory
- [ ] [T142] Security hardening
- [ ] [T143] Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all chapters
- **Chapters (Phase 3+)**: All depend on Foundational phase completion
  - Chapters can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → ...)

### User Story Dependencies

- **Chapter 00 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other chapters
- **Chapter 01 (P2)**: Can start after Foundational (Phase 2) 
- **Chapter 02 (P3)**: Can start after Foundational (Phase 2)
- **Chapter 03 (P4)**: Depends on Chapter 02 (ROS 2 basics)
- **Chapter 04 (P5)**: Depends on Chapter 03 (modeling basics)
- **Chapter 05 (P6)**: Depends on Chapter 04 (simulation basics)
- **Chapter 06 (P7)**: Depends on Chapter 05 (perception integration)
- **Chapter 07 (P8)**: Depends on Chapter 06 (locomotion integration)
- **Chapter 08 (P9)**: Depends on Chapter 07 (manipulation integration)
- **Chapter 09 (P10)**: Depends on Chapter 08 (VLA integration)
- **Chapter 10 (P11)**: Can start after Foundational (Phase 2)
- **Chapter 11 (P12)**: Depends on all previous chapters (capstone integration)

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core functionality before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, chapters 01, 02, 04, 05, 06, 07, 08, 09, 10 can start in parallel (if team capacity allows)
- Different chapters can be worked on in parallel by different team members
- Each chapter can be completed independently and tested separately

---

## Parallel Example: Chapter 00

```bash
# Launch all tests for Chapter 00 together (if tests requested):
Task: "Unit tests for setup scripts in 00-setup/"

# Launch all content for Chapter 00 together:
Task: "Create hardware requirements documentation in docs/00-setup/01-hardware.mdx"
Task: "Create Docker + devcontainer setup guide in docs/00-setup/02-docker-devcontainer.mdx"
```

---

## Implementation Strategy

### MVP First (Chapter 00 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all chapters)
3. Complete Phase 3: Chapter 00
4. **STOP and VALIDATE**: Test Chapter 00 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Chapter 00 → Test independently → Deploy/Demo (MVP!)
3. Add Chapter 01 → Test independently → Deploy/Demo
4. Add Chapter 02 → Test independently → Deploy/Demo
5. ...
6. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: Chapter 00
   - Developer B: Chapter 01 
   - Developer C: Chapter 02
   - Developer D: Chapters 04, 05, 06
   - Developer E: Chapters 07, 08, 09
3. Chapters complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific chapter for traceability
- Each chapter should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- Avoid: vague tasks, same file conflicts, cross-chapter dependencies that break independence