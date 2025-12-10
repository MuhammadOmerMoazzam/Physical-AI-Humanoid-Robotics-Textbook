# Data Model: Physical AI & Humanoid Robotics Textbook - Module 3

**Feature**: Module 3: Locomotion & Dexterous Manipulation (Chapters 06-07)
**Created**: 2025-12-09
**Status**: Complete

## Entity Models

### WalkingController
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, format: `ctrl_###` | Unique identifier |
| name | string | Required, max 50 chars | Controller name |
| algorithm_type | string | Required, enum: `classic_mpc`, `rl_raise`, `dreamerv3`, `hybrid` | Algorithm type |
| max_speed | float | Required, positive | Maximum achievable walking speed (m/s) |
| push_recovery_capability | float | Required, positive | Maximum recoverable push force (N) |
| terrain_tolerance | string | Required | Description of acceptable terrain variations |
| computational_requirements | string | Required | Hardware requirements for real-time operation |
| success_rate | float | Required, 0.0-1.0 range | Task success rate in real-world tests |

### ManipulationTask
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, format: `task_###` | Unique identifier |
| name | string | Required, max 100 chars | Task name |
| complexity_level | string | Required, enum: `low`, `medium`, `high`, `advanced` | Complexity rating |
| required_dofs | integer | Required, positive | Minimum degrees of freedom needed |
| success_rate | float | Required, 0.0-1.0 range | Success rate achieved |
| execution_time | float | Required, positive | Average execution time in seconds |
| object_categories | string[] | Required, non-empty | Types of objects this task handles |
| pre_conditions | string[] | Required | Conditions that must hold before task execution |
| post_conditions | string[] | Required | Conditions that hold after successful task completion |

### GraspStrategy
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| name | string | Required | Name of the grasp strategy |
| approach_direction | Vector3 | Required | Preferred approach direction for grasping |
| grasp_type | string | Required, enum: `parallel`, `tripod`, `pinch`, `power` | Type of grasp |
| success_rate | float | Required, 0.0-1.0 range | Success rate on selected objects |
| object_categories | string[] | Required, non-empty | Object categories this strategy works for |
| required_sensors | string[] | Required | Sensors needed for successful execution |
| computational_cost | string | Required, enum: `low`, `medium`, `high` | Computational requirements |

### HumanoidConfiguration
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, format: `robot_###` | Unique identifier |
| name | string | Required | Robot name (e.g., "Unitree G1", "Figure 02") |
| height | float | Required, positive | Robot height in meters |
| weight | float | Required, positive | Robot weight in kg |
| degrees_of_freedom | integer | Required, positive | Total DoF |
| leg_dofs | integer | Required, positive | DoF in legs only |
| arm_dofs | integer | Required, positive | DoF in arms only |
| max_payload | float | Required, positive | Maximum payload in kg |

## State Models

### WalkingControllerState
- `development` → `testing_simulation` → `validation_real_robot` → `production_ready`
- Initial: `development`

### ManipulationTaskState
- `definition` → `implementation` → `simulation_validation` → `real_robot_testing` → `complete`
- Initial: `definition`

## Relationship Models

```
[HumanoidConfiguration] 1->* [WalkingController] "supports"
[HumanoidConfiguration] 1->* [ManipulationTask] "can_execute"
[GraspStrategy] 1->* [ManipulationTask] "enables"
[ManipulationTask] 1->* [GraspStrategy] "requires"
[WalkingController] 1->* [LocomotionExercise] "implements"
```