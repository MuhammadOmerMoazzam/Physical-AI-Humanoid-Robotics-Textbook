# Data Model - Physical AI & Humanoid Robotics Textbook

## Core Entities

### Chapter
- chapter_id (string): Unique identifier (00, 01, 02, ..., 11)
- title (string): Chapter title
- description (string): Short description of the chapter
- position (integer): Chapter number in sequence
- learning_objectives ([]string): Learning objectives for the chapter
- target_word_count (integer): Target word count for the chapter
- status (enum): draft, review, published

### ContentFile
- file_path (string): Path to the MDX file
- chapter_id (string): Associated chapter
- file_type (enum): mdx, yaml, json, usd, urdf, srdf, cpp, py, launch
- content_type (string): intro, theory, practice, example, exercise, summary
- last_modified (datetime): Last time content was modified

### RobotModel
- model_name (string): Name of the humanoid robot (e.g., "Unitree G1", "Figure 02", "Tesla Optimus")
- degrees_of_freedom (integer): Number of actuated joints
- height (float): Height in meters
- weight (float): Weight in kg
- actuators ([]Actuator): List of actuators
- kinematic_chain (KinematicChain): Kinematic tree representation

### Actuator
- joint_name (string): Name of the joint
- joint_type (enum): revolute, prismatic, spherical, fixed, floating
- limits (JointLimits): Position, velocity, and effort limits
- dynamics (DynamicsParams): Inertia, damping, friction parameters

### JointLimits
- position_min (float): Minimum position
- position_max (float): Maximum position
- velocity_max (float): Maximum velocity
- effort_max (float): Maximum effort

### DynamicsParams
- mass (float): Mass of the link
- center_of_mass (Vector3): Center of mass in local coordinates
- inertia (Matrix3x3): Inertia matrix

### KinematicChain
- root_link (string): Base link name
- joints ([]Joint): Joints connecting the links
- links ([]Link): Links in the robot

### Joint
- name (string): Joint name
- type (string): Joint type
- parent_link (string): Parent link name
- child_link (string): Child link name
- origin (Transform): Transform from parent to joint
- axis (Vector3): Joint axis

### Link
- name (string): Link name
- visual_mesh (Mesh): Visual representation
- collision_mesh (Mesh): Collision representation
- inertial (Inertial): Inertial properties

### Mesh
- filename (string): Path to mesh file
- scale (Vector3): Scaling factor
- material (Material): Visual material

### Material
- name (string): Material name
- color (ColorRGBA): RGBA color values
- texture (string): Texture file path

### PolicyModel
- name (string): Model name (e.g., "RDT-1B", "OpenVLA", "Octo")
- parameters (integer): Number of parameters in billions
- training_data_size (integer): Size of training data in trajectories
- zero_shot_performance (float): Zero-shot performance on standard benchmarks
- real_robot_success_rate (float): Success rate on real robots
- open_weight (boolean): Whether weights are publicly available

### SafetyRequirement
- iso_standard (string): ISO standard (e.g., "ISO/TS 15066", "ISO 10218")
- requirement_type (enum): force_limiting, speed_separation, emergency_stop, etc.
- threshold_value (float): Threshold for the requirement
- measurement_unit (string): Unit of measurement (N, m/s, etc.)
- compliance_status (enum): not_started, in_progress, compliant

### Task
- task_name (string): Name of the task (e.g., "table_tidy", "water_bottle_grasp")
- task_description (string): Detailed description of the task
- success_criteria ([string]): Criteria for task success
- safety_constraints ([]SafetyConstraint): Safety constraints for the task
- estimated_completion_time (float): Estimated time in seconds

### SafetyConstraint
- constraint_type (enum): speed_limit, force_limit, distance_limit, etc.
- limit_value (float): Value for the constraint
- affected_joints ([string]): Joints affected by the constraint
- enforcement_method (string): How the constraint is enforced

### Citation
- id (string): Unique identifier for the citation (e.g., "author2023title")
- title (string): Title of the citation
- authors ([string]): List of authors
- venue (string): Venue (e.g., ICRA, IROS, Science Robotics)
- year (integer): Publication year
- doi (string): DOI if available
- url (string): URL if available
- citation_style (enum): ieee, acm, apa, etc.