# Data Model: Physical AI & Humanoid Robotics Textbook - Module 1

**Feature**: Module 1: Foundations & Infrastructure (Chapters 00-03)
**Created**: 2025-12-09
**Status**: Complete

## Entity Models

### Chapter
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, format: `ch##` | Unique identifier |
| title | string | Required, max 100 chars | Chapter title |
| position | integer | Required, 0-11 | Position in curriculum |
| module | string | Required, enum: `foundations`, `simulation`, `locomotion`, `intelligence` | Parent module |
| content | string | Required, MDX format | Main content |
| learning_objectives | string[] | Required, min 3 items | List of learning objectives |
| code_examples | CodeExample[] | Optional | Associated code examples |
| diagrams | Diagram[] | Optional | Associated diagrams |
| exercises | Exercise[] | Optional | End-of-chapter exercises |

### CodeExample
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| language | string | Required, enum: `python`, `bash`, `cpp`, `json`, `yaml` | Programming language |
| code | string | Required | Source code content |
| executable | boolean | Default: false | Whether code can be executed |
| description | string | Optional | Purpose of the example |

### Diagram
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| type | string | Required, enum: `mermaid`, `plantuml`, `image` | Diagram type |
| content | string | Required | Diagram code or path |
| alt_text | string | Required | Accessibility description |

### Exercise
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| type | string | Required, enum: `practice`, `discussion`, `project` | Exercise type |
| content | string | Required | Exercise description |
| difficulty | string | Required, enum: `light`, `medium`, `hard` | Difficulty level |
| solutions | Solution[] | Optional | Possible solutions |

### Solution
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| content | string | Required | Solution content |
| verified | boolean | Default: false | Whether solution is verified |

### RobotModel
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| name | string | Required | Robot name (e.g., "Unitree G1", "Figure 02") |
| degrees_of_freedom | integer | Required, positive | Number of degrees of freedom |
| urdf_path | string | Required | Path to URDF file |
| usd_path | string | Optional | Path to USD representation |
| mass | float | Required, positive | Mass in kg |
| link_count | integer | Required, positive | Number of links |
| joint_count | integer | Required, positive | Number of joints |

## State Models

### ChapterState
- `draft` → `review` → `approved` → `published`
- Initial: `draft`

### RobotModelState
- `design` → `urdf_created` → `collision_checked` → `usd_exported` → `sim_verified` → `real_verified`
- Initial: `design`

## Relationship Models

```
[Chapter] 1->* [CodeExample] "contains"
[Chapter] 1->* [Diagram] "contains"  
[Chapter] 1->* [Exercise] "has"
[Exercise] 1->* [Solution] "may_have"
[Chapter] 1->* [RobotModel] "may_reference"
[RobotModel] 1->* [RobotParameter] "has"
```