# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: Module 4: Intelligence, Transfer & Responsibility
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

### SafetyProtocol
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| standard | string | Required, enum: `ISO/TS 15066`, `ISO 10218`, `IEC 61508`, `IEC 60204-1` | Applicable standard |
| requirement | string | Required | Specific requirement text |
| implementation | string | Required | How requirement is implemented |
| verification_method | string | Required | How compliance is verified |

### VLACommand
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| input | string | Required | Natural language input command |
| processed_by | string | Required, enum: `whisper`, `vlm`, `vla` | Processing component |
| output | string | Required | Generated action sequence |
| confidence | number | Required, range 0.0-1.0 | Confidence score |

## State Models

### ChapterState
- `draft` → `review` → `approved` → `published`
- Initial: `draft`

### SafetyComplianceState
- `not_started` → `in_progress` → `verified` → `certified`
- Initial: `not_started`

## Relationship Models

```
[Chapter] 1->* [CodeExample] "contains"
[Chapter] 1->* [Diagram] "contains"  
[Chapter] 1->* [Exercise] "has"
[Exercise] 1->* [Solution] "may_have"
[Chapter] 1->* [SafetyProtocol] "implements"
```