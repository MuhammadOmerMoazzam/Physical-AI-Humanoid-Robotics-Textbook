# Data Model: Physical AI & Humanoid Robotics Textbook - Module 2

**Feature**: Module 2: Simulation & Perception (Chapters 04-05)
**Created**: 2025-12-09
**Status**: Complete

## Entity Models

### SimulationScene
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, format: `scene_##` | Unique identifier |
| name | string | Required, max 100 chars | Scene name |
| description | string | Required | Detailed scene description |
| usd_path | string | Required, valid file path | Path to USD file |
| complexity_level | string | Required, enum: `low`, `medium`, `high` | Computational complexity rating |
| physics_accuracy_rating | number | Required, 1-10 scale | Rating of physics fidelity |
| humanoid_compatibile | boolean | Default: true | Whether scene supports humanoid robots |

### PerceptionPipeline
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| name | string | Required | Pipeline name |
| input_types | string[] | Required, non-empty | Supported input modalities |
| output_types | string[] | Required, non-empty | Output data types |
| hardware_requirements | string | Required | Minimum hardware specs |
| frame_rate | number | Required, positive | Processing frame rate |
| accuracy_metrics | AccuracyMetric[] | Required | Performance metrics |

### AccuracyMetric
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| metric_type | string | Required, enum: `accuracy`, `recall`, `precision`, `mIoU` | Type of metric |
| value | float | Required, 0.0-1.0 range | Metric value |
| test_dataset | string | Required | Dataset used for evaluation |

### SyntheticAsset
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| id | string | Required, unique | Unique identifier |
| name | string | Required | Asset name |
| type | string | Required, enum: `mesh`, `texture`, `animation`, `simulation`, `material` | Asset type |
| source_format | string | Required | Original format (USD, OBJ, etc.) |
| license | string | Required | License information |
| source | string | Required | Origin of the asset |
| creation_date | string | Required, ISO format | When asset was created |
| physics_properties | PhysicsProperties | Optional | Physical properties if applicable |

### PhysicsProperties
| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| mass | float | Required, positive | Mass in kg |
| friction_coefficient | float | Required, 0.0-2.0 range | Friction coefficient |
| restitution | float | Required, 0.0-1.0 range | Bounciness coefficient |
| density | float | Required, positive | Density in kg/m³ |

## State Models

### SimulationSceneState
- `design` → `created_usd` → `physics_validated` → `benchmarking` → `published`
- Initial: `design`

### PerceptionPipelineState
- `planned` → `implemented` → `tested_synthetic` → `tested_real` → `validated` → `deployed`
- Initial: `planned`

## Relationship Models

```
[SimulationScene] 1->* [SyntheticAsset] "contains"
[PerceptionPipeline] 1->1 [AccuracyMetric] "has_primary_performance_metric"
[PerceptionPipeline] 1->* [AccuracyMetric] "has_additional_metrics"
[SyntheticAsset] 1->1 [PhysicsProperties] "has_optional_physics"
```