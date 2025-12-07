---
id: 0002
title: create-book-layout-specification
stage: spec
date: 2025-12-07
surface: agent
model: Claude
feature: 2-book-layout
branch: update-constitution-physical-ai-textbook
user: unknown
command: /sp.layout
labels: [specification, layout, docusaurus, navigation, textbook]
links:
  spec: specs/2-book-layout/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/2-book-layout/spec.md
 - specs/2-book-layout/checklists/requirements.md
tests:
 - none
---

## Prompt

/sp.layout Physical AI & Humanoid Robotics Textbook – Site Structure & Navigation

Site base URL: https://<your-org>.github.io/physical-ai-textbook

Docusaurus version: 3.x (Classic preset + MDX v3)

Top-level sidebar (exact order):

├── 🏠 Home
├── 🚀 Introduction
├── 📚 Chapters
│   ├── 00 – Setup & Development Environment
│   ├── 01 – Foundations of Physical AI & Embodied Intelligence
│   ├── 02 – ROS 2: The Robotic Nervous System
│   ├── 03 – Modeling Humanoids: URDF, SRDF & MoveIt 2
│   ├── 04 – Physics Simulation: Gazebo & NVIDIA Isaac Sim
│   ├── 05 – Perception Stack for Humanoids
│   ├── 06 – Bipedal Locomotion & Whole-Body Control
│   ├── 07 – Dexterous Manipulation & Grasp Synthesis
│   ├── 08 – Vision-Language-Action Models (VLA)
│   ├── 09 – Sim-to-Real Transfer & Domain Randomization
│   ├── 10 – Safety, Ethics & Human-Robot Interaction
│   └── 11 – Capstone: Autonomous Conversational Humanoid
├── ⚙️ Appendices
│   ├── Hardware Guide (2025 edition)
│   ├── Docker & DevContainer Setup
│   ├── Bibliography (master)
│   └── Glossary
└── 🔗 External Resources & Community

Additional pages (no sidebar nesting):
- Contributors
- License (CC-BY-SA 4.0 for text, MIT/Apache 2.0 for code)
- Version / Changelog

Features that MUST be enabled:
- Dark mode (default: system preference)
- Full-text search (Algolia DocSearch or built-in)
- MDX + React components for interactive demos
- Mermaid diagrams v10+
- GitHub link button on every page (Edit this page)
- Automated PDF export (via @docusaurus/pdf-generator on GitHub Actions)

Static assets folders:
static/models/      → sample URDFs & USD assets
static/datasets/    → small synthetic datasets
static/videos/      → short demo clips

## Response snapshot

Created comprehensive specification for Physical AI & Humanoid Robotics Textbook site layout with 3 prioritized user stories, 14 functional requirements, and 10 measurable success criteria. The spec includes navigation structure, Docusaurus configuration requirements, and site features.

## Outcome

- ✅ Impact: Created complete layout specification for Physical AI textbook website with navigation structure and feature requirements
- 🧪 Tests: none
- 📁 Files: specs/2-book-layout/spec.md and requirements checklist
- 🔁 Next prompts: /sp.outline to create chapter outlines
- 🧠 Reflection: The specification captures all essential layout requirements with clear success metrics.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): /sp.layout instead of /sp.specify
- Next experiment (smallest change to try): none needed
