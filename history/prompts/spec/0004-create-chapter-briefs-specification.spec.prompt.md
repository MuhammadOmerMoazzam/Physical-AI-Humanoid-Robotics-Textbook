---
id: 0004
title: create-chapter-briefs-specification
stage: spec
date: 2025-12-07
surface: agent
model: Claude
feature: 4-chapter-briefs
branch: update-constitution-physical-ai-textbook
user: unknown
command: /sp.chapter-briefs
labels: [specification, briefs, chapters, textbook, curriculum]
links:
  spec: specs/4-chapter-briefs/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/4-chapter-briefs/spec.md
 - specs/4-chapter-briefs/checklists/requirements.md
 - specs/4-chapter-briefs/chapter-00-brief.md
 - specs/4-chapter-briefs/chapter-01-brief.md
 - specs/4-chapter-briefs/chapter-02-brief.md
 - specs/4-chapter-briefs/chapter-03-brief.md
 - specs/4-chapter-briefs/chapter-04-brief.md
 - specs/4-chapter-briefs/chapter-05-brief.md
 - specs/4-chapter-briefs/chapter-06-brief.md
 - specs/4-chapter-briefs/chapter-07-brief.md
 - specs/4-chapter-briefs/chapter-08-brief.md
 - specs/4-chapter-briefs/chapter-09-brief.md
 - specs/4-chapter-briefs/chapter-10-brief.md
 - specs/4-chapter-briefs/chapter-11-brief.md
tests:
 - none
---

## Prompt

/sp.chapter-briefs for all 12 chapters with requirements as specified:

/sp.chapter-00-brief Setup & Development Environment
- Exact hardware requirements table (2025 edition: RTX 4070 Ti → 5090, 64 GB RAM, Ubuntu 22.04)
- Step-by-step Docker + devcontainer setup with provided Dockerfile and devcontainer.json
- One-click install script for ROS 2 Iron, NVIDIA drivers, CUDA 12.4+, Isaac Sim 2024.2+
- Repo clone + build instructions that work on day one
- VS Code recommended extensions list + settings.json
- Troubleshooting FAQ for the top 10 student failure modes seen in 2024–2025 cohorts
Success: Reader can launch a working Isaac Sim + ROS 2 bridge in <15 minutes

/sp.chapter-01-brief Foundations of Physical AI & Embodied Intelligence
- Moravec’s paradox revisited in the era of foundation models
- Why humanoid form factor wins in human environments (2025 data from Figure, Tesla, Agility, Boston Dynamics)
- The three data engines powering modern robotics (internet video, teleop, simulation)
- Economic inflection point: cost curves for actuators, compute, and AI training 2023–2025
- Definition and taxonomy of Physical AI vs Embodied AI vs Humanoid Robotics
- 15+ IEEE/ICRA citations + key arXiv papers that became canonical in 2024–2025
Success: Reader can explain in 3 minutes why 2025 is the “iPhone moment” for humanoid robots

[... and so on for all 12 chapters as specified ...]

## Response snapshot

Created comprehensive chapter briefs specification with 3 prioritized user stories, 13 functional requirements, and 10 success criteria. Also created individual brief files for all 12 chapters (00-11) with key content points, references, and success criteria.

## Outcome

- ✅ Impact: Created complete chapter briefs for all 12 chapters of the Physical AI textbook with key points and references
- 🧪 Tests: none
- 📁 Files: specs/4-chapter-briefs/spec.md, requirements checklist, and 12 chapter brief files
- 🔁 Next prompts: Begin detailed per-chapter specs in /sp.chapter-01, /sp.chapter-02, etc.
- 🧠 Reflection: The specification captures all essential brief content with clear structure and success criteria.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): /sp.chapter-briefs
- Next experiment (smallest change to try): none needed
