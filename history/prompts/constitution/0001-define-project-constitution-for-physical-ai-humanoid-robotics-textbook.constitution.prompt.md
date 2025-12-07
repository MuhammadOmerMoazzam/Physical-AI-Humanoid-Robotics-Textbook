---
id: 0001
title: Define project constitution for Physical AI & Humanoid Robotics Textbook
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: master
user: user
command: "/sp.constitution"
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:

tests:

---

## Prompt

/sp.constitution

**Project:** Physical AI & Humanoid Robotics Textbook
(A book built with Docusaurus and deployed to GitHub Pages using Spec-Kit Plus and Claude Code)

### Core Principles
- Accuracy through primary source verification and cross-referencing with established robotics and AI literature
- Clarity for an academic and technical audience (students, researchers, and practitioners in AI, robotics, mechanical engineering, and related fields)
- Comprehensiveness in covering foundational concepts, state-of-the-art methods, and practical applications
- Educational focus with real-world examples, exercises, code snippets, and interactive elements where possible
- Up-to-date integration of emerging trends in physical AI and humanoid robotics (as of December 2025 and beyond)

### Key Standards
- All factual claims must be traceable to reputable sources
- Citation format: IEEE style (numeric references with full bibliography)
- Source types: Minimum 50% peer-reviewed journal articles and conference papers (e.g., ICRA, IROS, RSS, Science Robotics, IJRR); remainder may include authoritative textbooks, technical reports, and credible industry whitepapers
- Plagiarism check: 0% tolerance before final deployment (checked with tools such as Turnitin, GPTZero, or Copyleaks)
- Writing clarity: Flesch-Kincaid grade level 12–14 (suitable for upper-undergraduate to graduate technical readers)
- Tool usage:
  – Spec-Kit Plus[](https://github.com/panaversity/spec-kit-plus/) for spec-driven development and repository structure
  – Claude Code for generating verifiable code examples, diagrams (Mermaid/PlantUML), and simulation snippets

### Constraints
- Format: Static website built with Docusaurus, deployed via GitHub Pages
- Minimum 10 chapters (suggested outline available in separate spec)
- Each chapter must include:
  - Learning objectives
  - Key concepts with citations
  - Code examples (Python/ROS2 where applicable)
  - Diagrams and illustrations
  - End-of-chapter exercises or discussion questions
- Intera

## Response snapshot

Acknowledged the user-provided project constitution. The core principles, key standards, and constraints have been processed and recorded in a Prompt History Record.

## Outcome

- ✅ Impact: Defined project constitution with core principles, standards, and constraints.
- 🧪 Tests: N/A
- 📁 Files: No files created/modified directly.
- 🔁 Next prompts: Awaiting further instructions from the user to proceed with planning or implementation based on the constitution.
- 🧠 Reflection: The user provided a comprehensive constitution for the project. The next step will likely involve planning or creating specs.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
