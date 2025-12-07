---
id: 0002
title: update-constitution-with-physical-ai-humanoid-robotics-principles
stage: constitution
date: 2025-12-07
surface: agent
model: Claude
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, physical-ai, humanoid-robotics, textbook]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

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
  – Spec-Kit Plus for spec-driven development and repository structure  
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
- Interactive elements: CodeSandbox/Colab links, embedded Mermaid diagrams, and links to open-source simulators when relevant
- Repository must follow Spec-Kit Plus template structure (specifications as first-class artifacts)

### Success Criteria
- 100% of technical claims verified against cited sources
- Zero plagiarism detected on final content
- Successful deployment to GitHub Pages with full functionality (search, dark mode, responsive design, fast loading)
- All code examples execute correctly and are licensed openly (MIT or Apache 2.0)
- Positive internal review for educational value, depth, and completeness
- Full alignment with Spec-Kit Plus workflow: every piece of content traceable to an approved specification

## Response snapshot

Updated the project constitution with specific principles and standards for the Physical AI & Humanoid Robotics Textbook project, replacing all template placeholders with concrete content focused on academic accuracy, educational value, and technical excellence.

## Outcome

- ✅ Impact: Updated constitution with 6 core principles and detailed standards for the textbook project
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: /sp.specify to create first feature specification
- 🧠 Reflection: The constitution now provides clear governance for the textbook project with emphasis on accuracy, academic rigor, and educational quality.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): none needed
