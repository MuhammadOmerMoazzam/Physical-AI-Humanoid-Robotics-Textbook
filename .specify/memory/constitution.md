<!--
SYNC IMPACT REPORT
Version change: N/A (new constitution) → 1.0.0
Added sections: Core Principles (6 principles), Key Standards, Constraints, Success Criteria
Removed sections: None
Modified principles: N/A (new principles)
Templates requiring updates:
- ✅ spec-template.md: Updated functional requirements to align with constitution
- ⚠ N/A: plan-template.md, tasks-template.md (no changes needed - they reference constitution dynamically)
Deferred items: RATIFICATION_DATE (TODO item intentionally left for later completion)
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Accuracy through Primary Source Verification
Every factual claim must be traceable to reputable sources, with cross-referencing between multiple established robotics and AI literature to ensure veracity and reliability of the textbook content.

### II. Academic and Technical Clarity
Content must be written for an academic and technical audience, including students, researchers, and practitioners in AI, robotics, mechanical engineering, and related fields, with appropriate technical depth and accessible explanations.

### III. Comprehensive Coverage
The textbook must provide comprehensive coverage of foundational concepts, state-of-the-art methods, and practical applications in physical AI and humanoid robotics to serve as a complete educational resource.

### IV. Educational Focus
All content must prioritize educational value with real-world examples, exercises, code snippets, and interactive elements where possible to enhance the learning experience and practical understanding.

### V. Up-to-Date Content
The textbook must integrate emerging trends in physical AI and humanoid robotics as of December 2025 and beyond, ensuring that readers receive current and relevant information on the rapidly evolving field.

### VI. Verifiable Technical Claims
All technical claims must be 100% verifiable against cited sources with zero tolerance for plagiarism, and all code examples must execute correctly and be licensed openly (MIT or Apache 2.0).

## Key Standards

- All factual claims must be traceable to reputable sources
- Citation format: IEEE style (numeric references with full bibliography)
- Source types: Minimum 50% peer-reviewed journal articles and conference papers (e.g., ICRA, IROS, RSS, Science Robotics, IJRR); remainder may include authoritative textbooks, technical reports, and credible industry whitepapers
- Plagiarism check: 0% tolerance before final deployment (checked with tools such as Turnitin, GPTZero, or Copyleaks)
- Writing clarity: Flesch-Kincaid grade level 12–14 (suitable for upper-undergraduate to graduate technical readers)
- Tool usage: Spec-Kit Plus for spec-driven development and repository structure, and Claude Code for generating verifiable code examples, diagrams (Mermaid/PlantUML), and simulation snippets

## Constraints

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

## Success Criteria

- 100% of technical claims verified against cited sources
- Zero plagiarism detected on final content
- Successful deployment to GitHub Pages with full functionality (search, dark mode, responsive design, fast loading)
- All code examples execute correctly and are licensed openly (MIT or Apache 2.0)
- Positive internal review for educational value, depth, and completeness
- Full alignment with Spec-Kit Plus workflow: every piece of content traceable to an approved specification

## Governance

This constitution governs all development of the Physical AI & Humanoid Robotics Textbook project. All contributions must comply with these principles and standards. Amendments require documentation of changes, approval by project maintainers, and a migration plan for existing content when necessary. All pull requests and reviews must verify compliance with the principles outlined in this constitution. All content must be justified with clear educational or technical rationale, and the project uses this constitution as its primary guidance document for development decisions.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date to be determined | **Last Amended**: 2025-12-09
