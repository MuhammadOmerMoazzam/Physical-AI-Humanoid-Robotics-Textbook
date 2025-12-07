# Physical AI & Humanoid Robotics Textbook – Technical Implementation Plan

(Aligned with Spec-Kit Plus + Docusaurus production workflow)

## 1. Architecture Sketch

```
└── repo root
    ├── docs/                          → All Docusaurus Markdown/MDX content
    │   ├── 00-setup/
    │   ├── 01-foundations/
    │   └── …
    │   └── 11-capstone/
    ├── src/                           → Executable ROS 2 packages, Isaac Lab envs, policies
    ├── assets/                        → URDFs, USDs, calibration files, images
    ├── static/                        → Videos, large PDFs, model weights (git-lfs)
    ├── capstone/                      → Full end-to-end demo scene + launch files
    ├── docusaurus.config.js           → Docusaurus v3.5+ (Classic preset)
    ├── sidebars.js                    → Exact order defined in /sp.layout
    ├── src/
    │   ├── css/custom.css
    │   └── components/LiteYoutube.jsx
    ├── .devcontainer/                 → 100 % reproducible dev environment
    └── package.json                   → npm scripts: start | build | pdf | serve
```

Site URL (final): https://muhammadomermoazzam.github.io/Physical-AI-Humanoid-Robotics-Textbook/

## 2. Section Structure (final, locked)

- 00 – Setup & Development Environment  
- 01 – Foundations of Physical AI & Embodied Intelligence  
- 02 – ROS 2: The Robotic Nervous System  
- 03 – Modeling Humanoids: URDF, SRDF & MoveIt 2  
- 04 – Physics Simulation: Gazebo & NVIDIA Isaac Sim  
- 05 – Perception Stack for Humanoids  
- 06 – Bipedal Locomotion & Whole-Body Control  
- 07 – Dexterous Manipulation & Grasp Synthesis  
- 08 – Vision-Language-Action Models (VLA)  
- 09 – Sim-to-Real Transfer & Domain Randomization  
- 10 – Safety, Ethics & Human-Robot Interaction  
- 11 – Capstone: Autonomous Conversational Humanoid  
+ Appendices (Hardware Guide, Bibliography, Glossary)

## 3. Research Approach – Research-Concurrent (2025 Standard)

- Research is NOT done all upfront  
- Each chapter is researched while being written  
- Sources added on-the-fly → master Zotero library → auto-export to BibTeX  
- Citation style: IEEE numeric (overridden from original APA to match robotics community norm — documented as allowed)

## 4. Quality Validation Strategy (mapped to /sp.specify acceptance criteria)

| Acceptance Criterion                         | Validation Method                                                                 |
|----------------------------------------------|------------------------------------------------------------------------------------|
| All claims cited                             | CI runs citation checker + broken ref detection                                    |
| Code executes                                | GitHub Actions runs all Python/launch files in devcontainer (colcon test)              |
| Site builds & deploys                        | GitHub Actions → npm run build → pages-build-deployment                            |
| Capstone runs end-to-end                     | CI launches Isaac Sim headless + ROS 2 bridge + 60-second demo → success exit code|
| Responsive + dark mode + search              | Lighthouse CI + manual mobile checklist                                           |
| PDF export works                             | Weekly GitHub Action generates textbook.pdf and attaches to Releases              |

## 5. Important Decisions – Options & Trade-offs (locked choices)

| Decision                        | Chosen Option                  | Alternatives Considered               | Trade-offs                                                                                     |
|---------------------------------|--------------------------------|---------------------------------------|------------------------------------------------------------------------------------------------|
| Static site generator           | Docusaurus 3.5+ (React/MDX)    | Next.js, Hugo, MkDocs Material        | Docusaurus wins on MDX + React components + PDF plugin + ecosystem                             |
| Citation style                  | IEEE numeric                   | APA, BibLaTeX                         | IEEE is de facto standard in robotics (ICRA, IROS, RSS, RA-L)                                   |
| PDF generation                  | @docusaurus/plugin-pdf         | LaTeX pandoc, Typst                   | Only Docusaurus plugin preserves Mermaid, code blocks, dark mode                               |
| Video embedding                 | Custom LiteYouTube component   | Native iframe, YouTube-nocookie       | LiteYouTube = zero layout shift + GDPR friendly                                                |
| Code execution testing          | Devcontainer + colcon test     | Manual, Docker                        | Guarantees 100 % reproducibility across OSes                                                   |
| Model licensing                 | 8-bit quantised open weights   | Full-precision, closed                | 8-bit fits RTX 4070 Ti, preserves >99 % performance (2025 standard)                            |
| Large file hosting              | Git LFS + GitHub Releases      | HuggingFace datasets                  | Git LFS simpler for students, Releases gives stable URLs                                       |

## 6. Execution Phases (Research-Concurrent Model)

| Phase         | Duration | Key Activities                                                                 | Status    |
|---------------|----------|---------------------------------------------------------------------------------|-----------|
| Research      | Ongoing  | Continuous paper reading while writing each chapter                             | Complete  |
| Foundation    | Done     | Chapters 00–04 (setup, theory, ROS 2, modeling, simulation)                     | Complete  |
| Analysis      | Done     | Chapters 05–09 (perception, locomotion, manipulation, VLA, sim-to-real)         | Complete  |
| Synthesis     | Done     | Chapters 10–11 (safety/ethics + capstone integration)                           | Complete  |
| Polish & Ship | 5 days   | CI fixes, PDF, video, launch assets                                             | In progress |

## 7. Testing Strategy (Automated + Manual)

| Test Type                | Tool / Method                               | Frequency   |
|--------------------------|---------------------------------------------|-------------|
| Build & lint             | npm run build + eslint                      | Every push  |
| Link checking            | lychee action                               | Weekly      |
| Code execution           | colcon test in devcontainer                | Every push  |
| Capstone smoke test      | Isaac Sim headless + ROS 2 bridge (60 s)    | Every push  |
| PDF generation           | docusaurus-pdf action                       | Weekly      |
| Mobile responsiveness    | Lighthouse CI                               | Weekly      |
| Accessibility            | axe-core                                    | Weekly      |