# AI usage record

Last updated: 2026-11-18.

This file records material use of generative AI in SphereCalib research and software. It is intentionally broader than a future paper disclosure. Human authors remain responsible for verifying every equation, reference, implementation, experiment, figure, and conclusion.

## Tools used

- **OpenAI ChatGPT:** research discussion, adversarial review prompts, roadmap development, and critique of possible publication claims. The exact model/version should be added from the relevant account export if available.
- **OpenAI Codex coding agent:** repository audit; Python implementation; tests; dataset adapters; experiment orchestration; result summaries; documentation; mathematical checking; and drafting of research-planning text. The exact deployed model/version should be recorded when exposed by the client.

AI systems are tools, not authors. They did not independently validate the scientific results and must not be credited as authors.

## Repository areas materially generated or modified with AI assistance

The first Python reimplementation and its documentation were substantially produced with Codex assistance, under human direction and review. This includes:

- `src/spherecalib/geometry.py`: calibrated-ray geometry, Direct3pFit-style equal-cone estimator, experimental 1.5p estimator, residuals, and refinement;
- `src/spherecalib/conics.py`: ellipse/conic representations and conic-to-sphere conversion;
- `src/spherecalib/robust.py`: RANSAC and refinement orchestration;
- `src/spherecalib/edges.py`: classical edge extraction, subpixel localization, tangents, and confidence proxy;
- `src/spherecalib/lidar.py`: sphere fitting, robust samplers, and plane removal;
- `src/spherecalib/registration.py`: rigid registration and error metrics;
- `src/spherecalib/pipeline.py`, `data.py`, `visualization.py`, and `synthetic.py`;
- `src/spherecalib/datasets/`: CalibRGBD and SHREC 2022 adapters;
- `src/spherecalib/cli.py` and `evaluation.py`;
- `tests/test_spherecalib.py`;
- PowerShell scripts under `scripts/`;
- research and implementation documents under `docs/`, `README.md`, and `REVIEW.md`.

This list describes provenance, not correctness. Each component requires normal human code review and independent scientific validation.

## Mathematical assistance

AI assistance has been used to:

- restate the Direct3pFit/equal-cone equations from source and papers;
- derive and inspect the experimental one-oriented-plus-one-point (1.5p) formula;
- enumerate the squared tangency roots and forward-visibility checks;
- outline a two-oriented-point (2OP) formulation as intersection of center-candidate lines;
- identify degeneracies, conditioning factors, sign ambiguities, and possible covariance double counting;
- propose independent numerical, symbolic, and finite-difference checks.

No novelty, uniqueness, optimality, or uncertainty claim based on these derivations is considered verified until checked independently by a human author and, where appropriate, by symbolic/numerical fixtures.

## Experimental assistance

AI assistance has been used to create or modify:

- paired synthetic benchmark generation;
- equal-proposal and assumed equal-confidence RANSAC comparisons;
- bootstrap summaries and paired method differences;
- CalibRGBD processed-measurement evaluation;
- SHREC 2022 sphere-fitting evaluation;
- commands, download scripts, checksums, and compact result records.

The human authors must verify dataset licenses, split integrity, coordinate conventions, failure accounting, statistical assumptions, and all reported values before publication.

## Manuscript assistance

No submission-ready VISAPP manuscript exists as of this update. ChatGPT and Codex have assisted with planning, critical-review text, and possible paper structure. If AI later drafts or edits manuscript prose, the affected sections, tool, purpose, and date must be appended here before submission.

## Proposed paper disclosure

The current SCITEPRESS ethics policy says that AI-generated text, images, figures, and code used in an article must be disclosed in the acknowledgments, naming the tool and identifying the affected parts. The VISAPP 2027 guidelines also state that sections using AI-generated text should cite the AI system. At the same time, the double-blind instructions require omission of acknowledgments and identifying details from the review manuscript.

Because those instructions are in tension, the authors should obtain written guidance from the VISAPP secretariat before submission. Do not silently omit the disclosure and do not insert an identifying acknowledgment into the anonymized manuscript without confirmation.

Candidate camera-ready wording, to be updated to match actual use:

> OpenAI ChatGPT and the OpenAI Codex coding agent were used for research planning, adversarial review, software drafting, test and experiment scaffolding, and language editing. AI-assisted repository areas and manuscript sections are recorded in the accompanying provenance log. The authors independently verified the mathematical derivations, source code, references, experiments, figures, and conclusions and take full responsibility for the content.

## Submission-integrity checklist

- Record exact AI product/model identifiers when available.
- Record every manuscript section, figure, or table materially drafted or edited with AI.
- Independently verify citations against primary sources.
- Independently rederive central equations and inspect generated code.
- Preserve trial-level outputs, configurations, environment versions, and failures.
- Do not upload a confidential manuscript or peer-review material to a public AI service.
- Do not publish the anonymized submission on a public preprint server during review; VISAPP explicitly forbids this.
- Do not link an identifying public repository from a double-blind manuscript unless the conference explicitly permits it; prepare an anonymous artifact if needed.

## Official policy sources checked

- VISAPP 2027 guidelines: https://visapp.scitevents.org/Guidelines.aspx?y=2027
- SCITEPRESS ethics and generative-AI policy: https://www.scitepress.org/EthicsOfPublication.aspx
