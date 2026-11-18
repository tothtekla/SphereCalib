# SphereCalib adversarial scientific review

Audit date: 2026-11-18. Repository tip audited: `eaeb594` on `master` before this report was added.

## Executive assessment

The repository is a credible research scaffold, not yet evidence for a new publication claim. It contains a complete classical Python calibration pipeline, an algebraically source-consistent Direct3pFit-style estimator, an experimental 1.5p hypothesis, heuristic orientation-aware refinement, reproducible synthetic comparisons, and public processed-measurement/3D benchmarks.

The central proposed claim is unresolved. Existing results do **not** show that 1.5p is more accurate than Direct3pFit under noisy tangents. With equal proposals and common point refinement the two were effectively tied in the selected synthetic condition; under the declared equal-confidence model, 3p was better. An initial 2OP geometric baseline now exists, but it has only exact tests and a non-conclusive smoke run. No experiment measures real image-derived tangent error against ground truth, and no covariance-aware refinement exists.

The strongest defensible research direction is therefore not “a superior smaller minimal solver.” It is a controlled investigation of whether and when oriented contour information improves robust sphere estimation, including negative regimes, calibrated reliability, and total-cost comparisons. A useful paper could remain possible if it produces a clear regime map and a principled orientation-aware estimator, even if 1.5p/2OP are not more accurate as isolated hypotheses.

**Current recommendation: NO-GO for submission.** Reconsider only after the evidence gates below are met. In particular, rushing the incomplete work into the remaining VISAPP 2027 late-breaking window would optimize for deadline rather than scientific correctness.

## Repository audit: implemented versus missing

| Area | Implemented now | Missing for the central claim |
| --- | --- | --- |
| Direct3pFit | Python equal-cone minimal and overdetermined estimator; exact tests; source equation matches legacy MATLAB | Runtime MATLAB/Python noisy fixtures, IJCV-scale reproduction, explicit numerical/failure-policy parity |
| 1.5p | Closed-form forward branch, sign handling, exact tests, RANSAC integration | Independent symbolic/Jacobian verification, conditioning map, complete degeneracy tests, real tangent evaluation |
| 2OP | Signed center-line least-squares baseline, RANSAC integration, exact randomized tests | Independent reference implementation, diagnostics, covariance model, preregistered benchmarks |
| Refinement | Point-only and heuristic stacked point+orientation residuals | Correlated covariance, calibrated weights, derivative tests, oracle/estimated tangent isolation |
| Synthetic evaluation | Paired seeds; point/tangent noise; outliers; equal-proposal and assumed equal-confidence budgets; bootstrap medians | Arc length/occlusion, projected size, position/range sweeps, blur/distortion/intrinsics/radius error, equal wall clock, total tangent-extraction time |
| CalibRGBD | 7,700 processed-measurement runs across all synthetic depth-noise groups | Raw-image tangent evaluation; a fair tangent source; caution remains because provided ellipses are already processed |
| 3D subsystem | SHREC 2022 sphere tests and multiple fitters | Not central to this paper; adjacency sampler failure needs fixing only if end-to-end calibration is claimed |
| Real pipeline | Bundled examples and end-to-end API | Independent surveyed GT, real tangent annotations, workflow success rate, evidence that image improvement affects extrinsics |
| Statistics | Failure counts, paired seeds, bootstrap intervals | Preregistered primary endpoints, scene-level resampling, failure-aware estimands, multiple-comparison policy |
| Publication | Citation/provenance docs | Novelty conclusion, validated figures/tables, anonymous artifact, manuscript, resolved AI disclosure placement |

## Top 10 threats to validity

### 1. Novelty is not established

- **Likely objection:** A center line from an oriented contour point and intersection of two such lines are direct tangent-plane geometry, not a publishable solver contribution.
- **Current evidence:** The 2OP derivation reduces to line intersection; the initial literature search found extensive sphere/conic/tangent-plane geometry but no verified identical 1.5p/2OP paper.
- **Missing evidence:** Systematic primary-source novelty review across sphere localization, oriented/affine features, conic fitting, and apparent-contour reconstruction.
- **Correction:** Build a claim chart: assumptions, measurements, equations, sample count, ambiguity, and output for each closest work.
- **If unresolved:** Fatal to a solver-novelty paper; not fatal to an empirical orientation-information study.

### 2. 2OP has only an initial geometric implementation

- **Likely objection:** A straightforward closest-lines implementation is being promoted before independent validation or evidence of benefit.
- **Current evidence:** Production code enumerates four sign pairs, fits the two candidate center lines, enforces forward visibility, and passes exact randomized tests. A 10-trial smoke run is negative and non-conclusive.
- **Missing evidence:** Independent reference implementation, conditioning diagnostics, covariance model, and preregistered noisy evaluation.
- **Correction:** Implement independently a second time, cross-check, then run the fixed experiment matrix without retuning to rescue 2OP.
- **If unresolved:** Fatal to any 2OP claim.

### 3. Point and tangent uncertainty is double counted

- **Likely objection:** The orientation-aware objective treats correlated measurements from the same pixels as independent observations and may manufacture confidence.
- **Current evidence:** Fixed `sigma_angle` and `sigma_orientation` scale stacked residuals; no covariance is propagated.
- **Missing evidence:** Joint covariance from the edge-localization procedure and correlation across neighboring samples.
- **Correction:** Estimate local measurement covariance or use a generative image/contour fit; compare covariance-aware and heuristic objectives.
- **If unresolved:** Weakens or invalidates claims of statistical efficiency.

### 4. Direct3pFit parity is incomplete

- **Likely objection:** A baseline labeled Direct3pFit may differ in normalization, least-squares policy, degeneracy handling, or refinement.
- **Current evidence:** Source inspection shows algebraic equivalence to legacy `fitSphere3p.m`; exact Python tests pass.
- **Missing evidence:** Paired noisy fixtures and outputs from MATLAB/final IJCV code across minimal/overdetermined and ill-conditioned cases.
- **Correction:** Export fixed fixtures/results from MATLAB or verified reference code and define numeric tolerances and failure equivalence.
- **If unresolved:** Undermines every comparative claim against Direct3pFit.

### 5. Existing synthetic coverage is too narrow

- **Likely objection:** One sphere pose, radius, threshold, outlier model, and full contour do not establish general behavior.
- **Current evidence:** Current sweep varies only point noise and independent tangent noise around one scene with 25% artificial outliers.
- **Missing evidence:** Range/position/size, arc visibility, sampling, blur, distortion, intrinsics, radius error, and near-degeneracy sweeps.
- **Correction:** Freeze a factorial experiment matrix before inspecting new results.
- **If unresolved:** High risk of cherry-picked conclusions.

### 6. RANSAC fairness is incomplete

- **Likely objection:** Fewer hypotheses are not a speed advantage when tangent estimation and lower hypothesis quality are ignored.
- **Current evidence:** Equal proposals and an assumed-confidence calculation exist.
- **Missing evidence:** Equal solver wall time and equal total processing time including tangent extraction; measured tangent reliability rather than assumed `q=0.95`.
- **Correction:** Benchmark proposal throughput separately and enforce matched wall-clock budgets; sweep measured `q`.
- **If unresolved:** Any efficiency claim is unsupported.

### 7. Tangent-noise model is unrealistic

- **Likely objection:** Independently rotating analytic tangents does not represent tangents derived from noisy, blurred pixels.
- **Current evidence:** Synthetic position and tangent perturbations are independent; edge confidence is normalized gradient magnitude.
- **Missing evidence:** Oracle versus image-estimated tangents, their bias/covariance, highlights, texture, shadow, and occlusion boundaries.
- **Correction:** Render images with controlled degradations and compare estimated tangents to analytic silhouette tangents.
- **If unresolved:** Practical benefit of orientation information is unknown.

### 8. Practical calibration benefit is unproven

- **Likely objection:** Even a better image center may not improve extrinsics because LiDAR centers, correspondence, and target configuration dominate.
- **Current evidence:** Processed CalibRGBD center differences lead to small calibration differences; real examples lack independent GT.
- **Missing evidence:** Error-source sensitivity and surveyed/reliable extrinsic GT.
- **Correction:** Inject isolated image/cloud center errors into controlled registration configurations and then validate on independent real data.
- **If unresolved:** Keep claims at image-based sphere estimation, not calibration improvement.

### 9. Conditioning and failures are not characterized

- **Likely objection:** Closed forms may appear fast only because unstable hypotheses are silently rejected or refined away.
- **Current evidence:** Basic degeneracy checks and final failure counts exist.
- **Missing evidence:** Jacobian singular values, sample separation, branch/sign diagnostics, rejection reasons, and failure-aware accuracy.
- **Correction:** Log condition diagnostics per hypothesis and publish failure maps.
- **If unresolved:** Numerical-stability claims are unsupported.

### 10. Publication timing and anonymity create integrity risk

- **Likely objection:** The work is being rushed after regular deadlines, and the public author-linked repository can defeat double blind review.
- **Current evidence:** VISAPP regular stages are closed; repository and prior papers identify the authors.
- **Missing evidence:** Conference confirmation on anonymous artifacts and AI disclosure placement.
- **Correction:** Do not link the identifying repository in review; request written guidance; prepare an anonymous artifact only if allowed.
- **If unresolved:** Administrative rejection or compromised review.

## Mathematical correctness and novelty assessment

The Direct3pFit-style Python equation is algebraically equivalent to the legacy MATLAB source after calibrated rays are normalized. Runtime parity is still untested. The 1.5p forward branch has a coherent derivation, but its denominator `1-u.v` exposes severe close-ray conditioning. The second squared branch fails the two-forward-contact test.

Two oriented observations define two candidate center lines. Exact compatible lines intersect; noisy lines are skew. Their closest-point midpoint is a simple estimator, not automatically an optimal one. This makes 2OP a useful baseline and consistency test, while its theoretical novelty remains doubtful. See `docs/ORIENTED_SOLVER_AUDIT.md` for the derivation and verification plan.

An initial 10-trial, 100-proposal smoke run at 0.5 px position noise and 3 degree tangent noise produced unrefined median center errors of approximately 6.83 mm (3p), 6.96 mm (1.5p), and 13.07 mm (2OP). Point-refined medians were approximately 1.86 mm, 2.27 mm, and 2.50 mm, respectively. The 2OP path was also about three times slower in this Python implementation because it enumerates sign pairs and solves a least-squares problem. These values are debugging evidence only: the sample is too small, the settings predate preregistration, and they must not be cited as a scientific comparison. They nevertheless argue against assuming a 2OP advantage.

No current evidence justifies the phrases “unique under all conditions,” “numerically stable,” “statistically efficient,” or “novel 2OP solver.”

## Experimental fairness audit

### Required comparison layers

1. **Hypothesis only:** 3p, 1.5p, 2OP, and any minimal conic baseline; no refinement.
2. **Common point-only refinement:** identical residual, initialization policy, stopping rule, and inlier set policy.
3. **Orientation refinement ablation:** heuristic weights, covariance-aware weights, oracle tangents, and image-estimated tangents.
4. **Robust fitting:** equal proposals, equal theoretical confidence with a sensitivity curve over orientation reliability, equal solver time, and equal total time.
5. **Full-information estimator:** Direct3pFit over all inliers and full ellipse-conic fitting, clearly labeled as non-minimal.

### Fixed primary metrics

- 3D center Euclidean error;
- radial/depth and lateral center error;
- failure/rejection rate with reason;
- hypothesis and total runtime distributions;
- RANSAC success probability at fixed budgets;
- condition number/smallest singular value;
- contour angular residual and tangent angular residual;
- downstream rotation/translation error only in an explicitly separate calibration experiment.

### Necessary experiment axes

- contour position noise: including 0, subpixel, 1 px, and 2 px reference settings;
- tangent source: oracle, independently perturbed, and image-estimated;
- tangent noise/reliability sensitivity;
- visible arc fraction and contiguous/random occlusion;
- projected diameter, range, and off-axis position;
- point count and nonuniform/adjacent sampling;
- blur, contrast, texture, highlights, shadows, and occlusion boundaries;
- intrinsic and distortion perturbation;
- sphere-radius error;
- outlier fraction and structured false edges;
- exact and near-degenerate sample geometry.

Trial seeds and generated observations must be shared across methods. No threshold may be tuned on the final test conditions. Threshold selection requires an explicit validation split or noise-model derivation.

## Evidence for and against candidate claims

| Candidate claim | Supporting evidence | Contradicting/missing evidence | Current verdict |
| --- | --- | --- | --- |
| 1.5p needs fewer sampled locations | One oriented plus one ordinary location forms a center hypothesis | Tangents require neighborhoods; total pixels and extraction time are not lower by definition | Narrow algebraic fact only |
| 1.5p is more accurate | Exact tests recover GT | Noisy minimal results can be worse than 3p; refined results tie in a reported setting | Not supported |
| 1.5p speeds RANSAC | Assumed success model gives 10 versus 13 proposals | `q` is assumed; tangent cost omitted; hypothesis quality differs | Not supported yet |
| Orientation refinement improves accuracy | It adds a geometrically relevant residual | Existing selected setting shows essentially identical point/oriented refined medians; covariance is wrong/unknown | Unresolved |
| 2OP improves robustness | Extra oriented constraint can verify consistency | No code/results; skew-line fitting may amplify tangent noise | Speculative |
| Better image centers improve calibration | CalibRGBD gives measurable extrinsic changes across center estimators | LiDAR/target geometry may dominate; no independent real GT | Unresolved |
| A negative regime map is scientifically useful | Current results already expose noisy-tangent weaknesses | Needs broad, preregistered evidence and a principled estimator | Plausible narrow contribution |

## Missing baselines

Essential before a central claim:

- verified MATLAB/final Direct3pFit outputs;
- Python Direct3pFit minimal and overdetermined;
- full ellipse-conic fit;
- Sun et al. LLSB;
- Shi et al. IMM if a verified implementation can be obtained;
- 1.5p and 2OP with identical scoring/refinement;
- point-only versus orientation-aware refinement.

Optional unless the scope expands to full calibration:

- CCSD and MARSCalib end-to-end pipelines;
- neural target proposal methods;
- BAGSFit and broad 3D primitive segmentation.

Neural development is not justified for the present question.

## VISAPP 2027 status

Official pages checked on 2026-11-18:

- Regular-paper first stage closed 2026-09-29.
- Position/regular second stage closed 2026-10-29.
- Late-breaking submission deadline: **2026-11-29 AOE**.
- Abstract and special-session deadline: 2026-12-03; the abstract track is not published in the proceedings.
- Regular submissions use double-blind review and 10,000–50,000 non-whitespace characters. Accepted full/short papers have 12/8-page camera-ready limits, with up to four paid extra pages.
- The official site did not expose a separate late-breaking format in the pages found; confirm its eligible paper type and limits in PRIMORIS or with the secretariat.
- AI-generated article content must be disclosed, but acknowledgments are removed for double blind review. Ask the secretariat where the anonymous-stage disclosure belongs.

Sources:

- https://visapp.scitevents.org/ImportantDates.aspx
- https://visapp.scitevents.org/Guidelines.aspx?y=2027
- https://visapp.scitevents.org/Templates.aspx
- https://www.scitepress.org/EthicsOfPublication.aspx

### Deadline assessment

Eleven days remain until the late-breaking deadline. The central 2OP method, novelty review, real tangent experiments, covariance model, wall-clock evaluation, and manuscript do not exist. A defensible complete paper cannot be promised on this schedule without relaxing the stated scientific standard. The recommended action is to continue the research and target a later venue unless the work independently reaches all evidence gates unusually quickly.

## Evidence gates for reconsidering GO

All gates are required for a solver/robust-estimation submission:

1. 2OP reference and production implementations agree on randomized exact fixtures and have explicit sign/failure policies; the current production implementation alone is insufficient.
2. Direct3pFit Python/MATLAB parity is demonstrated on exact, noisy, overdetermined, and ill-conditioned fixtures.
3. A primary-source novelty claim chart finds no equivalent prior solver or reframes the contribution honestly.
4. The experiment matrix is frozen before final runs, with validation/test separation.
5. Equal-proposal, equal-confidence sensitivity, equal-solver-time, and equal-total-time comparisons are complete.
6. Oracle and image-estimated tangents are both evaluated, with tangent error measured.
7. Failures and negative regimes are included; no method is rescued by post-hoc condition selection.
8. Orientation-aware refinement shows a meaningful benefit in a declared primary regime, or the paper is reframed as a negative/diagnostic study with a principled reliability model.
9. At least one public or independently verifiable experiment supports the central conclusion beyond a single analytic simulator.
10. AI disclosure, anonymity, artifact, and overlap with IJCV 2023/ICRA 2020 are resolved.

## GO / NO-GO recommendation

**NO-GO as of 2026-11-18.** This is not a judgment that the research direction is unpromising. It means that the evidence currently contradicts a simple superiority claim, the proposed 2OP method is absent, novelty is unverified, and the remaining VISAPP late-breaking schedule is inconsistent with the required validation.

Reassess after the gates above. A GO decision should require a precise claim of one of these forms:

- a verified reliability-aware orientation estimator improves success/error at equal total time in preregistered regimes;
- 2OP supplies a measurable and robust hypothesis-verification advantage despite not improving raw accuracy;
- a broad negative/positive regime map establishes when tangent information is beneficial, neutral, or harmful, with a principled explanation.

## Strongest defensible contribution remaining

The strongest current direction is a failure-aware study and estimator for **orientation-aware robust perspective sphere fitting**, not a claim that a smaller minimal sample is inherently more accurate. The contribution must include the cost and covariance of tangent estimation, identify counterexamples, and show whether any improvement survives identical refinement and equal total computation.
