# SphereCalib continuation: repository audit and implementation plan

Date: 2026-11-16. This is an inspection and planning deliverable, not a completed reimplementation or literature review.

**Superseded language decision (2026-11-17):** The user has explicitly chosen a full Python reimplementation, with MATLAB retained on legacy branches. The implementation now lives on `python-reimplementation`; `legacy/matlab-master` and `legacy/matlab-dev` preserve both inspected MATLAB snapshots. This document remains the historical initial audit. See `PYTHON_IMPLEMENTATION.md` for current functionality and validation; its original MATLAB-first implementation sequence below is no longer the active language decision.

## Scope and evidence

The current user request takes precedence over the attached handoff. The handoff records earlier proposals, instructions and unverified results; those are research context, not automatic authorization to execute its entire roadmap. This first stage inspects MATLAB and establishes a Python implementation plan. The user's preference for research/reimplementation in MATLAB is retained: MATLAB remains the reference and a supported research target; a wholesale language migration is not assumed.

Inspected sources:

- Working tree `master`, commit `1c660fb`, initially clean; MIT license in `LICENSE.txt`.
- Locally available remote-tracking `origin/dev`, commit `b4d14a27edab2a201a1f74428dc63e91aadad16b`, dated 2022-11-07. Inspected using `git show` without changing branches. This is a local snapshot, not a verified current upstream checkout.
- `C:/Users/tekla/Downloads/SphereCalib_Codex_handoff.txt`, read in full.
- `C:/Users/tekla/Downloads/spherecalib_python_prototype.zip`, inspected in place. No archive code was installed or copied into the baseline.

## Available MATLAB implementation

| Area | Checked-out implementation and interface | Porting implications |
| --- | --- | --- |
| Entry point | `sphereCalib/sphereCalib.m`: paired image/cloud cells, four intrinsics, algorithm flags; returns rotation, translation, centers, mean radius, cloud inliers | Correspondence is supplied by frame order; this is not automatic multi-target association |
| Camera normalization | `imageProc/pixel2meter.m`: `(xy-[u0,v0])./[fu,fv]` | Output is dimensionless normalized image coordinates despite its name; no skew/distortion model |
| Image extraction | `rgb2gray`, Canny/Sobel, index extraction and row/column swap | Preserve MATLAB pixel origin and intrinsics convention when loading in Python |
| Camera model 1 | `detectEllipseWithCircle`, ellipse conversion, `implEllipse2implSphereIni/Opt` | Circle-based consensus followed by nonlinear coefficient fitting |
| Camera model 2 | `detectSphereRand3p`: sample three points, `fitEllipse3p`, approximate ellipse-distance scoring, final `fitSphere3p` | Port the hypothesis, score and refit separately; the ZIP uses a different scoring/refinement pipeline |
| Direct sphere fitting | `fitSphere3p(XY,r)`: normalize `[x,y,1]`, solve `U*a=1`, set `c=1/norm(a)`, return `r*c*a/sqrt(1-c^2)` | Supports overdetermined fitting too; noisy least squares must be compared, not only exact triples |
| Cloud detection | Adjacency k-NN, distance-neighborhood or four-point RANSAC; radius gates; radial-distance classification | Existing pipeline estimates radius from clouds rather than assuming known radius throughout |
| Cloud fitting | `fitSphereNonit`, `fitSphere4p`, `fitSphereLsq` | Algebraic hypotheses and iterative center/radius refit; preserve original stochastic initialization for reference runs |
| Registration | `pointReg(S0Cam,S0Lid)`: centered cross-covariance SVD | Camera-to-LiDAR rotation; existing returned translation requires explicit treatment below |
| Examples | `example1` synthetic; `example2/3` real, images, XYZ clouds, MAT configurations, saved projections | Existing local starting data; scripts are demonstrations, not an automated regression suite |

`init.m` recursively adds the repository to the MATLAB path. Expected dependencies from source inspection: Image Processing Toolbox (`edge`, `rgb2gray`), Statistics and Machine Learning Toolbox (`KDTreeSearcher`, `knnsearch`, `rangesearch`), Optimization Toolbox (`lsqnonlin`, `optimoptions`) and Symbolic Math Toolbox (`sym` on the ellipse optimization path). Availability/version behavior has not been tested.

### Additional code in local origin/dev

The dev snapshot adds `sphereCalibNew/PointSet2D`, `PointSet3D`, `ImageData`, `ImplicitEllipse`, `ParametricEllipse`, `SphereConverter`, `SphericalProjection`, `EllipseConverter`, `FileReaderWriter`, and `PointRegistration`. There is also a separate `sphereEllipseConverters/RefactoredSphereCalib` tree with overlapping class names, including `Sphere`.

`PointSet2D` contains `fitSphere`, `fitSphereA/B`, `fitSphereSun16`, `fitSphereShi19/B/C`, ellipse alternatives and detection. `Test_Belnsor_SphereCenter.m` labels its `fitSphere` output `Direct3pFit` and compares it with Sun16/Shi19 variants. Thus Direct3pFit exists as an experiment label, not an identified standalone function. This is promising research source, but equivalence to the final IJCV 2023 publication remains unverified. Do not infer that equivalence from file names or labels.

Dev also adds known-radius cloud fitting (`fitSphereLsqRKnown`), registration RANSAC, edge-point refinement, and spline/conditioning experiments. Some scripts depend on external Blensor data/helpers, for example `readBlensorTestData` and `blensor_sph_S0_e2.mat`, not found by the inspected tree-name search. Treat them as research scripts needing dependency recovery. Resolve class/path ambiguity before using recursive `addpath` on dev.

## Findings to resolve before parity claims

1. `pointReg` returns `trans=mean(dst)-mean(src)`, whereas a conventional transform `dst=R*src+t` needs `t=mean(dst)-R*mean(src)`. Examples recompute this latter translation for projection/evaluation. Preserve the legacy output in reference fixtures and expose a separately named corrected SE(3) API. The SVD also lacks a determinant correction for reflections.
2. `detectSphereWithDistance` calls `fittingSphereNoniterative` and `fittingSphereLsq`; the supplied functions are named `fitSphereNonit` and `fitSphereLsq`. This path is incomplete as checked out.
3. Solvers/detectors lack comprehensive handling for rank deficiency, invalid square roots, empty consensus, insufficient points and nonfinite models. Add explicit failure statuses in new implementations without silently redefining the legacy baseline.
4. `fitEllipse3p` has an optical-axis special-case comparison between column `w` and row `[0 0 1]`; verify on-axis behavior in MATLAB. Ellipse distance is an approximation, not exact closest-point distance.
5. `implEllipse2implSphereOpt` requests Levenberg–Marquardt and supplies bounds; verify supported behavior in the installed MATLAB release. Preserve bounds/tolerances and record any required compatibility changes.
6. Distortion correction, uncertainty propagation, synchronization, learned inference and benchmark adapters are not established by the inspected entry point.

## Python ZIP assessment

The archive supplies `sphere_geometry.py`, `pipeline.py`, three test scripts, a benchmark, CSV, image and README, plus Python 3.13 bytecode. Its own README explicitly states that it is an independent implementation, not a validated MATLAB port.

`solve_3p` uses the same equal-cone equation for an exact triple, but accepts only a 3-by-3 ray matrix, unlike MATLAB's overdetermined refit. It also has its own degeneracy and visibility policies. `solve_1p5` chooses the tangent-normal sign using the second ray, projects the normal perpendicular to the first ray, and rejects small denominators. These policies require independent derivation and tests.

The benchmark passes normals to both methods and both receive orientation-aware refinement. Consequently its CSV compares complete mixed pipelines, not isolated 3p versus 1.5p hypotheses. It uses a fixed equal proposal count, a cone residual with depth-dependent sensitivity, an arbitrary orientation weight, no confidence intervals and no explicit failure-rate summary. The returned consensus mask is not recomputed after refinement. Existing exact testing covers one scene/triple/pair. Reported scores remain unverified; neither improvement nor novelty is established.

## Implementation sequence and acceptance gates

1. **Recover MATLAB reference.** Pin master/dev source hashes; map dev call graphs and missing assets; select the intended paper implementation. Keep baseline source intact. Add a MATLAB batch fixture exporter using explicit paths and seeds, with source hash, MATLAB release/toolboxes, inputs, outputs and failure diagnostics.
2. **Create a small Python package.** Use `pyproject.toml`, `src/spherecalib/`, `tests/`, `scripts/` and `configs/`; initially NumPy/SciPy and pytest as a development dependency. Select a supported Python version after checking dependency compatibility; the installed interpreter is 3.14, not the handoff's proposed 3.12. Introduce OpenCV only for the image stage and neural dependencies only when needed. Retain MIT attribution for translated code.
3. **Port deterministic geometry first.** Modules: `geometry` (rays, sphere/conic conversions, legacy direct fit), `registration` (legacy and conventional transforms), `data` (fixture schemas). Use one documented convention: meters, radians, undistorted pixels with explicit pixel origin, column-vector transform equations, and named `T_camera_from_lidar`. Convert legacy camera-to-LiDAR outputs explicitly.
4. **Establish numerical parity.** Shared stored inputs, not merely matching seeds across languages. Test exact and noisy triples, overdetermined contours, optical-axis/off-axis scenes, unequal focal lengths, large ranges, repeated/near-collinear points, invalid radii and near-degenerate conics. Compare intermediate rays, linear solve, cone angle, center and residuals. Start well-conditioned center tolerance at `atol=1e-9 m, rtol=1e-9`; record conditioning and calibrate tolerances from measured fixtures. MATLAB parity remains pending until MATLAB has actually executed.
5. **Implement oriented solvers in MATLAB and Python.** For unit rays, derive `s=lambda*u+sigma*r*n`, `n.u=0`, and both roots of `(lambda*u.v+sigma*r*n.v)^2=lambda^2`. Check first and second contact parameters `u.s>0`, `v.s>0`, camera outside sphere, and positive camera depth. Do not accept the handoff's selected root without testing the other branch. Test tangent sign flips, reversed sample order, coincident/nearby rays, invalid normals, noise and partial contours; record conditioning via `1-u.v`. Compare a two-oriented-point formulation separately.
6. **Run the first reproducible experiment.** Exact synthetic contours with analytic projected tangents, then paired pixel/orientation-noise sweeps. Save GT, sample indices and seeds. First compare hypotheses without refinement; then identical point-only refinement; then orientation refinement as a separate ablation. For RANSAC use common point scoring/threshold units, report equal-budget and equal-confidence experiments separately, and account for orientation reliability rather than assuming every point inlier has a correct tangent. Save trial-level CSV/JSON, failure rates, center/depth error, timings and paired bootstrap intervals. Start with a smoke test, then scale trials before scientific conclusions.
7. **Extend the full pipeline.** Add `edges`, `robust`, `refinement`, `lidar`, `evaluation` and `detection` modules. Preserve original unknown-radius cloud baseline; introduce known-radius geometric fitting as a separate method. Develop uncertainty-scaled contour/orientation residuals and verify Jacobians. Add classical subpixel contours before learned ROI proposals. Test registration on synthetic paired centers before real calibration.
8. **Research and benchmark verification.** Verify original paper equations and metadata, sphere-solver prior art, oriented contour estimators, learned boundary accuracy and recent calibration baselines using primary sources. Create `REVIEW.md` with evidence and unresolved claims. Inspect dataset schemas, units, frame conventions and annotation provenance before downloading large assets. Distinguish simulated exact GT, surveyed GT with uncertainty and fitted centers. CalibRGBD and all handoff citations/download links remain unverified in this initial local audit.

## Execution status and next concrete deliverable

No algorithms were modified and no prototype performance claims were adopted. No MATLAB/Octave command was found on PATH; this does not establish that MATLAB is absent from the machine. Python, NumPy, SciPy and OpenCV are discoverable; pytest is not. A read-only SciPy `whosmat` inspection of the example MAT files failed with `TypeError: 'NoneType' object is not iterable`; investigate serialization/version compatibility before choosing a loader. Example-script GT variable names are therefore source evidence, not verified MAT contents.

Next deliverable: isolated MATLAB reference exporter plus a minimal Python geometry package and shared fixtures for exact/noisy 3p parity; then the independently derived 1.5p experiment in both languages. MATLAB installation discovery and fixture execution are prerequisites for a numerical equivalence claim. Model downloads, upstream freshness checks, dataset evaluation and a verified literature review have not been performed in this stage.
