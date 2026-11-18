# Evaluation protocol

This protocol follows the research handoff's central rule: distinguish exact synthetic ground truth, surveyed physical ground truth with uncertainty, and estimates produced by fitting. A fitted sphere center is never relabelled as independent ground truth.

## Reproducible commands

From PowerShell at the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
.\scripts\test.ps1 -BuildWheel
```

The complete current evaluation is:

```powershell
.\scripts\download_calibrgbd.ps1
.\scripts\download_shrec_spheres.ps1
.\scripts\evaluate.ps1 -Trials 100
```

For a fast check:

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s tests -v
python -m spherecalib benchmark --trials 3 --iterations 100 --output results/smoke.csv
python -m spherecalib evaluate-calibrgbd data/calibrgbd/Synth `
  --noise NoiseDepth0 --limit 2 --output results/calibrgbd_smoke.csv
```

Raw downloads and generated `results/` are ignored by Git. Compact reference results live under `docs/experiments/`. The downloader verifies the official archive's byte count and SHA-256 and rejects archive paths outside its extraction directory.

## Evaluation layers

Report each layer independently so an improvement cannot be credited to the wrong component.

1. **Minimal hypothesis:** exact/noisy samples, without consensus or refinement. Compare Direct3pFit/3p, LLSB, five-point conic fitting, 1.5p, and the 2OP geometric baseline. Record invalid/degenerate hypotheses.
2. **Robust estimator:** use identical generated measurements, inlier definitions and residual units. Compare equal proposal counts and equal theoretical success confidence separately. The 1.5p confidence model includes the probability that the oriented sample has a reliable tangent.
3. **Refinement:** hypothesis-only, identical point-only refinement, and orientation-aware refinement are separate ablations. Recompute the consensus after refinement.
4. **Detection:** measure candidate-mask Dice/IoU where masks exist, boundary displacement in pixels, tangent angular error and false-edge rate. A neural proposal and its subpixel boundary localization are separate modules.
5. **3D fitting:** evaluate known-radius and unknown-radius fitting, full/partial visibility, noise, outliers and clutter. Separate segmentation failure from center fitting on a supplied target crop.
6. **Extrinsic calibration:** use the same center correspondences for all registration methods. Report rotation and translation error against exact/surveyed GT, center-registration RMSE, reprojection error, failures and runtime.
7. **Full pipeline:** report results from raw images/clouds separately from results using benchmark-provided processed ellipses/spheres.

## Synthetic design

Use fixed, versioned configurations and save every trial seed. Sweep at least:

- range, sphere radius/projected diameter, position across the image and camera intrinsics;
- pixel noise, blur, contrast, distortion/intrinsic perturbation and rasterization;
- tangent angular noise and probability that a contour inlier has a reliable orientation;
- outlier ratio, occlusion and visible arc fraction, including adjacent-point ill-conditioning;
- point-cloud radial/range noise, missing caps, outliers, background planes and point count;
- number and spatial conditioning of sphere-center correspondences.

The IJCV paper's comparable setup should be reproduced explicitly: `s=[-0.95, 0.35, 3.00] m`, `r=0.35 m`, 3–700 contour points, 0/1/2 pixel Gaussian noise, and 1,000 repeats. Add the new tangent-noise axes without changing that reference experiment.

## Fair RANSAC budgets

For inlier probability `w`, sample size `s`, and target confidence `p`, use

`N = ceil(log(1-p) / log(1-w^s))`.

For 1.5p, the current explicit model uses per-point inlier probability `w` and conditional oriented-edge reliability `q`, giving one-hypothesis success probability `w²q`. Because `q` is rarely known on real data, report a sensitivity curve over plausible values and an equal-wall-clock comparison as well. Never interpret the lower sample count alone as an accuracy improvement.

## Metrics and statistics

Primary image-sphere metrics are 3D center error, separated into depth/radial and lateral components; ellipse boundary distance; tangent angular residual; failure rate; and runtime. Cloud metrics are center/radius error, point residuals, precision/recall of sphere points and failure rate. Calibration metrics are geodesic rotation error in degrees, translation error in millimetres, 3D alignment error and reprojection error.

Use paired trials and report the distribution of method differences. The CLI writes per-method summaries plus paired `1p5 - 3p` median differences and 95% bootstrap intervals. Confidence intervals are conditional on successful paired trials; failures are counted separately. For final claims, include a failure-aware sensitivity analysis rather than dropping failed trials. Report median, interquartile range and 95% interval; add mean/standard deviation only for comparison with publications that use them.

Runtime comparisons require a fixed machine, thread count, warm-up policy and method implementation language. Record Python/package versions, CPU/GPU, model/weight identifiers and data checksum. Neural methods must use fixed released weights with no benchmark-specific training unless a separate training split is declared.

## Dataset matrix

| Dataset | Use | Ground-truth status | Current status |
| --- | --- | --- | --- |
| Bundled SphereCalib example1 | End-to-end regression | Exact synthetic centers/radius/extrinsics in MAT metadata | Executed |
| CalibRGBD Synthetic | Primary public RGB-D calibration benchmark | Exact constructed centers, radius, intrinsics and extrinsics | Adapter and full processed-measurement evaluation implemented |
| CalibRGBD Random/Equidistant/Double | Real detection/calibration consistency | Manual segmentations/fitted centers plus manufacturer parameters | Registered; large archives not downloaded |
| MARSCalib samples | Modern real end-to-end comparison | Pipeline-estimated centers; no independently surveyed center GT identified | External ROS2 baseline registered, not executed |
| SHREC 2022 | 3D sphere fitting and primitive classification | Exact synthetic primitive parameters | Adapter implemented; all 185 test spheres evaluated |
| BAGSFit | Cluttered 3D segmentation/fitting | Synthetic primitive parameters and point labels | Registered; data/model evaluation pending |
| TUM RGB-D metallic sphere | Qualitative real RGB-D robustness | Known 150 mm diameter and camera trajectory; no per-frame surveyed center GT | Registered; not an accuracy-GT dataset |

The machine-readable version is `datasets/registry.json`.

## Algorithm comparison matrix

| Component | Methods | Status |
| --- | --- | --- |
| Image center | Direct3pFit/3p; experimental 1.5p and 2OP hypotheses; Sun et al. LLSB; direct five-point ellipse + conic-to-sphere; legacy circle consensus | Implemented; historical processed CalibRGBD results predate 2OP |
| Robust estimation | 3p/1.5p/2OP RANSAC; point/orientation refinement | Equal-proposal/equal-confidence modes implemented; equal-wall-clock mode pending |
| Cloud center | algebraic hypothesis; known/unknown-radius nonlinear refit; random/distance/adjacency sampling | Implemented |
| Registration | determinant-corrected least squares; robust three-center consensus | Implemented |
| CalibRGBD published | DLT; Staranowicz 2014; Boas 2019 | Official benchmark outputs imported and labelled as supplied results |
| Modern sphere calibration | RA-L 2024 CCSD; MARSCalib | External comparison targets; source/runtime environment needed before execution |
| Learned proposals | SAM/MARSCalib; boundary detectors | Interface/research target; no weights downloaded and no result claimed |

## Interpretation rules

- CalibRGBD synthetic processed ellipses test center estimation and calibration under the benchmark's measurement noise. They do not test this repository's raw edge detector.
- Official CalibRGBD method results are imported benchmark outputs, not reruns in the Python environment.
- MARSCalib real samples and CalibRGBD real sets support consistency and cross-method comparisons, but fitted centers are not independent physical GT.
- SHREC/BAGSFit evaluate the 3D subsystem only; they cannot establish paired RGB–LiDAR calibration performance.
- A public score is reported only with the exact archive/version, split, preprocessing, units, convention and failures.
