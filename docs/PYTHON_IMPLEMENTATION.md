# Python implementation status

Updated 2026-11-17. The user selected Python as the full implementation language. MATLAB is historical reference code, retained on separate branches. The current work delivers a functioning classical Python calibration pipeline and an initial oriented-edge research implementation; it does not establish final-paper numerical parity or complete the broader research roadmap.

## Branch layout

- `legacy/matlab-master`: `1c660fb0da46357c89c5feb50a483dc363e088a9`, original master MATLAB files.
- `legacy/matlab-dev`: `b4d14a27edab2a201a1f74428dc63e91aadad16b`, original local dev snapshot including exploratory classes/scripts.
- `python-reimplementation`: active Python working branch. All 51 tracked `.m` files have been removed here; source remains recoverable on both legacy branches. Example data/metadata and historical saved outputs remain available.

The legacy branch names are local. `legacy/matlab-dev` tracks the existing remote `dev`; `legacy/matlab-master` and `python-reimplementation` do not yet have remote counterparts. No remote publication or branch replacement was performed at the time of this report.

## Available modules

| Module | Functionality |
| --- | --- |
| `geometry` | Calibrated rays and tangent planes, exact/overdetermined equal-cone sphere fit, 1.5p solver, angular contour/orientation residuals, robust refinement, projection |
| `conics` | Implicit/parametric ellipses, sphere projection, conic-to-sphere initialization/refinement, direct ellipse fit, circle consensus, legacy approximate ellipse distance |
| `robust` | Seeded 3p/1.5p RANSAC, separate refinement ablations, confidence iteration helper, updated final inlier masks |
| `lidar` | Algebraic sphere hypotheses, adjacency/distance/random consensus, known/unknown-radius geometric fitting, sequential plane removal |
| `registration` | Proper determinant-corrected SE(3), robust correspondence filtering, inverse transform, rotation/translation errors |
| `edges` | Canny/Sobel, local quadratic subpixel gradient maxima, tangents, ROI/mask, optional undistortion |
| `pipeline` | Ordered image/cloud observations → cloud radius/centers → image centers → LiDAR-to-camera transform; per-frame failure context and consistency diagnostics |
| `data` | JSON manifests, XYZ/CSV/NPY/ASCII/binary PLY, selective legacy MAT metadata, bundled example matching |
| `synthetic` | Analytic silhouettes/tangents, pixel/orientation noise and outliers, partial cloud observations, rasterized images, paired multi-frame scene |
| `visualization` | Projection/sphere overlays, RGB-colored clouds and PLY export |
| `evaluation`, `cli` | Trial CSV, configuration/environment JSON, bootstrap median intervals, CLI calibration/demo/benchmark/example runners |

## Relationship to MATLAB

The normalized-ray equal-cone equation in `fitSphere3p` is implemented for both three-point and overdetermined cases. Both high-level camera pathways and all three cloud sampling families are available. The Python pipeline deliberately uses conventional rigid translation and determinant correction instead of reproducing the legacy registration defects. It uses geometric robust least squares instead of Eberly's original fixed-point cloud iteration, and normalized conic coefficients instead of the original bounded coefficient-scale optimization. Its default image consensus score is angular error rather than MATLAB's approximate ellipse distance. The `circle` path retains the latter approximation as its reported score.

These choices implement the pipeline's functionality without preserving every numerical detail. They must not be labelled a numerically equivalent port. The dev branch's Sun16/Shi19 exploratory solvers, spline experiments and interactive MATLAB diagnostics have not been translated wholesale; recover them selectively when a verified paper/baseline comparison requires them. Published-method identification still needs the actual paper equations and missing experiment dependencies.

The oriented solver enumerates the squared tangency equation conceptually: with `s=lambda*u+sigma*r*n`, its two roots have denominators `+1-u.v` and `-1-u.v`. The second root cannot have both positive forward contact depths; the implementation selects the first with the normal sign chosen for positive lambda. Tests exercise tangent-sign flips, reversed sample order, multiple off-axis/on-axis scenes, radius changes, unequal focal lengths/skew and degeneracy. This is implementation validation, not a novelty claim.

Orientation refinement uses angular errors scaled separately for ray and tangent-plane uncertainty. It does not yet propagate their correlated image-measurement covariance, infer confidence scales from gradients, or provide calibrated uncertainty estimates. The subpixel edge procedure is a classical local approximation and does not guarantee the physical silhouette when highlights/texture create competing edges.

## Executed validation

Environment: Windows, Python 3.14.2, NumPy 2.4.3, SciPy 1.17.1; existing OpenCV installation. Standard-library unittest avoids an extra test dependency. A PEP 517 wheel was built successfully after obtaining setuptools as an isolated build dependency.

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s tests -v
python -m spherecalib demo --iterations 200 --output results/demo.json
python -m spherecalib benchmark --trials 30 --iterations 300 --output results/benchmark.csv
python -m spherecalib benchmark --trials 100 --iterations 300 --budget equal-proposals --output results/synthetic_equal_proposals.csv
python -m spherecalib benchmark --trials 100 --budget equal-confidence --confidence 0.999 --inlier-probability 0.75 --orientation-reliability 0.95 --output results/synthetic_equal_confidence.csv
python -m spherecalib example examples/example1 --radius-max 3 --iterations 1000 --output results/example1.json
python -m spherecalib example examples/example2 --planes 5 --close-threshold 1.5 --radius-min 0.2 --radius-max 0.4 --iterations 2000 --output results/example2_filtered.json
python -m spherecalib example examples/example3 --planes 4 --radius-min 0.2 --radius-max 0.4 --iterations 2000 --output results/example3_filtered.json
python -m spherecalib evaluate-calibrgbd data/calibrgbd/Synth --output results/calibrgbd.csv
python -m spherecalib evaluate-shrec data/shrec2022/repo/dataset/test --output results/shrec2022.csv
```

- **11 tests pass:** exact/noisy geometry, conic conversions, both image solvers and refinement ablations, all cloud sampling/radius modes, plane removal, registration/reflection/outliers, image/manifest-to-extrinsics integration, PLY formats and projection export.
- **Synthetic paired-cloud demo:** translation error 0.0010400 m, rotation error 0.00044037 rad (~0.0252°). Ideal image rays, 1 mm cloud noise, 50 cloud outliers per frame, known radius 0.12 m, six target positions, 200 proposals. These are synthetic GT errors for this scene only.
- **Synthetic ablation sweep:** 30 paired trials × 9 position/orientation-noise settings × 2 solvers × 3 refinement modes = 1,620 fits; 300 proposals each; 25% injected image outliers; no reported solver failures. This is one scene and fixed threshold, not evidence of general superiority. Bootstrap intervals are conditional on successful trials; failure counts are retained separately. Runtime measurements include Python overhead and are not controlled hardware comparisons.
- **Expanded synthetic comparison:** two 100-trial sweeps produced 5,400 fits each with no solver failures. Equal-proposal mode used 300 proposals for both methods. Equal-confidence mode used 13 proposals for 3p and 10 for 1.5p from the declared `w=0.75`, `q=0.95`, `p=0.999` model. At 0.5 px point noise and 3 degrees tangent noise, the unrefined median center errors were 5.16 mm (3p) and 6.82 mm (1.5p) under equal proposals. Common point refinement made their medians 1.9790 mm and 1.9790 mm; the paired median difference was effectively zero. Under equal confidence the refined medians were 2.19 mm (3p) and 4.03 mm (1.5p). This experiment does not support an accuracy advantage for 1.5p under noisy tangents.
- **CalibRGBD Synthetic:** all 11 depth-noise levels and 100 calibrations per level were evaluated for seven image-center variants, for 7,700 runs and zero solver failures. At `NoiseDepth0`, mean translation/rotation errors were 1.49 mm/0.0501 degrees for full fitted ellipse-to-conic, 2.62 mm/0.0795 degrees for refined 1.5p, 2.71 mm/0.0825 degrees for Direct3pFit, and 2.71 mm/0.0818 degrees for LLSB. At `NoiseDepth4` they were 2.52 mm/0.2001 degrees, 3.36 mm/0.2113 degrees, 3.43 mm/0.2125 degrees, and 3.43 mm/0.2122 degrees. These use benchmark-provided processed ellipses and exact synthetic GT; they do not evaluate raw edge detection. The conic method also receives the full fitted ellipse rather than a minimal sample.
- **SHREC 2022 spheres:** all 185 exact-GT sphere test clouds were evaluated with seven 3D fitting variants. Known-radius geometric fitting had median center error 0.00228 dataset units; unknown-radius geometric fitting had 0.00295. Random-sample robust known-radius fitting had 0.00157. The legacy-style adjacency sampler failed in 76/185 cases and had a 95th-percentile center error of 1.20 among successes, revealing that its ordering assumption is unsuitable for this benchmark. SHREC units are dataset units, and this is a 3D subsystem test rather than camera–LiDAR calibration.
- **Bundled synthetic example1:** estimated radius 1.993300 m versus 2 m GT; mean image-center error 0.017403 m; mean cloud-center error 0.004359 m; center registration RMSE 0.008999 m. This uses actual bundled images/clouds and independently estimated radius. Pixel origin is converted by subtracting one from MATLAB principal points. Original GT extrinsic conventions are not inferred, so no extrinsic GT error is reported for this example.
- **Real examples2/3:** no independent center/extrinsic GT was established. Unrestricted first runs had registration RMSE 0.9289 m / 3.4641 m. Filtering radius to 0.2–0.4 m, with the commands above, reduced these to 0.2864 m / 1.8569 m. Both remain failed target-selection/calibration checks, not successful benchmark results. Restricting radius around the example scripts' approximate 0.3 m is a prior, not certified GT. Image ROIs/cloud segmentation and tuned detector thresholds remain necessary.

Generated files are written to ignored `results/`. Compact reference records are retained under `docs/experiments/initial-python/` and `docs/experiments/2026-11-17-evaluation/`. The MATLAB numerical-equivalence suite has not run because a MATLAB/Octave executable was not established. The former MAT inspection error is resolved for named numeric/path metadata through selective loading; unsupported MATLAB string arrays are bypassed using explicit filename prefixes and frame indices.

## Remaining research work

1. Recover the final IJCV solver baseline and run MATLAB-generated parity fixtures, including noisy overdetermined cases and failure policies.
2. Improve target proposals/association on real data. Add optional learned segmentation through an explicit detector interface, followed by image-based subpixel localization; no neural weights or inference are currently present.
3. Complete edge covariance propagation, orientation-confidence calibration, derivative checks and joint cross-modal refinement/uncertainty analysis.
4. Reproduce the external MARSCalib/CCSD pipelines in their required ROS2/GTSAM/SAM environments and compare on a declared common real split. Do not treat pipeline-fitted centers as independent physical GT.
5. Expand synthetic scenes/noise/occlusion/blur sweeps and obtain independently surveyed real calibration validation. Add BAGSFit only after fixing a target segmentation protocol and split without tuning on SHREC test data.

The legacy project remains intact on its branches while these additions can evolve entirely in Python.
