# Evaluation record: 2026-11-17

This directory is the compact, versioned record for the first full Python evaluation. Raw CSV files and downloaded datasets remain under ignored `results/` and `data/`. All generated summaries use seed 0.

## Validation scope

- 11 unit/integration tests passed on Python 3.14.2, NumPy 2.4.3, SciPy 1.17.1, and the installed OpenCV.
- Synthetic image-center experiments: 100 paired trials, nine point/tangent-noise conditions, two solvers, and three refinements. Each budget mode contains 5,400 fits and zero reported solver failures.
- CalibRGBD Synthetic: 11 depth-noise groups, 100 calibrations per group, and seven center estimators: 7,700 runs, zero solver failures.
- SHREC 2022: all 185 test-set sphere clouds and seven 3D fitting variants: 1,295 method-runs. The adjacency method accounts for all 76 failures.

## Selected synthetic result

At 0.5 px point noise and 3 degrees tangent noise:

| Budget/refinement | 3p median center error | 1.5p median center error | Paired median `1.5p - 3p` |
| --- | ---: | ---: | ---: |
| 300 proposals, none | 5.16 mm | 6.82 mm | +0.72 mm, 95% bootstrap CI [-0.67, 2.93] mm |
| 300 proposals, point refinement | 1.9790 mm | 1.9790 mm | approximately 0, CI approximately 0 |
| confidence model, point refinement | 2.19 mm | 4.03 mm | see paired JSON |

Equal-confidence counts are 13 proposals for 3p and 10 for 1.5p, using `p=0.999`, inlier probability `w=0.75`, and conditional orientation reliability `q=0.95`. This model is an explicit assumption, not an estimated property of a real detector. The current study does not support a noisy-tangent accuracy advantage for 1.5p.

## Selected CalibRGBD result

Mean translation error / mean rotation error:

| Method | NoiseDepth0 | NoiseDepth2 | NoiseDepth4 |
| --- | ---: | ---: | ---: |
| Full ellipse-conic | 1.49 mm / 0.0501 deg | 1.82 mm / 0.1241 deg | 2.52 mm / 0.2001 deg |
| 1.5p refined | 2.62 mm / 0.0795 deg | 2.86 mm / 0.1422 deg | 3.36 mm / 0.2113 deg |
| Direct3pFit | 2.71 mm / 0.0825 deg | 2.94 mm / 0.1440 deg | 3.43 mm / 0.2125 deg |
| LLSB | 2.71 mm / 0.0818 deg | 2.94 mm / 0.1436 deg | 3.43 mm / 0.2122 deg |

These numbers evaluate center estimation and registration from benchmark-provided processed ellipses and spheres. They do not evaluate raw-image detection. The full ellipse method receives more measurements than a minimal hypothesis. Published CalibRGBD DLT/Boas/Staranowicz outputs are retained in the ignored raw result for context, but are not ranked against this table because those pipelines also estimate depth intrinsics.

## Selected SHREC result

| Method | Failures | Median center error | 95th percentile |
| --- | ---: | ---: | ---: |
| Geometric, known radius | 0/185 | 0.00228 | 0.0538 |
| Geometric, unknown radius | 0/185 | 0.00295 | 0.0551 |
| RANSAC random, known radius | 0/185 | 0.00157 | 0.0506 |
| RANSAC random, unknown radius | 0/185 | 0.00410 | 0.0627 |
| RANSAC adjacency, unknown radius | 76/185 | conditional median near zero | 1.1988 among successes |
| RANSAC distance, unknown radius | 0/185 | 0.00900 | 0.5986 |

Errors are in SHREC dataset units; normalized errors are in the JSON summary. The adjacency result is reported without test-set threshold tuning and demonstrates a serious ordering/visibility mismatch.

## Files

- `synthetic_equal_proposals*.json`: run metadata, method summaries, and paired differences.
- `synthetic_equal_confidence*.json`: the same for the explicit confidence budget.
- `calibrgbd_summary.json`: every noise/method summary.
- `shrec2022_summary.json`: every 3D method summary, including failures.

See `docs/EVALUATION.md` for the complete protocol and interpretation rules.
