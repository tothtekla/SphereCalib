# SphereCalib

Python sphere estimation and camera–LiDAR calibration, continuing the MATLAB SphereCalib research project by Tekla Tóth et al. The active implementation is Python. Original MATLAB code is preserved on local branches `legacy/matlab-master` (ICRA implementation) and `legacy/matlab-dev` (research snapshot).

The package runs a complete classical pipeline: paired images and clouds → subpixel edge candidates → robust image/cloud sphere fitting → corresponding centers → rigid camera–LiDAR registration. It includes experimental 1.5-point and two-oriented-point geometric baselines, synthetic ablations, a CalibRGBD Synthetic adapter, and SHREC 2022 sphere-fitting evaluation. Numerical MATLAB parity, solver novelty, neural inference, and independently surveyed real-world validation remain separate research milestones.

## Install and run

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
python -m unittest discover -s tests -v
python -m spherecalib demo --output results/demo.json
python -m spherecalib benchmark --trials 30 --iterations 300
python -m spherecalib example examples/example1 --radius-max 3 --output results/example1.json
```

For the existing environment without installation, set `$env:PYTHONPATH = 'src'` before running the commands. Benchmark defaults produce 2,430 fits across paired noise conditions, three hypothesis methods, and refinement ablations. This is an equal-proposal experiment, not an equal-confidence comparison.

Convenience scripts run the same commands from the repository root:

```powershell
.\scripts\test.ps1 -BuildWheel
.\scripts\download_calibrgbd.ps1
.\scripts\download_shrec_spheres.ps1
.\scripts\evaluate.ps1 -Trials 100
```

The evaluation script runs unit/integration tests, equal-proposal and equal-confidence synthetic studies, the bundled exact-GT example, the official CalibRGBD synthetic benchmark, and SHREC when its sparse sphere subset is present. See [the evaluation protocol](docs/EVALUATION.md) for metrics, fairness rules, datasets, baselines and interpretation, and [the current measured results](docs/experiments/2026-11-17-evaluation/README.md) for the versioned benchmark record.

## Calibrate your data

```powershell
python -m spherecalib calibrate frames.json --output results/calibration.json
```

Example `frames.json` (at least three noncollinear target positions are required):

```json
{
  "units": "meters",
  "pixel_origin": 0,
  "K": [[950, 0, 640], [0, 950, 480], [0, 0, 1]],
  "radius_m": 0.12,
  "config": {
    "image_method": "1p5",
    "refinement": "points",
    "cloud_method": "adjacency",
    "radius_bounds": [0.10, 0.14],
    "iterations": 1000,
    "seed": 0
  },
  "frames": [
    {"image": "images/001.png", "cloud": "clouds/001.xyz", "roi": [300, 200, 250, 250]},
    {"image": "images/002.png", "cloud": "clouds/002.xyz", "roi": [400, 300, 250, 250]},
    {"image": "images/003.png", "cloud": "clouds/003.xyz", "roi": [500, 150, 250, 250]}
  ]
}
```

Paths resolve relative to the manifest. Cloud formats: XYZ, TXT, CSV, NPY and PLY (ASCII and binary scalar vertex properties). Images load as OpenCV BGR. Supply optional OpenCV `distortion` coefficients to undistort before extracting edges; ROIs then refer to the undistorted image. Omit `radius_m` to estimate the radius per cloud and use their mean for camera estimation. Provide cropped clouds and image ROIs where background objects compete with the sphere.

Output `T_camera_from_lidar` follows `X_camera = R @ X_lidar + t` using column vectors. Coordinates are meters; image coordinates are zero-based undistorted pixels; errors use radians and meters. `registration_consistent` checks center-registration RMSE against configurable `registration_consistency_m` (default 0.05 m). It is an internal consistency diagnostic, not ground-truth accuracy or proof of correct target association.

Image methods: `3p`, `1p5`, `2op`, `circle`. Refinement: `none`, `points`, `oriented` (explicit angular scales; preliminary independent-error model). The oriented methods are research baselines, not established novelty or superiority claims. Cloud methods: `adjacency`, `distance`, `random`; known-radius and unknown-radius fits are supported. Plane removal and robust center registration are configurable. Correspondence is supplied by frame order; there is one target per frame.

See [implementation status and measured checks](docs/PYTHON_IMPLEMENTATION.md), [evaluation protocol](docs/EVALUATION.md), [adversarial publication review](docs/CRITICAL_REVIEW.md), [oriented-solver mathematical audit](docs/ORIENTED_SOLVER_AUDIT.md), [AI usage record](AI_USAGE.md), [verified research review](REVIEW.md), [initial repository audit](docs/REIMPLEMENTATION_PLAN.md), and `python -m spherecalib --help`.

## MATLAB legacy

Both legacy branches point to existing committed history. No MATLAB source needs to remain in the Python working tree. The examples' images, clouds, MAT metadata and historical output artifacts are retained as data. Avoid switching branches with uncommitted changes; use a separate checkout/worktree if you need MATLAB alongside Python.

## Citation and attribution

Original project: Tóth, T., Pusztai, Z., Hajder, L., *Automatic LiDAR-Camera Calibration of Extrinsic Parameters Using a Spherical Target*, ICRA 2020, pp. 8580–8586. DOI: `10.1109/ICRA40945.2020.9197316`.

If you use the camera–LiDAR calibration pipeline, please cite the ICRA 2020 paper:

```bibtex
@inproceedings{Toth_ICRA_2020,
  author    = {Tekla Tóth and Zoltán Pusztai and Levente Hajder},
  title     = {Automatic LiDAR-Camera Calibration of Extrinsic Parameters Using a
               Spherical Target},
  booktitle = {2020 {IEEE} International Conference on Robotics and Automation ({ICRA})},
  pages     = {8580--8586},
  publisher = {{IEEE}},
  year      = {2020},
  url       = {https://doi.org/10.1109/ICRA40945.2020.9197316},
  doi       = {10.1109/ICRA40945.2020.9197316}
}
```

The image-based sphere estimation and Direct3pFit research is described in Tóth, T. and Hajder, L., *A Minimal Solution for Image-Based Sphere Estimation*, International Journal of Computer Vision 131, 1428–1447 (2023). DOI: [10.1007/s11263-023-01766-1](https://doi.org/10.1007/s11263-023-01766-1). If you use the image-sphere estimators or build research on this implementation, please cite:

```bibtex
@article{toth2023minimal,
  author  = {Tekla Tóth and Levente Hajder},
  title   = {A Minimal Solution for Image-Based Sphere Estimation},
  journal = {International Journal of Computer Vision},
  volume  = {131},
  pages   = {1428--1447},
  year    = {2023},
  doi     = {10.1007/s11263-023-01766-1}
}
```

The equal-cone image fit and classical processing architecture follow the original source. Python robust fitting/refinement and proper SE(3) translation are independently implemented, with documented differences; no MATLAB numerical-equivalence claim is made. MIT license and original attribution are retained in [LICENSE.txt](LICENSE.txt).
