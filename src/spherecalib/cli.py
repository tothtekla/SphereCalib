"""python -m spherecalib: synthetic checks, benchmarks and manifest calibration."""
import argparse
import csv
import json
import platform
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
import numpy as np
import scipy
from .pipeline import CalibrationConfig, calibrate, calibrate_observations
from .synthetic import paired_scene, silhouette
from .geometry import pixel_rays, tangent_normals
from .registration import transform_error
from .robust import fit_image, confidence_iterations


def benchmark(output, trials=30, iterations=300, seed=0, budget="equal-proposals",
              confidence=0.999, inlier_probability=0.75, orientation_reliability=0.95):
    """Paired trials with either equal proposals or explicit success confidence."""
    if trials < 1 or iterations < 1:
        raise ValueError("Positive trial and iteration counts required")
    if budget not in ("equal-proposals", "equal-confidence"):
        raise ValueError("Unknown RANSAC budget mode")
    if not 0 < confidence < 1 or not 0 < inlier_probability <= 1:
        raise ValueError("Confidence and inlier probability must be in (0, 1]")
    if not 0 < orientation_reliability <= 1:
        raise ValueError("Orientation reliability must be in (0, 1]")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    truth, radius = np.array([0.18, -0.08, 2.8]), 0.12
    for noise in (0.0, 0.5, 1.0):
        for tangent_noise in (0.0, 3.0, 6.0):
            for trial in range(trials):
                xy, t, K, _ = silhouette(truth, radius, noise_px=noise,
                                         tangent_noise_deg=tangent_noise,
                                         outlier_fraction=0.25, seed=seed+trial)
                U, N = pixel_rays(xy, K), tangent_normals(xy, t, K)
                for refinement in ("none", "points", "oriented"):
                    for method in ("3p", "1p5"):
                        proposals = iterations
                        if budget == "equal-confidence":
                            if method == "3p":
                                proposals = confidence_iterations(inlier_probability, 3, confidence)
                            else:
                                success = inlier_probability**2 * orientation_reliability
                                proposals = (1 if success >= 1 else
                                             int(np.ceil(np.log1p(-confidence)/np.log1p(-success))))
                        start = perf_counter()
                        error, count, failure = None, 0, ""
                        try:
                            fit = fit_image(U, radius, N, method, threshold=0.0015,
                                            iterations=proposals, seed=seed+trial,
                                            refinement=refinement)
                            error = float(np.linalg.norm(fit.center-truth))
                            count = int(fit.inliers.sum())
                        except ValueError as exc:
                            failure = str(exc)
                        rows.append(dict(noise_px=noise, tangent_noise_deg=tangent_noise,
                                         trial=trial, seed=seed+trial, method=method,
                                         refinement=refinement, error_m=error, inliers=count,
                                         proposals=proposals, elapsed_s=perf_counter()-start,
                                         failure=failure))
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {"kind": "synthetic_solver_ablation", "budget": budget, "trials": trials,
                "iterations": iterations, "seed": seed, "outlier_fraction": 0.25,
                "confidence": confidence, "assumed_inlier_probability": inlier_probability,
                "assumed_orientation_reliability": orientation_reliability,
                "threshold_rad": 0.0015, "python": platform.python_version(),
                "numpy": np.__version__, "scipy": scipy.__version__,
                "truth_center_m": truth.tolist(), "radius_m": radius,
                "failures": sum(bool(row["failure"]) for row in rows), "rows": len(rows)}
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    from .evaluation import summarize_benchmark, paired_solver_differences
    summary = summarize_benchmark(output, seed=seed)
    output.with_name(output.stem+"_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    paired = paired_solver_differences(output, seed=seed)
    output.with_name(output.stem+"_paired.json").write_text(json.dumps(paired, indent=2), encoding="utf-8")
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("demo", help="Synthetic paired-cloud calibration")
    demo.add_argument("--seed", type=int, default=0)
    demo.add_argument("--iterations", type=int, default=200)
    demo.add_argument("--output", type=Path)
    bench = sub.add_parser("benchmark", help="Paired synthetic solver/refinement ablations")
    bench.add_argument("--output", type=Path, default=Path("results/benchmark.csv"))
    bench.add_argument("--trials", type=int, default=30)
    bench.add_argument("--iterations", type=int, default=300)
    bench.add_argument("--seed", type=int, default=0)
    bench.add_argument("--budget", choices=["equal-proposals", "equal-confidence"],
                       default="equal-proposals")
    bench.add_argument("--confidence", type=float, default=0.999)
    bench.add_argument("--inlier-probability", type=float, default=0.75)
    bench.add_argument("--orientation-reliability", type=float, default=0.95)
    calibration = sub.add_parser("calibrate", help="Calibrate paired images/clouds from JSON")
    calibration.add_argument("manifest", type=Path)
    calibration.add_argument("--output", type=Path, default=Path("results/calibration.json"))
    example = sub.add_parser("example", help="Run a bundled MATLAB-era dataset with Python")
    example.add_argument("directory", type=Path)
    example.add_argument("--camera", default="C1")
    example.add_argument("--lidar", default="L1")
    example.add_argument("--iterations", type=int, default=1000)
    example.add_argument("--planes", type=int, default=0)
    example.add_argument("--radius", type=float, help="Known/nominal radius in meters")
    example.add_argument("--radius-min", type=float, default=0.1)
    example.add_argument("--radius-max", type=float, default=1.0)
    example.add_argument("--close-threshold", type=float, default=0.0)
    example.add_argument("--image-method", choices=["3p", "1p5", "circle"], default="3p")
    example.add_argument("--output", type=Path, default=Path("results/example.json"))
    public = sub.add_parser("evaluate-calibrgbd", help="Evaluate methods on extracted CalibRGBD Synth")
    public.add_argument("root", type=Path)
    public.add_argument("--output", type=Path, default=Path("results/calibrgbd.csv"))
    public.add_argument("--noise", help="One group, e.g. NoiseDepth0")
    public.add_argument("--limit", type=int, help="Limit repetitions per noise group")
    shrec = sub.add_parser("evaluate-shrec", help="Evaluate 3D fitting on SHREC 2022 spheres")
    shrec.add_argument("root", type=Path)
    shrec.add_argument("--output", type=Path, default=Path("results/shrec2022.csv"))
    shrec.add_argument("--limit", type=int)
    shrec.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    try:
        if args.command == "demo":
            rays, normals, clouds, _, radius, truth = paired_scene(seed=args.seed)
            cfg = CalibrationConfig(iterations=args.iterations, seed=args.seed,
                                    cloud_threshold_m=0.005, image_threshold_rad=0.0005)
            result = calibrate_observations(rays, clouds, radius, normals, cfg)
            payload = result.to_dict()
            payload["synthetic_ground_truth_error"] = transform_error(result.camera_from_lidar, truth)
        elif args.command == "benchmark":
            payload = benchmark(args.output, args.trials, args.iterations, args.seed, args.budget,
                                args.confidence, args.inlier_probability,
                                args.orientation_reliability)
        elif args.command == "example":
            from .data import load_bundled_example
            images, clouds, K, metadata, ids = load_bundled_example(args.directory, args.camera, args.lidar)
            cfg = CalibrationConfig(iterations=args.iterations, plane_count=args.planes,
                                    cloud_threshold_m=0.05, image_threshold_rad=4/float(K[0, 0]),
                                    radius_bounds=(args.radius_min, args.radius_max),
                                    close_threshold_m=args.close_threshold, image_method=args.image_method)
            result = calibrate(images, clouds, K, radius=args.radius, config=cfg)
            payload = result.to_dict()
            payload["frame_ids"] = ids
            payload["pixel_origin_conversion"] = "MATLAB one-based to Python zero-based; principal point minus one"
            # Centers/radius are explicit synthetic metadata; real examples lack these.
            for key, centers in (("S0CamGT", result.camera_centers), ("S0LidGT", result.lidar_centers)):
                if key in metadata:
                    truth = np.asarray(metadata[key])[np.array(ids)-1]
                    payload[key+"_center_errors_m"] = np.linalg.norm(centers-truth, axis=1).tolist()
            if "rGT" in metadata:
                payload["radius_error_m"] = abs(result.radius-float(metadata["rGT"]))
        elif args.command == "evaluate-calibrgbd":
            from .datasets.calibrgbd import (evaluate_processed, official_baseline_summary,
                                             evaluate_official_per_calibration)
            payload = evaluate_processed(args.root, args.output, noise=args.noise, limit=args.limit)
            official = {"aggregate": official_baseline_summary(args.root),
                        "per_calibration": evaluate_official_per_calibration(args.root, args.noise, args.limit)}
            official_path = args.output.with_name(args.output.stem+"_official.json")
            official_path.write_text(json.dumps(official, indent=2), encoding="utf-8")
            payload["official_baselines"] = str(official_path)
        elif args.command == "evaluate-shrec":
            from .datasets.shrec2022 import evaluate
            payload = evaluate(args.root, args.output, limit=args.limit, seed=args.seed)
        else:
            from .data import load_manifest
            images, clouds, K, rois, metadata = load_manifest(args.manifest)
            cfg = CalibrationConfig(**metadata.get("config", {}))
            payload = calibrate(images, clouds, K, metadata.get("radius_m"), cfg,
                                rois, metadata.get("distortion")).to_dict()
        if args.command not in ("benchmark", "evaluate-calibrgbd", "evaluate-shrec") and args.output:
            payload["config"] = asdict(cfg)
            payload["versions"] = {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(json.dumps(payload, indent=2))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, f"spherecalib: {error}\n")


if __name__ == "__main__":
    main()
