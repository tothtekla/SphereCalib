"""SHREC 2022 sphere-only test adapter and fitting evaluation."""
import csv
import json
from pathlib import Path
from time import perf_counter
import numpy as np

from ..lidar import algebraic_sphere, refine_cloud, fit_cloud


def locate(root):
    root = Path(root)
    candidates = [root, root / "dataset" / "test"]
    for candidate in candidates:
        if (candidate / "GTpointCloud").is_dir() and (candidate / "pointCloud").is_dir():
            return candidate
    raise ValueError(f"Not a SHREC 2022 test root: {root}")


def load_ground_truth(path):
    values = np.loadtxt(path).reshape(-1)
    if len(values) != 5 or int(values[0]) != 3:
        raise ValueError(f"Expected SHREC sphere ground truth: {path}")
    return float(values[1]), values[2:5]


def sphere_cases(root):
    root = locate(root)
    for gt in sorted((root / "GTpointCloud").glob("GTpointCloud*.txt"),
                     key=lambda path: int(path.stem[12:])):
        values = np.loadtxt(gt).reshape(-1)
        if len(values) and int(values[0]) == 3:
            identifier = int(gt.stem[12:])
            cloud = root / "pointCloud" / f"pointCloud{identifier}.txt"
            if not cloud.exists():
                raise ValueError(f"Missing sparse SHREC sphere cloud: {cloud}")
            yield identifier, cloud, float(values[1]), values[2:5]


def _fit(points, radius, method, seed):
    initial_center, initial_radius = algebraic_sphere(points)
    if method == "algebraic":
        return initial_center, initial_radius
    scale = max(radius * 0.02, 1e-6)
    if method == "geometric-unknown":
        return refine_cloud(points, initial_center, initial_radius, False, scale)
    if method == "geometric-known":
        return refine_cloud(points, initial_center, radius, True, scale)
    known = radius if method.endswith("-known") else None
    sampling = method.split("-")[1]
    result = fit_cloud(points, known, sampling, threshold=max(radius*0.05, 1e-6),
                       iterations=300, neighbors=min(50, len(points)),
                       neighborhood_radius=radius*0.75,
                       radius_bounds=(radius*0.25, radius*4), seed=seed)
    return result.center, result.radius


def evaluate(root, output, methods=None, limit=None, seed=0):
    methods = methods or ("algebraic", "geometric-unknown", "geometric-known",
                          "ransac-random-unknown", "ransac-random-known",
                          "ransac-adjacency-unknown", "ransac-distance-unknown")
    cases = list(sphere_cases(root))[:limit]
    if not cases:
        raise ValueError("No SHREC sphere cases found")
    rows = []
    for identifier, path, radius, center in cases:
        points = np.loadtxt(path, delimiter=",")
        for method in methods:
            start = perf_counter()
            try:
                estimate, estimate_radius = _fit(points, radius, method, seed+identifier)
                rows.append({"case": identifier, "method": method, "points": len(points),
                             "center_error": float(np.linalg.norm(estimate-center)),
                             "center_error_over_radius": float(np.linalg.norm(estimate-center)/radius),
                             "radius_error": abs(float(estimate_radius-radius)),
                             "elapsed_s": perf_counter()-start, "failure": ""})
            except (ValueError, np.linalg.LinAlgError) as error:
                rows.append({"case": identifier, "method": method, "points": len(points),
                             "center_error": "", "center_error_over_radius": "",
                             "radius_error": "", "elapsed_s": perf_counter()-start,
                             "failure": str(error)})
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for method in methods:
        selected = [row for row in rows if row["method"] == method]
        valid = [row for row in selected if not row["failure"]]
        item = {"method": method, "cases": len(selected), "failures": len(selected)-len(valid)}
        for field in ("center_error", "center_error_over_radius", "radius_error", "elapsed_s"):
            values = np.asarray([float(row[field]) for row in valid])
            item[field+"_median"] = float(np.median(values)) if len(values) else None
            item[field+"_mean"] = float(np.mean(values)) if len(values) else None
            item[field+"_q95"] = float(np.quantile(values, .95)) if len(values) else None
        summary.append(item)
    summary_path = output.with_name(output.stem+"_summary.json")
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return {"cases": len(cases), "rows": len(rows),
            "failures": sum(bool(row["failure"]) for row in rows),
            "methods": list(methods), "output": str(output), "summary": str(summary_path)}
