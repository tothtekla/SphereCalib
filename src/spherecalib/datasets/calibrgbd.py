"""CalibRGBD 2019 adapter and processed-measurement evaluation.

The official synthetic files use millimetres. Public API values are converted
to metres except benchmark output fields explicitly suffixed `_mm`/`_deg`.
The processed ellipse/sphere files are measurements, while GroundTruth/scene.yml
contains exact constructed values.
"""
from dataclasses import dataclass
import csv
import json
from pathlib import Path
from time import perf_counter
import cv2
import numpy as np

from ..conics import parametric_conic, conic_to_sphere
from ..geometry import (fit_sphere_image, fit_sphere_llsb, fit_sphere_oriented,
                        pixel_rays, tangent_normals, refine_image)
from ..registration import Transform, register_points, transform_error


OFFICIAL_URLS = {
    "synthetic": "https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/data/Synth.zip",
    "random": "https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/data/Random.zip",
    "equidistant": "https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/data/Equidistant.zip",
    "double": "https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/data/Double.zip",
}


def _node(path, name):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f"Cannot open OpenCV YAML: {path}")
    node = storage.getNode(name)
    if node.empty():
        storage.release()
        raise ValueError(f"Missing {name} in {path}")
    value = node.mat() if node.isMap() else node.real()
    storage.release()
    return value


def _scalars(path, names):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f"Cannot open OpenCV YAML: {path}")
    values = {}
    for name in names:
        node = storage.getNode(name)
        if node.empty():
            storage.release()
            raise ValueError(f"Missing {name} in {path}")
        values[name] = float(node.real())
    storage.release()
    return values


def locate(root):
    """Accept the extracted directory, its parent, or the archive sibling."""
    root = Path(root)
    candidates = [root, root / "Synth"]
    for candidate in candidates:
        if (candidate / "GroundTruth" / "scene.yml").exists() and (candidate / "data").is_dir():
            return candidate
    raise ValueError(f"Not a CalibRGBD synthetic root: {root}")


def natural_image_paths(folder, suffix=".yml"):
    paths = list(Path(folder).glob(f"img*{suffix}"))
    try:
        return sorted(paths, key=lambda p: int(p.stem[3:]))
    except ValueError as error:
        raise ValueError(f"Unexpected CalibRGBD frame name in {folder}") from error


@dataclass(frozen=True)
class SyntheticTruth:
    K: np.ndarray
    radius_m: float
    depth_centers_m: np.ndarray
    rgb_from_depth: Transform

    @property
    def rgb_centers_m(self):
        return self.rgb_from_depth.apply(self.depth_centers_m)


def load_truth(root):
    root = locate(root)
    scene = root / "GroundTruth" / "scene.yml"
    K = np.asarray(_node(scene, "rgbIntrinsic"), float)
    radius = float(_node(scene, "sphereRadius")) / 1000
    centers = np.asarray(_node(scene, "SphereCenters"), float).T / 1000
    translation = np.asarray(_node(scene, "translation"), float).ravel() / 1000
    rotation_degree = np.asarray(_node(scene, "rotation_degree"), float).ravel()
    # Dataset synthetic GT declares zero rotation. Refuse to guess its Euler convention.
    if not np.allclose(rotation_degree, 0):
        raise ValueError("Nonzero CalibRGBD GT rotation convention needs explicit handling")
    return SyntheticTruth(K, radius, centers, Transform(np.eye(3), translation))


def load_ellipse(path):
    values = _scalars(path, ("x", "y", "majorAxis", "minorAxis", "angle"))
    # cv::RotatedRect stores full axis lengths. Normalize ordering while retaining angle.
    a, b = values["majorAxis"] / 2, values["minorAxis"] / 2
    angle = np.deg2rad(values["angle"])
    if b > a:
        a, b, angle = b, a, angle + np.pi/2
    return np.array([a, b, values["x"], values["y"], angle])


def load_sphere(path):
    values = _scalars(path, ("x", "y", "z", "Radius", "rms"))
    return np.array([values["x"], values["y"], values["z"]]) / 1000, values["Radius"] / 1000


def ellipse_samples(parameters, count=100):
    a, b, x, y, angle = np.asarray(parameters, float)
    phase = np.linspace(0, 2*np.pi, count, endpoint=False)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    points = np.c_[a*np.cos(phase), b*np.sin(phase)] @ rotation.T + [x, y]
    derivatives = np.c_[-a*np.sin(phase), b*np.cos(phase)] @ rotation.T
    derivatives /= np.linalg.norm(derivatives, axis=1, keepdims=True)
    return points, derivatives


def estimate_rgb_center(ellipse, K, radius_m, method, sample_count=100):
    pixels, tangents = ellipse_samples(ellipse, sample_count)
    rays = pixel_rays(pixels, K)
    if method == "direct3p":
        return fit_sphere_image(rays, radius_m)
    if method == "direct3p-minimal":
        return fit_sphere_image(rays[[0, sample_count//3, 2*sample_count//3]], radius_m)
    if method == "llsb":
        return fit_sphere_llsb(rays, radius_m)
    if method == "llsb-minimal":
        return fit_sphere_llsb(rays[[0, sample_count//3, 2*sample_count//3]], radius_m)
    if method == "ellipse-conic":
        homogeneous_transform = np.asarray(K, float)
        Q_pixel = parametric_conic(ellipse)
        Q_normalized = homogeneous_transform.T @ Q_pixel @ homogeneous_transform
        return conic_to_sphere(Q_normalized, radius_m)
    if method in ("1p5-minimal", "1p5-refined"):
        normals = tangent_normals(pixels, tangents, K)
        initial = fit_sphere_oriented(rays[0], rays[sample_count//3], normals[0], radius_m)
        return initial if method == "1p5-minimal" else refine_image(initial, rays, radius_m)
    raise ValueError(f"Unknown image center method: {method}")


def calibration_directories(root, noise=None, limit=None):
    root = locate(root)
    groups = [root / "data" / noise] if noise else sorted((root / "data").glob("NoiseDepth*"),
                                                            key=lambda p: float(p.name[10:]))
    for group in groups:
        if not group.is_dir():
            raise ValueError(f"Missing noise group: {group}")
        calibrations = sorted(group.glob("cal*"), key=lambda p: int(p.name[3:]))
        for path in calibrations[:limit]:
            yield group.name, path


def evaluate_processed(root, output, methods=None, noise=None, limit=None):
    """Evaluate center solvers and rigid calibration on official measurements."""
    root = locate(root)
    truth = load_truth(root)
    methods = methods or ("direct3p", "direct3p-minimal", "llsb", "llsb-minimal",
                          "ellipse-conic", "1p5-minimal", "1p5-refined")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for noise_name, calibration in calibration_directories(root, noise, limit):
        ellipse_paths = natural_image_paths(calibration / "Processed" / "Ellipse")
        sphere_paths = natural_image_paths(calibration / "Processed" / "Sphere")
        if len(ellipse_paths) != len(truth.depth_centers_m) or len(sphere_paths) != len(ellipse_paths):
            raise ValueError(f"Unexpected observation count in {calibration}")
        depth_centers = np.array([load_sphere(path)[0] for path in sphere_paths])
        ellipses = [load_ellipse(path) for path in ellipse_paths]
        for method in methods:
            start = perf_counter()
            failure = ""
            try:
                rgb_centers = np.array([estimate_rgb_center(e, truth.K, truth.radius_m, method)
                                        for e in ellipses])
                transform = register_points(depth_centers, rgb_centers)
                errors = transform_error(transform, truth.rgb_from_depth)
                center_errors_mm = np.linalg.norm(rgb_centers-truth.rgb_centers_m, axis=1)*1000
                depth_errors_mm = np.linalg.norm(depth_centers-truth.depth_centers_m, axis=1)*1000
                row = dict(noise=noise_name, calibration=calibration.name, method=method,
                           rotation_error_deg=np.rad2deg(errors["rotation_rad"]),
                           translation_error_mm=errors["translation_m"]*1000,
                           image_center_rmse_mm=np.sqrt(np.mean(center_errors_mm**2)),
                           image_center_median_mm=np.median(center_errors_mm),
                           depth_center_rmse_mm=np.sqrt(np.mean(depth_errors_mm**2)),
                           elapsed_s=perf_counter()-start, failure=failure)
            except (ValueError, np.linalg.LinAlgError) as error:
                row = dict(noise=noise_name, calibration=calibration.name, method=method,
                           rotation_error_deg="", translation_error_mm="",
                           image_center_rmse_mm="", image_center_median_mm="",
                           depth_center_rmse_mm="", elapsed_s=perf_counter()-start,
                           failure=str(error))
            rows.append(row)
    if not rows:
        raise ValueError("No CalibRGBD calibration sets found")
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    output.with_name(output.stem+"_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return {"rows": len(rows), "failures": sum(bool(row["failure"]) for row in rows),
            "noise_groups": sorted({row["noise"] for row in rows}), "methods": list(methods),
            "output": str(output), "summary": str(output.with_name(output.stem+"_summary.json"))}


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["noise"], row["method"]), []).append(row)
    result = []
    for (noise, method), records in sorted(groups.items(), key=lambda item: (float(item[0][0][10:]), item[0][1])):
        valid = [record for record in records if not record["failure"]]
        item = {"noise": noise, "method": method, "runs": len(records),
                "failures": len(records)-len(valid)}
        for field in ("rotation_error_deg", "translation_error_mm", "image_center_rmse_mm",
                      "image_center_median_mm", "depth_center_rmse_mm", "elapsed_s"):
            values = np.array([float(record[field]) for record in valid])
            item[field+"_median"] = float(np.median(values)) if len(values) else None
            item[field+"_mean"] = float(np.mean(values)) if len(values) else None
        result.append(item)
    return result


def official_baseline_summary(root):
    """Read benchmark-provided aggregate results without relabelling them as ours."""
    root = locate(root)
    rows = []
    for algorithm_dir in sorted((root / "results").iterdir()):
        if not algorithm_dir.is_dir():
            continue
        for noise_dir in sorted(algorithm_dir.iterdir(), key=lambda p: float(p.name[10:])):
            path = noise_dir / "CalibrationResults.csv"
            if not path.exists():
                continue
            with path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream, delimiter=";")
                mean = next(reader)
            rows.append({"algorithm": algorithm_dir.name, "noise": noise_dir.name,
                         "source": "official benchmark-provided aggregate",
                         "rotation_error_deg_mean": float(mean["R_err"]),
                         "translation_error_mm_mean": float(mean["t_err"]),
                         "reprojection_error_mean": float(mean["e_r"]),
                         "point_error_mean": float(mean["E_r"])})
    return rows


def evaluate_official_per_calibration(root, noise=None, limit=None):
    """Recompute GT transform errors from benchmark-provided Calibration.yml.

    Synthetic GT rotation is zero, so the norm of the dataset's three reported
    degree components is its rotation error (matching its supplied CSV metric).
    """
    root = locate(root)
    truth = load_truth(root)
    rows = []
    for noise_name, calibration in calibration_directories(root, noise, limit):
        output = calibration / "Processed" / "OUTPUT"
        for algorithm_dir in sorted(output.iterdir()):
            path = algorithm_dir / "Calibration.yml"
            if not path.exists():
                continue
            translation = np.asarray(_node(path, "translation"), float).ravel() / 1000
            rotation_degree = np.asarray(_node(path, "rotation_degree"), float).ravel()
            rows.append({"noise": noise_name, "calibration": calibration.name,
                         "method": algorithm_dir.name,
                         "source": "official benchmark-provided calibration",
                         "rotation_error_deg": float(np.linalg.norm(rotation_degree)),
                         "translation_error_mm": float(np.linalg.norm(translation-
                                                        truth.rgb_from_depth.translation)*1000)})
    return rows
