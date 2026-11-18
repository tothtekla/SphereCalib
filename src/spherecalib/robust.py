"""Reproducible consensus fitting and post-refinement consensus updates."""
from dataclasses import dataclass
import numpy as np
from .geometry import (unit, points_array, positive_radius, fit_sphere_image,
                       fit_sphere_oriented, fit_sphere_two_oriented,
                       angular_residual, refine_image)


@dataclass
class FitResult:
    center: np.ndarray
    radius: float
    inliers: np.ndarray
    residuals: np.ndarray
    proposals: int


def confidence_iterations(inlier_probability, sample_size, confidence=0.999):
    if not 0 < inlier_probability <= 1 or not 0 < confidence < 1 or sample_size < 1:
        raise ValueError("Invalid confidence parameters")
    if inlier_probability == 1:
        return 1
    return int(np.ceil(np.log1p(-confidence) / np.log1p(-inlier_probability**sample_size)))


def fit_image(rays, radius, normals=None, method="3p", threshold=0.002,
              iterations=1000, seed=0, refinement="points", sigma_orientation=0.05):
    if method not in ("3p", "1p5", "2op") or refinement not in ("none", "points", "oriented"):
        raise ValueError("Unknown solver or refinement")
    if threshold <= 0 or iterations < 1:
        raise ValueError("Positive threshold and iterations required")
    r = positive_radius(radius)
    U = unit(points_array(rays, 3, 3))
    if normals is not None:
        normals = unit(points_array(normals, 3, len(U)))
        if normals.shape != U.shape:
            raise ValueError("Normals must match rays")
    if (method in ("1p5", "2op") or refinement == "oriented") and normals is None:
        raise ValueError("Oriented fitting requires tangent normals")
    rng = np.random.default_rng(seed)
    best = None
    score_best = (-1, -np.inf)
    for _ in range(iterations):
        ids = rng.choice(len(U), 2 if method in ("1p5", "2op") else 3, replace=False)
        try:
            if method == "3p":
                s = fit_sphere_image(U[ids], r)
            elif method == "1p5":
                s = fit_sphere_oriented(U[ids[0]], U[ids[1]], normals[ids[0]], r)
            else:
                s = fit_sphere_two_oriented(U[ids[0]], U[ids[1]],
                                            normals[ids[0]], normals[ids[1]], r)
            e = np.abs(angular_residual(s, U, r))
            mask = e < threshold
            score = (int(mask.sum()), -float(np.minimum(e, threshold).sum()))
            if score > score_best:
                best, score_best = (s, mask), score
        except (ValueError, np.linalg.LinAlgError):
            continue
    if best is None or best[1].sum() < 3:
        raise ValueError("No sphere consensus found")
    s, mask = best
    if refinement != "none":
        try:
            refined = refine_image(s, U[mask], r,
                                   normals[mask] if refinement == "oriented" else None,
                                   threshold / 2, sigma_orientation)
            s = refined
        except (ValueError, np.linalg.LinAlgError):
            pass  # Keep the valid consensus hypothesis if refinement fails.
    e = angular_residual(s, U, r)
    mask = np.abs(e) < threshold
    if mask.sum() < 3:
        raise ValueError("Refinement lost sphere consensus")
    return FitResult(s, r, mask, e, iterations)
