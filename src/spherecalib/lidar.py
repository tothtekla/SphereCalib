"""Cloud sphere and plane fitting, with adjacency/distance/random hypotheses."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from .geometry import points_array, positive_radius
from .robust import FitResult


def algebraic_sphere(points):
    X = points_array(points, 3, 4)
    offset = X.mean(axis=0)
    Y = X - offset
    a, _, rank, _ = np.linalg.lstsq(np.c_[2*Y, np.ones(len(Y))], np.sum(Y*Y, axis=1), rcond=None)
    radius2 = a[:3] @ a[:3] + a[3]
    if rank < 4 or radius2 <= 0:
        raise ValueError("Degenerate sphere sample")
    return a[:3] + offset, float(np.sqrt(radius2))


def refine_cloud(points, center, radius, known_radius=False, scale=0.01):
    X = points_array(points, 3, 4)
    positive_radius(radius)
    def residual(a):
        return (np.linalg.norm(X-a[:3], axis=1) - (radius if known_radius else a[3])) / scale
    initial = center if known_radius else np.r_[center, radius]
    bounds = (-np.inf, np.inf) if known_radius else ([-np.inf]*3 + [1e-10], np.inf)
    out = least_squares(residual, initial, bounds=bounds, loss="soft_l1", max_nfev=300)
    if not out.success:
        raise ValueError("Cloud refinement did not converge")
    return out.x[:3], float(radius if known_radius else out.x[3])


def fit_cloud(points, radius=None, method="adjacency", threshold=0.02,
              iterations=1000, neighbors=30, neighborhood_radius=0.15,
              radius_bounds=(0.01, 10.0), seed=0):
    X = points_array(points, 3, 4)
    if method not in ("adjacency", "distance", "random"):
        raise ValueError("Unknown cloud method")
    if threshold <= 0 or iterations < 1 or neighbors < 4 or neighborhood_radius <= 0:
        raise ValueError("Invalid cloud fitting configuration")
    lo, hi = radius_bounds
    if not 0 < lo < hi:
        raise ValueError("Invalid radius bounds")
    if radius is not None:
        positive_radius(radius)
        if not lo <= radius <= hi:
            raise ValueError("Known radius outside radius bounds")
    tree = cKDTree(X)
    rng = np.random.default_rng(seed)
    best, best_score = None, (-1, -np.inf)
    for _ in range(iterations):
        if method == "random":
            ids = rng.choice(len(X), 4, replace=False)
        elif method == "adjacency":
            _, ids = tree.query(X[rng.integers(len(X))], k=min(neighbors, len(X)))
        else:
            ids = tree.query_ball_point(X[rng.integers(len(X))], neighborhood_radius)
        try:
            c, r = algebraic_sphere(X[ids])
            if radius is not None:
                c, r = refine_cloud(X[ids], c, radius, True, threshold)
            if not lo <= r <= hi:
                continue
            e = np.abs(np.linalg.norm(X-c, axis=1)-r)
            mask = e < threshold
            score = (int(mask.sum()), -float(np.minimum(e, threshold).sum()))
            if score > best_score:
                best, best_score = (c, r, mask), score
        except (ValueError, np.linalg.LinAlgError):
            continue
    if best is None or best[2].sum() < 4:
        raise ValueError("No cloud sphere consensus found")
    c, r, mask = best
    c, r = refine_cloud(X[mask], c, r, radius is not None, threshold)
    if not lo <= r <= hi:
        raise ValueError("Refined radius outside bounds")
    e = np.linalg.norm(X-c, axis=1)-r
    mask = np.abs(e) < threshold
    if mask.sum() < 4:
        raise ValueError("Refinement lost cloud consensus")
    return FitResult(c, r, mask, e, iterations)


def remove_planes(points, count=0, close_threshold=0.0, threshold=0.04,
                  iterations=500, min_inliers=30, seed=0):
    X = points_array(points, 3, 4)
    if count < 0 or threshold <= 0 or iterations < 1 or min_inliers < 3:
        raise ValueError("Invalid plane removal configuration")
    X = X[np.linalg.norm(X, axis=1) >= close_threshold]
    rng = np.random.default_rng(seed)
    for _ in range(count):
        if len(X) < max(4, min_inliers):
            break
        best = np.zeros(len(X), bool)
        for _ in range(iterations):
            p = X[rng.choice(len(X), 3, replace=False)]
            n = np.cross(p[1]-p[0], p[2]-p[0])
            length = np.linalg.norm(n)
            if length < 1e-12:
                continue
            mask = np.abs((X-p[0]) @ (n/length)) < threshold
            if mask.sum() > best.sum():
                best = mask
        if best.sum() < min_inliers:
            break
        X = X[~best]
    return X
