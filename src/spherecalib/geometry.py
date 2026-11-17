"""Calibrated sphere silhouette geometry. Legacy equal-cone fit follows Toth et al."""
import numpy as np
from scipy.optimize import least_squares


def points_array(points, dim, minimum=1):
    a = np.asarray(points, dtype=float)
    if a.ndim != 2 or a.shape[1] != dim or len(a) < minimum or not np.isfinite(a).all():
        raise ValueError(f"Expected at least {minimum} finite {dim}D points")
    return a


def unit(a):
    a = np.asarray(a, dtype=float)
    lengths = np.linalg.norm(a, axis=-1, keepdims=True)
    if not np.isfinite(a).all() or np.any(lengths < 1e-14):
        raise ValueError("Cannot normalize zero or nonfinite vector")
    return a / lengths


def positive_radius(radius):
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Radius must be finite and positive")
    return float(radius)


def camera_matrix(K):
    K = np.asarray(K, dtype=float)
    if K.shape != (3, 3) or not np.isfinite(K).all() or abs(np.linalg.det(K)) < 1e-14:
        raise ValueError("K must be finite, invertible and 3 by 3")
    if not np.allclose(K[2], [0, 0, 1]) or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("Expected positive focal lengths and standard pinhole K")
    return K


def pixel_rays(xy, K):
    xy = points_array(xy, 2)
    return unit(np.linalg.solve(camera_matrix(K), np.c_[xy, np.ones(len(xy))].T).T)


def tangent_normals(xy, tangents, K):
    xy = points_array(xy, 2)
    tangents = points_array(tangents, 2)
    if xy.shape != tangents.shape:
        raise ValueError("One tangent is required per pixel")
    lines = np.cross(np.c_[xy, np.ones(len(xy))], np.c_[unit(tangents), np.zeros(len(xy))])
    return unit(lines @ camera_matrix(K))


def valid_center(center, radius, rays=None):
    s = np.asarray(center, dtype=float)
    return (s.shape == (3,) and np.isfinite(s).all() and s[2] > radius
            and (rays is None or np.all(np.asarray(rays) @ s > 0)))


def fit_sphere_image(rays, radius):
    """MATLAB fitSphere3p equal-cone equation, including N>3 least squares.

    New API rejects rank deficiency and spheres crossing the camera plane.
    """
    r = positive_radius(radius)
    U = unit(points_array(rays, 3, 3))
    a, _, rank, _ = np.linalg.lstsq(U, np.ones(len(U)), rcond=None)
    delta = a @ a - 1
    if rank < 3 or delta <= 1e-14:
        raise ValueError("Degenerate silhouette rays")
    s = r * a / np.sqrt(delta)
    if not valid_center(s, r, U):
        raise ValueError("Sphere is not fully in front of camera")
    return s


def fit_sphere_llsb(rays, radius):
    """Sun et al. 2016 linear least-squares sphere-center baseline.

    This implements the LLSB formulation reproduced in the project's MATLAB
    research branch. Inputs are calibrated rays; they are converted to the
    normalized image coordinates expected by the published formulation.
    """
    r = positive_radius(radius)
    U = unit(points_array(rays, 3, 3))
    if np.any(U[:, 2] <= 1e-14):
        raise ValueError("LLSB requires forward perspective rays")
    xy = U[:, :2] / U[:, 2, None]
    A = np.c_[xy, -np.sqrt(np.sum(xy*xy, axis=1) + 1)]
    parameters, _, rank, _ = np.linalg.lstsq(A, -np.ones(len(A)), rcond=None)
    denominator2 = 1 + parameters[0]**2 + parameters[1]**2 - parameters[2]**2
    if rank < 3 or denominator2 <= 1e-14:
        raise ValueError("Degenerate LLSB sphere solution")
    center = r * np.array([parameters[0], parameters[1], 1.0]) / np.sqrt(denominator2)
    if not valid_center(center, r, U):
        raise ValueError("LLSB sphere is not fully in front of camera")
    return center


def fit_sphere_oriented(u, v, normal, radius):
    """One oriented contour ray plus one ordinary ray (three scalar constraints).

    Unit u and normal n give s=lambda*u+sigma*r*n. Squaring second-ray
    tangency yields lambda=(sigma*r*n.v)/(+/-1-u.v). The negative root
    cannot have both positive contact depths u.s and v.s. Choose n's sign
    so n.v>0; reject small separation and invalid forward contacts.
    """
    r = positive_radius(radius)
    u, v = unit(u), unit(v)
    n = np.asarray(normal, float)
    n = unit(n - (n @ u) * u)
    denominator = 0.5 * np.sum((u - v)**2)
    if denominator < 1e-12:
        raise ValueError("Oriented sample has coincident or poorly separated rays")
    if n @ v < 0:
        n = -n
    center = r * (n @ v) / denominator * u + r * n
    if not valid_center(center, r, np.array([u, v])):
        raise ValueError("No valid forward sphere")
    return center


def angular_residual(center, rays, radius):
    """Signed ray-to-silhouette angular error in radians, independent of range."""
    r = positive_radius(radius)
    s = np.asarray(center, float)
    distance = np.linalg.norm(s)
    if distance <= r:
        raise ValueError("Camera must be outside sphere")
    return np.arccos(np.clip(unit(rays) @ (s / distance), -1, 1)) - np.arcsin(r / distance)


def orientation_residual(center, rays, normals):
    U = unit(rays)
    predicted = unit(center - (U @ center)[:, None] * U)
    observed = unit(normals - np.sum(normals * U, axis=1)[:, None] * U)
    # Unsigned angle between tangent planes; normal sign is unobservable.
    return np.arctan2(np.sum(predicted * np.cross(U, observed), axis=1),
                      np.abs(np.sum(predicted * observed, axis=1)))


def refine_image(center, rays, radius, normals=None, sigma_angle=0.001,
                 sigma_orientation=0.05):
    r = positive_radius(radius)
    if sigma_angle <= 0 or sigma_orientation <= 0:
        raise ValueError("Residual scales must be positive")
    U = unit(points_array(rays, 3, 3))
    def residual(s):
        e = angular_residual(s, U, r) / sigma_angle
        if normals is not None:
            e = np.r_[e, orientation_residual(s, U, normals) / sigma_orientation]
        return e
    out = least_squares(residual, center, bounds=([-np.inf, -np.inf, r + 1e-10], np.inf),
                        loss="soft_l1", max_nfev=200)
    if not out.success or not valid_center(out.x, r, U):
        raise ValueError("Image refinement did not converge")
    return out.x


def sphere_conic(center, radius):
    """Symmetric normalized-image conic matrix, x_h.T @ Q @ x_h = 0."""
    s = np.asarray(center, float)
    r = positive_radius(radius)
    return (s @ s - r*r) * np.eye(3) - np.outer(s, s)


def project(points, K):
    points = points_array(points, 3)
    if np.any(points[:, 2] <= 0):
        raise ValueError("Projection needs positive depth")
    h = points @ camera_matrix(K).T
    return h[:, :2] / h[:, 2, None]
