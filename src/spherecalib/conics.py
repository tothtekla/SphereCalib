"""Sphere/ellipse conversion and the legacy circle-consensus image pathway.

Ellipse convention: [semi-major, semi-minor, center_x, center_y, angle_rad].
Implicit coefficients: [A,B,C,D,E,F], A*x²+B*x*y+C*y²+D*x+E*y+F=0.
The direct ellipse fit follows the Halir–Flusser reduction used by MATLAB.
"""
import numpy as np
from .geometry import points_array, positive_radius, sphere_conic, valid_center


def coefficients_to_matrix(coefficients):
    A, B, C, D, E, F = np.asarray(coefficients, float)
    return np.array([[A, B/2, D/2], [B/2, C, E/2], [D/2, E/2, F]])


def matrix_to_coefficients(Q):
    Q = np.asarray(Q, float)
    return np.array([Q[0, 0], 2*Q[0, 1], Q[1, 1], 2*Q[0, 2], 2*Q[1, 2], Q[2, 2]])


def ellipse_parameters(Q):
    Q = np.asarray(Q, float)
    if Q.shape != (3, 3) or not np.isfinite(Q).all():
        raise ValueError("Expected finite 3 by 3 conic")
    Q = (Q+Q.T)/2
    if np.trace(Q[:2, :2]) < 0:
        Q = -Q
    H, g = Q[:2, :2], Q[:2, 2]
    center = -np.linalg.solve(H, g)
    eigenvalues, vectors = np.linalg.eigh(H)
    level = center @ H @ center - Q[2, 2]
    if eigenvalues.min() <= 0 or level <= 0:
        raise ValueError("Conic is not a real ellipse")
    axes = np.sqrt(level / eigenvalues)
    theta = np.arctan2(vectors[1, 0], vectors[0, 0])
    return np.r_[axes, center, theta]


def parametric_conic(parameters):
    a, b, x, y, angle = np.asarray(parameters, float)
    if a <= 0 or b <= 0:
        raise ValueError("Ellipse axes must be positive")
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    H = R @ np.diag([1/a**2, 1/b**2]) @ R.T
    center = np.array([x, y])
    Q = np.empty((3, 3))
    Q[:2, :2], Q[:2, 2], Q[2, :2] = H, -H @ center, -center @ H
    Q[2, 2] = center @ H @ center - 1
    return Q


def ellipse_distance(points, parameters):
    """Same approximate ellipse-distance expression as legacy MATLAB."""
    X = points_array(points, 2)
    a, b, x, y, angle = parameters
    c, s = np.cos(angle), np.sin(angle)
    P = np.abs((X-[x, y]) @ np.array([[c, -s], [s, c]]))
    k0 = np.linalg.norm(P / [a, b], axis=1)
    k1 = np.linalg.norm(P / [a*a, b*b], axis=1)
    return np.divide(np.abs(k0*(k0-1)), k1, out=np.full(len(X), min(a, b)), where=k1 > 1e-15)


def fit_circle(points):
    X = points_array(points, 2, 3)
    offset = X.mean(axis=0)
    Y = X-offset
    a, _, rank, _ = np.linalg.lstsq(np.c_[2*Y, np.ones(len(Y))], np.sum(Y*Y, axis=1), rcond=None)
    r2 = a[:2] @ a[:2] + a[2]
    if rank < 3 or r2 <= 0:
        raise ValueError("Degenerate circle sample")
    return a[:2]+offset, float(np.sqrt(r2))


def fit_ellipse(points):
    X = points_array(points, 2, 5)
    offset = X.mean(axis=0)
    x, y = (X-offset).T
    D1 = np.c_[x*x, x*y, y*y]
    D2 = np.c_[x, y, np.ones(len(X))]
    S1, S2, S3 = D1.T @ D1, D1.T @ D2, D2.T @ D2
    T = -np.linalg.solve(S3, S2.T)
    M = S1+S2 @ T
    M = np.array([M[2]/2, -M[1], M[0]/2])
    _, vectors = np.linalg.eig(M)
    for vector in vectors.T:
        if np.max(np.abs(vector.imag)) > 1e-8:
            continue
        a = vector.real
        if 4*a[0]*a[2]-a[1]**2 <= 0:
            continue
        Q = coefficients_to_matrix(np.r_[a, T @ a])
        shift = np.eye(3)
        shift[:2, 2] = -offset
        Q = shift.T @ Q @ shift
        ellipse_parameters(Q)  # Reject imaginary or singular ellipses.
        return Q / np.linalg.norm(Q)
    raise ValueError("No valid fitted ellipse")


def conic_to_sphere(Q, radius):
    """Project a measured ellipse to a sphere conic through its eigenstructure.

    Exact sphere conics have repeated positive eigenvalues and one negative
    eigenvalue. For a measured general ellipse, averaging the two positive
    eigenvalues is the symmetric least-squares projection onto that spectrum.
    Coefficient-space nonlinear refinement is deliberately avoided because its
    scale ambiguity makes it poorly conditioned for noisy, near-circular conics.
    """
    r = positive_radius(radius)
    Q = np.asarray(Q, float)
    ellipse_parameters(Q)
    if np.trace(Q[:2, :2]) < 0:
        Q = -Q
    values, vectors = np.linalg.eigh(Q)
    if values[0] >= 0 or values[1] <= 0:
        raise ValueError("Invalid projected sphere conic")
    scale = -r*r/values[0]
    distance = np.sqrt(scale*np.mean(values[1:])+r*r)
    initial = vectors[:, 0]*distance
    if initial[2] < 0:
        initial = -initial
    if not valid_center(initial, r):
        raise ValueError("Ellipse does not yield a forward sphere")
    return initial


def fit_circle_consensus(points, radius, threshold=0.004, iterations=1000,
                         radius_bounds=(0.001, 2.0), seed=0):
    from .robust import FitResult
    X = points_array(points, 2, 5)
    positive_radius(radius)
    if threshold <= 0 or iterations < 1:
        raise ValueError("Invalid consensus configuration")
    rng = np.random.default_rng(seed)
    best = np.zeros(len(X), bool)
    for _ in range(iterations):
        try:
            c, r = fit_circle(X[rng.choice(len(X), 3, replace=False)])
            if not radius_bounds[0] <= r <= radius_bounds[1]:
                continue
            mask = np.abs(np.linalg.norm(X-c, axis=1)-r) < threshold
            if mask.sum() > best.sum():
                best = mask
        except (ValueError, np.linalg.LinAlgError):
            continue
    if best.sum() < 5:
        raise ValueError("No circle consensus")
    Q = fit_ellipse(X[best])
    center = conic_to_sphere(Q, radius)
    residuals = ellipse_distance(X, ellipse_parameters(sphere_conic(center, radius)))
    return FitResult(center, radius, residuals < threshold, residuals, iterations)
