"""Proper SE(3) registration. Output always maps LiDAR to camera in pipeline."""
from dataclasses import dataclass
import numpy as np
from .geometry import points_array


@dataclass(frozen=True)
class Transform:
    rotation: np.ndarray
    translation: np.ndarray

    def apply(self, points):
        return np.asarray(points) @ self.rotation.T + self.translation

    def inverse(self):
        return Transform(self.rotation.T, -self.rotation.T @ self.translation)

    def matrix(self):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.rotation, self.translation
        return T


def register_points(source, target):
    X, Y = points_array(source, 3, 3), points_array(target, 3, 3)
    if X.shape != Y.shape:
        raise ValueError("Corresponding point sets must match")
    xc, yc = X.mean(axis=0), Y.mean(axis=0)
    A, B = X-xc, Y-yc
    if np.linalg.matrix_rank(A) < 2 or np.linalg.matrix_rank(B) < 2:
        raise ValueError("Registration requires noncollinear centers")
    U, _, Vt = np.linalg.svd(A.T @ B)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ correction @ U.T
    return Transform(R, yc - R @ xc)


def robust_register(source, target, threshold=0.03, iterations=500, seed=0):
    X, Y = points_array(source, 3, 3), points_array(target, 3, 3)
    if X.shape != Y.shape or threshold <= 0 or iterations < 1:
        raise ValueError("Invalid registration inputs")
    rng = np.random.default_rng(seed)
    best = np.zeros(len(X), bool)
    for _ in range(iterations):
        ids = rng.choice(len(X), 3, replace=False)
        try:
            T = register_points(X[ids], Y[ids])
        except ValueError:
            continue
        mask = np.linalg.norm(T.apply(X)-Y, axis=1) < threshold
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 3:
        raise ValueError("No registration consensus")
    T = register_points(X[best], Y[best])
    mask = np.linalg.norm(T.apply(X)-Y, axis=1) < threshold
    if mask.sum() < 3:
        raise ValueError("Refinement lost registration consensus")
    return T, mask


def transform_error(estimate, truth):
    angle = np.arccos(np.clip((np.trace(truth.rotation.T @ estimate.rotation)-1)/2, -1, 1))
    return {"rotation_rad": float(angle),
            "translation_m": float(np.linalg.norm(estimate.translation-truth.translation))}
