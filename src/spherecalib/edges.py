"""Classical edge candidates with local quadratic subpixel localization.

Input images use OpenCV BGR ordering (or grayscale), uint8. Tangents are
perpendicular to the measured gradient; they are not guaranteed silhouettes.
Use an ROI/mask to exclude texture, highlights and background edges.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from .geometry import camera_matrix


@dataclass
class EdgeCandidates:
    pixels: np.ndarray
    tangents: np.ndarray
    confidence: np.ndarray


def extract_edges(image, low=50, high=150, method="canny", roi=None, mask=None,
                  blur_sigma=1.0, max_points=4000):
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim not in (2, 3):
        raise ValueError("Expected uint8 grayscale or BGR image")
    if method not in ("canny", "sobel") or not 0 <= low <= high or max_points < 3:
        raise ValueError("Invalid edge configuration")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    gray = cv2.GaussianBlur(gray, (0, 0), blur_sigma) if blur_sigma > 0 else gray
    f = gray.astype(np.float32)
    gx = cv2.Sobel(f, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(f, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = np.hypot(gx, gy)
    edges = (cv2.Canny(gray, low, high) > 0 if method == "canny" else magnitude > high)
    if roi is not None:
        x, y, w, h = map(int, roi)
        if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > gray.shape[1] or y+h > gray.shape[0]:
            raise ValueError("ROI outside image")
        allowed = np.zeros_like(edges)
        allowed[y:y+h, x:x+w] = True
        edges &= allowed
    if mask is not None:
        if np.shape(mask) != edges.shape:
            raise ValueError("Mask must match image dimensions")
        edges &= np.asarray(mask, bool)
    edges[[0, -1], :] = False
    edges[:, [0, -1]] = False
    y, x = np.nonzero(edges & (magnitude > 1e-6))
    if len(x) < 3:
        raise ValueError("Too few edge candidates")
    # Deterministic spatial subsampling, rather than selecting only high contrast.
    ids = np.linspace(0, len(x)-1, min(max_points, len(x)), dtype=int)
    x, y = x[ids], y[ids]
    strength = magnitude[y, x]
    nx, ny = gx[y, x]/strength, gy[y, x]/strength
    def sample(offset):
        return cv2.remap(magnitude, (x+offset*nx).astype(np.float32)[:, None],
                         (y+offset*ny).astype(np.float32)[:, None],
                         cv2.INTER_LINEAR).ravel()
    before, after = sample(-1), sample(1)
    curvature = before - 2*strength + after
    delta = np.zeros(len(x))
    good = curvature < -1e-6
    delta[good] = np.clip(0.5*(before[good]-after[good])/curvature[good], -0.5, 0.5)
    pixels = np.c_[x+delta*nx, y+delta*ny]
    return EdgeCandidates(pixels, np.c_[-ny, nx], strength / max(float(strength.max()), 1))


def undistort(image, K, distortion):
    """Undistort before extraction; output uses the same K and image dimensions."""
    return cv2.undistort(image, camera_matrix(K), np.asarray(distortion, float))
