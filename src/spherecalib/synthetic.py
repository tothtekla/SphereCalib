"""Analytic perspective silhouettes and matched synthetic cloud observations."""
import cv2
import numpy as np
from .geometry import unit, positive_radius, project, camera_matrix
from .registration import Transform


def silhouette(center, radius, count=200, K=None, noise_px=0.0,
               tangent_noise_deg=0.0, outlier_fraction=0.0, seed=0):
    r = positive_radius(radius)
    s = np.asarray(center, float)
    if s.shape != (3,) or s[2] <= r or count < 3 or not 0 <= outlier_fraction < 1:
        raise ValueError("Invalid synthetic sphere")
    K = camera_matrix(K if K is not None else [[950, 0, 640], [0, 950, 480], [0, 0, 1]])
    rng = np.random.default_rng(seed)
    direction = unit(s)
    a = unit(np.cross(direction, [0, 1, 0] if abs(direction[1]) < 0.9 else [1, 0, 0]))
    b = np.cross(direction, a)
    angle = np.arcsin(r / np.linalg.norm(s))
    phase = np.linspace(0, 2*np.pi, count, endpoint=False)
    U = np.cos(angle)*direction + np.sin(angle)*(np.cos(phase)[:, None]*a + np.sin(phase)[:, None]*b)
    dU = np.sin(angle)*(-np.sin(phase)[:, None]*a + np.cos(phase)[:, None]*b)
    h, dh = U @ K.T, dU @ K.T
    xy = project(U, K)
    tangents = unit((dh[:, :2]*h[:, 2, None]-h[:, :2]*dh[:, 2, None]) / h[:, 2, None]**2)
    xy += rng.normal(0, noise_px, xy.shape)
    da = rng.normal(0, np.deg2rad(tangent_noise_deg), count)
    tangents = np.c_[np.cos(da)*tangents[:, 0]-np.sin(da)*tangents[:, 1],
                     np.sin(da)*tangents[:, 0]+np.cos(da)*tangents[:, 1]]
    is_inlier = np.ones(count, bool)
    bad = rng.choice(count, int(count*outlier_fraction), replace=False)
    xy[bad] += rng.normal(0, 100, (len(bad), 2))
    tangents[bad] = unit(rng.normal(size=(len(bad), 2))) if len(bad) else tangents[bad]
    is_inlier[bad] = False
    return xy, tangents, K, is_inlier


def sphere_cloud(center, radius, count=300, noise_m=0.001, outliers=50,
                 visible_only=True, seed=0):
    positive_radius(radius)
    rng = np.random.default_rng(seed)
    center = np.asarray(center, float)
    normals = unit(rng.normal(size=(count, 3)))
    if visible_only:
        normals[normals @ center > -radius] *= -1
        # A visible spherical cap rather than a full sphere.
        direction = unit(-center)
        normals = unit(normals + 1.5*direction)
    points = center + radius*normals + rng.normal(0, noise_m, (count, 3))
    background = center + rng.uniform(-3*radius, 3*radius, (outliers, 3))
    return np.r_[points, background]


def render_sphere(center, radius, K, shape=(960, 1280), supersample=3, blur_sigma=0.8):
    xy, _, _, _ = silhouette(center, radius, count=720, K=K)
    canvas = np.full((shape[0]*supersample, shape[1]*supersample), 30, np.uint8)
    cv2.fillPoly(canvas, [np.round(xy*supersample).astype(np.int32)], 220)
    image = cv2.resize(canvas, (shape[1], shape[0]), interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(image, (0, 0), blur_sigma) if blur_sigma else image


def paired_scene(seed=0, noise_px=0.0, noise_m=0.001):
    from scipy.spatial.transform import Rotation
    from .geometry import pixel_rays, tangent_normals
    T = Transform(Rotation.from_rotvec([0.08, -0.05, 0.03]).as_matrix(), np.array([0.2, -0.1, 0.15]))
    centers = np.array([[-0.45, -0.2, 2.4], [0.4, -0.3, 2.8], [0.1, 0.35, 3.2],
                        [-0.3, 0.25, 2.7], [0.5, 0.1, 3.5], [-0.1, -0.4, 3.1]])
    r = 0.12
    rays, normals, clouds = [], [], []
    for i, c in enumerate(centers):
        xy, tangents, K, _ = silhouette(c, r, noise_px=noise_px, seed=seed+i)
        rays.append(pixel_rays(xy, K))
        normals.append(tangent_normals(xy, tangents, K))
        clouds.append(sphere_cloud(T.inverse().apply(c), r, noise_m=noise_m, seed=seed+i))
    return rays, normals, clouds, centers, r, T
