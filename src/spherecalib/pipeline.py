"""Paired-frame calibration; one target per frame with ordered correspondence."""
from dataclasses import dataclass, field
import numpy as np
from .geometry import pixel_rays, tangent_normals, positive_radius
from .edges import extract_edges, undistort
from .lidar import fit_cloud, remove_planes
from .robust import fit_image
from .registration import register_points, robust_register


@dataclass
class CalibrationConfig:
    image_method: str = "3p"
    refinement: str = "points"
    cloud_method: str = "adjacency"
    iterations: int = 1000
    image_threshold_rad: float = 0.002
    circle_threshold_normalized: float = 0.004
    cloud_threshold_m: float = 0.02
    radius_bounds: tuple = (0.01, 10.0)
    plane_count: int = 0
    close_threshold_m: float = 0.0
    registration_threshold_m: float | None = None
    registration_consistency_m: float = 0.05
    seed: int = 0
    edge_options: dict = field(default_factory=dict)


@dataclass
class CalibrationResult:
    camera_from_lidar: object
    camera_centers: np.ndarray
    lidar_centers: np.ndarray
    radius: float
    image_fits: list
    cloud_fits: list
    registration_inliers: np.ndarray
    registration_errors_m: np.ndarray
    consistency_threshold_m: float

    def to_dict(self):
        return {"T_camera_from_lidar": self.camera_from_lidar.matrix().tolist(),
                "camera_centers_m": self.camera_centers.tolist(),
                "lidar_centers_m": self.lidar_centers.tolist(), "radius_m": self.radius,
                "registration_inliers": self.registration_inliers.tolist(),
                "registration_errors_m": self.registration_errors_m.tolist(),
                "registration_rmse_m": float(np.sqrt(np.mean(self.registration_errors_m[self.registration_inliers]**2))),
                "consistency_threshold_m": self.consistency_threshold_m,
                "registration_consistent": bool(np.sqrt(np.mean(self.registration_errors_m[self.registration_inliers]**2)) <= self.consistency_threshold_m),
                "image_inlier_counts": [int(f.inliers.sum()) for f in self.image_fits],
                "cloud_inlier_counts": [int(f.inliers.sum()) for f in self.cloud_fits]}


def calibrate_observations(rays, clouds, radius=None, normals=None, config=None):
    """Calibrate from measured rays/clouds; errors identify their frame index.

    If radius is absent, estimate it per cloud and use their mean for images,
    as in the MATLAB pipeline. With radius supplied, constrain cloud fits too.
    """
    cfg = config or CalibrationConfig()
    if cfg.registration_consistency_m <= 0:
        raise ValueError("Registration consistency threshold must be positive")
    if len(rays) != len(clouds) or len(rays) < 3:
        raise ValueError("Need at least three paired target observations")
    if normals is not None and len(normals) != len(rays):
        raise ValueError("Normal observations must match frames")
    if radius is not None:
        positive_radius(radius)
    cloud_fits = []
    for i, X in enumerate(clouds):
        try:
            X = remove_planes(X, cfg.plane_count, cfg.close_threshold_m, seed=cfg.seed+i)
            cloud_fits.append(fit_cloud(X, radius, cfg.cloud_method, cfg.cloud_threshold_m,
                                        cfg.iterations, radius_bounds=cfg.radius_bounds, seed=cfg.seed+i))
        except ValueError as error:
            raise ValueError(f"Cloud frame {i}: {error}") from error
    r = float(radius if radius is not None else np.mean([f.radius for f in cloud_fits]))
    image_fits = []
    for i, U in enumerate(rays):
        try:
            if cfg.image_method == "circle":
                from .conics import fit_circle_consensus
                U = np.asarray(U, float)
                if np.any(U[:, 2] <= 0):
                    raise ValueError("Expected forward rays")
                image_fits.append(fit_circle_consensus(U[:, :2]/U[:, 2, None], r,
                                  cfg.circle_threshold_normalized, cfg.iterations, seed=cfg.seed+i))
            else:
                image_fits.append(fit_image(U, r, None if normals is None else normals[i],
                                            cfg.image_method, cfg.image_threshold_rad,
                                            cfg.iterations, cfg.seed+i, cfg.refinement))
        except ValueError as error:
            raise ValueError(f"Image frame {i}: {error}") from error
    camera_centers = np.array([f.center for f in image_fits])
    lidar_centers = np.array([f.center for f in cloud_fits])
    if cfg.registration_threshold_m is None:
        T = register_points(lidar_centers, camera_centers)
        mask = np.ones(len(rays), bool)
    else:
        T, mask = robust_register(lidar_centers, camera_centers,
                                  cfg.registration_threshold_m, cfg.iterations, cfg.seed)
    errors = np.linalg.norm(T.apply(lidar_centers)-camera_centers, axis=1)
    return CalibrationResult(T, camera_centers, lidar_centers, r,
                             image_fits, cloud_fits, mask, errors, cfg.registration_consistency_m)


def calibrate(images, clouds, K, radius=None, config=None, rois=None, distortion=None):
    cfg = config or CalibrationConfig()
    if len(images) != len(clouds) or (rois is not None and len(rois) != len(images)):
        raise ValueError("Images, clouds and ROIs must match")
    rays, normals = [], []
    for i, image in enumerate(images):
        if distortion is not None:
            image = undistort(image, K, distortion)
        edges = extract_edges(image, roi=None if rois is None else rois[i], **cfg.edge_options)
        rays.append(pixel_rays(edges.pixels, K))
        normals.append(tangent_normals(edges.pixels, edges.tangents, K))
    return calibrate_observations(rays, clouds, radius, normals, cfg)
