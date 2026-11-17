"""File-based projection diagnostics replacing MATLAB interactive figures."""
from pathlib import Path
import cv2
import numpy as np
from .geometry import project, sphere_conic, camera_matrix


def projection_overlay(image, lidar_points, camera_from_lidar, K, step=1):
    result = image.copy()
    X = camera_from_lidar.apply(np.asarray(lidar_points)[::step])
    X = X[np.isfinite(X).all(axis=1) & (X[:, 2] > 0)]
    if not len(X):
        return result
    pixels = np.rint(project(X, K)).astype(int)
    valid = ((pixels[:, 0] >= 0) & (pixels[:, 0] < image.shape[1]) &
             (pixels[:, 1] >= 0) & (pixels[:, 1] < image.shape[0]))
    for x, y in pixels[valid]:
        cv2.circle(result, (int(x), int(y)), 1, (0, 0, 255), -1)
    return result


def sphere_overlay(image, center, radius, K):
    from .conics import ellipse_parameters
    inv = np.linalg.inv(camera_matrix(K))
    parameters = ellipse_parameters(inv.T @ sphere_conic(center, radius) @ inv)
    a, b, x, y, angle = parameters
    result = image.copy()
    cv2.ellipse(result, (round(x), round(y)), (round(a), round(b)),
                float(np.rad2deg(angle)), 0, 360, (0, 255, 0), 1, cv2.LINE_AA)
    return result


def write_ply(path, points, colors=None):
    from .geometry import points_array
    X = points_array(points, 3)
    if colors is not None:
        colors = np.asarray(colors)
        if colors.shape != X.shape or not np.isfinite(colors).all():
            raise ValueError("RGB colors must match points")
        colors = np.clip(colors, 0, 255).astype(np.uint8)
    with Path(path).open('w', encoding='ascii', newline='\n') as stream:
        stream.write(f"ply\nformat ascii 1.0\nelement vertex {len(X)}\nproperty double x\nproperty double y\nproperty double z\n")
        if colors is not None:
            stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("end_header\n")
        if colors is None:
            np.savetxt(stream, X, fmt='%.17g')
        else:
            np.savetxt(stream, np.c_[X, colors], fmt=['%.17g']*3 + ['%d']*3)


def color_cloud(image, lidar_points, camera_from_lidar, K):
    X = camera_from_lidar.apply(lidar_points)
    colors = np.zeros((len(X), 3), np.uint8)
    positive = np.flatnonzero(X[:, 2] > 0)
    if not len(positive):
        return colors
    xy = np.rint(project(X[positive], K)).astype(int)
    valid = ((xy[:, 0] >= 0) & (xy[:, 0] < image.shape[1]) &
             (xy[:, 1] >= 0) & (xy[:, 1] < image.shape[0]))
    xy = xy[valid]
    colors[positive[valid]] = image[xy[:, 1], xy[:, 0], ::-1]  # BGR -> RGB
    return colors
