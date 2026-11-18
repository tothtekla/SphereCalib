import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from spherecalib.geometry import (pixel_rays, tangent_normals, fit_sphere_image,
    fit_sphere_oriented, fit_sphere_two_oriented, fit_sphere_llsb,
    angular_residual, sphere_conic, project)
from spherecalib.robust import fit_image, confidence_iterations
from spherecalib.lidar import fit_cloud, remove_planes, algebraic_sphere
from spherecalib.registration import Transform, register_points, robust_register, transform_error
from spherecalib.synthetic import silhouette, sphere_cloud, render_sphere, paired_scene
from spherecalib.edges import extract_edges
from spherecalib.pipeline import calibrate_observations, calibrate, CalibrationConfig
from spherecalib.data import load_manifest, read_cloud
from spherecalib.visualization import write_ply, color_cloud, projection_overlay, sphere_overlay
from spherecalib.conics import (fit_ellipse, fit_circle, ellipse_parameters,
    parametric_conic, coefficients_to_matrix, matrix_to_coefficients,
    conic_to_sphere, fit_circle_consensus, ellipse_distance)


class GeometryTests(unittest.TestCase):
    def test_conic_conversions_and_ellipse_fit(self):
        center, r = np.array([0.18, -0.08, 2.8]), 0.12
        xy, _, K, _ = silhouette(center, r)
        U = pixel_rays(xy, K)
        normalized = U[:, :2]/U[:, 2, None]
        Q = sphere_conic(center, r)
        np.testing.assert_allclose(coefficients_to_matrix(matrix_to_coefficients(Q)), Q)
        param = ellipse_parameters(Q)
        P = parametric_conic(param)
        np.testing.assert_allclose(P/P[0, 0], Q/Q[0, 0], atol=1e-10)
        fitted = fit_ellipse(normalized)
        np.testing.assert_allclose(conic_to_sphere(fitted, r), center, atol=1e-8)
        self.assertLess(ellipse_distance(normalized, param).max(), 1e-10)
        result = fit_circle_consensus(normalized, r, iterations=30)
        np.testing.assert_allclose(result.center, center, atol=1e-8)
        theta = np.linspace(0, 2*np.pi, 30, endpoint=False)
        c, radius = fit_circle(np.c_[np.cos(theta), np.sin(theta)]*3 + [2, -1])
        np.testing.assert_allclose(c, [2, -1], atol=1e-12)
        self.assertAlmostEqual(radius, 3)

    def test_exact_offaxis_and_axis_with_skew(self):
        K = np.array([[900, 7, 640], [0, 1100, 480], [0, 0, 1.]])
        for center in ([0, 0, 2.8], [0.18, -0.08, 2.8], [1, -0.5, 8]):
            for radius in (0.12, 0.3):
                xy, t, _, _ = silhouette(center, radius, K=K)
                U, N = pixel_rays(xy, K), tangent_normals(xy, t, K)
                for ids in ([0, 55, 120], np.arange(len(U))):
                    np.testing.assert_allclose(fit_sphere_image(U[ids], radius), center, atol=1e-9)
                    np.testing.assert_allclose(fit_sphere_llsb(U[ids], radius), center, atol=1e-9)
                for i, j in ((0, 60), (60, 0), (30, 120)):
                    for sign in (-1, 1):
                        s = fit_sphere_oriented(U[i], U[j], sign*N[i], radius)
                        np.testing.assert_allclose(s, center, atol=1e-9)
                    for sign1 in (-1, 1):
                        for sign2 in (-1, 1):
                            s = fit_sphere_two_oriented(U[i], U[j], sign1*N[i], sign2*N[j], radius)
                            np.testing.assert_allclose(s, center, atol=1e-9)
                h = np.c_[xy, np.ones(len(xy))] @ np.linalg.inv(K).T
                self.assertLess(np.max(np.abs(np.einsum('ni,ij,nj->n', h, sphere_conic(center, radius), h))), 1e-10)

    def test_degeneracies(self):
        with self.assertRaises(ValueError):
            fit_sphere_image(np.tile([0, 0, 1.], (3, 1)), 0.12)
        with self.assertRaises(ValueError):
            fit_sphere_oriented([0, 0, 1], [0, 0, 1], [1, 0, 0], 0.12)
        with self.assertRaises(ValueError):
            fit_sphere_image(np.eye(3), -1)
        with self.assertRaises(ValueError):
            fit_sphere_oriented([0, 0, 1], [0.1, 0, 1], [0, 0, 1], 0.12)
        with self.assertRaises(ValueError):
            fit_sphere_two_oriented([0, 0, 1], [0, 0, 1], [1, 0, 0], [0, 1, 0], 0.12)

    def test_two_oriented_randomized_exact(self):
        rng = np.random.default_rng(12)
        K = np.array([[870, 11, 630], [0, 1040, 470], [0, 0, 1.]])
        for case in range(20):
            radius = rng.uniform(0.05, 0.35)
            center = np.r_[rng.uniform(-0.8, 0.8, 2), rng.uniform(1.5, 8.0)]
            xy, tangents, _, _ = silhouette(center, radius, count=180, K=K, seed=case)
            rays = pixel_rays(xy, K)
            normals = tangent_normals(xy, tangents, K)
            i = int(rng.integers(0, 90))
            j = i + int(rng.integers(30, 90))
            signs = rng.choice([-1, 1], 2)
            fitted = fit_sphere_two_oriented(rays[i], rays[j],
                                             signs[0]*normals[i], signs[1]*normals[j], radius)
            np.testing.assert_allclose(fitted, center, atol=1e-9)

    def test_noisy_consensus_and_updated_mask(self):
        center = np.array([0.18, -0.08, 2.8])
        xy, t, K, _ = silhouette(center, 0.12, noise_px=0.3, tangent_noise_deg=2,
                                 outlier_fraction=0.25, seed=4)
        U, N = pixel_rays(xy, K), tangent_normals(xy, t, K)
        for method in ("3p", "1p5", "2op"):
            for refinement in ("none", "points", "oriented"):
                f = fit_image(U, 0.12, N, method, 0.0015, 150, 4, refinement)
                self.assertLess(np.linalg.norm(f.center-center), 0.06)
                np.testing.assert_array_equal(f.inliers, np.abs(angular_residual(f.center, U, 0.12)) < 0.0015)
        self.assertEqual(confidence_iterations(1, 3), 1)
        self.assertGreater(confidence_iterations(0.5, 3), confidence_iterations(0.5, 2))


class CloudAndRegistrationTests(unittest.TestCase):
    def test_cloud_methods_and_radius_modes(self):
        center = np.array([0.3, -0.1, 2.0])
        X = sphere_cloud(center, 0.12, noise_m=0.0005, outliers=50, seed=3)
        for method in ("adjacency", "random", "distance"):
            for radius in (None, 0.12):
                f = fit_cloud(X, radius, method, threshold=0.003, iterations=70,
                              neighborhood_radius=0.08, radius_bounds=(0.08, 0.18), seed=3)
                self.assertLess(np.linalg.norm(f.center-center), 0.006)
                self.assertLess(abs(f.radius-0.12), 0.006)
        with self.assertRaises(ValueError):
            algebraic_sphere(np.zeros((4, 3)))

    def test_plane_removal(self):
        rng = np.random.default_rng(9)
        plane = np.c_[rng.uniform(-2, 2, (150, 2)), np.zeros(150)]
        sphere = sphere_cloud([0, 0, 2], 0.12, count=80, outliers=0, seed=4)
        X = remove_planes(np.r_[plane, sphere], 1, threshold=0.001, iterations=50)
        self.assertEqual(len(X), 80)

    def test_registration_direction_translation_reflection(self):
        rng = np.random.default_rng(2)
        X = rng.normal(size=(20, 3)) + [5, -3, 2]
        T = Transform(Rotation.from_rotvec([0.2, -0.1, 0.05]).as_matrix(), np.array([0.4, -0.2, 0.3]))
        Y = T.apply(X)
        fit = register_points(X, Y)
        np.testing.assert_allclose(fit.matrix(), T.matrix(), atol=1e-12)
        np.testing.assert_allclose(fit.inverse().apply(Y), X, atol=1e-12)
        Y[:3] += 5
        fit, mask = robust_register(X, Y, 0.01, 100, 0)
        self.assertEqual(mask.sum(), 17)
        np.testing.assert_allclose(fit.matrix(), T.matrix(), atol=1e-12)
        mirror = X * [-1, 1, 1]
        self.assertAlmostEqual(np.linalg.det(register_points(X, mirror).rotation), 1)
        with self.assertRaises(ValueError):
            register_points(np.tile([1, 2, 3], (3, 1)), np.eye(3))


class PipelineTests(unittest.TestCase):
    def test_cloud_io_and_projection_diagnostics(self):
        X = np.array([[0, 0, 2.], [0.01, 0, 2], [0, 0.01, 2], [0, 0, -2]])
        K = np.array([[100, 0, 20], [0, 100, 20], [0, 0, 1.]])
        T = Transform(np.eye(3), np.zeros(3))
        image = np.full((40, 40, 3), [10, 20, 30], np.uint8)
        colors = color_cloud(image, X, T, K)
        np.testing.assert_array_equal(colors[0], [30, 20, 10])
        np.testing.assert_array_equal(colors[-1], [0, 0, 0])
        self.assertEqual(projection_overlay(image, X, T, K).shape, image.shape)
        self.assertEqual(sphere_overlay(image, [0, 0, 2], 0.12, K).shape, image.shape)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'cloud.ply'
            write_ply(path, X, colors)
            np.testing.assert_allclose(read_cloud(path), X)
            for endian, name in (('<', 'binary_little_endian'), ('>', 'binary_big_endian')):
                header = f'ply\nformat {name} 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
                path.write_bytes(header.encode('ascii') + X.astype(endian+'f4').tobytes())
                np.testing.assert_allclose(read_cloud(path), X)

    def test_paired_calibration_known_and_unknown_radius(self):
        rays, normals, clouds, centers, radius, truth = paired_scene(seed=3, noise_m=0.0003)
        cfg = CalibrationConfig(iterations=60, cloud_threshold_m=0.003, image_threshold_rad=0.0005)
        for known in (radius, None):
            result = calibrate_observations(rays, clouds, known, normals, cfg)
            errors = transform_error(result.camera_from_lidar, truth)
            self.assertLess(errors["translation_m"], 0.012)
            self.assertLess(errors["rotation_rad"], 0.005)
            json.dumps(result.to_dict(), allow_nan=False)

    def test_rasterized_image_pipeline(self):
        center = np.array([0.18, -0.08, 2.8])
        K = np.array([[950, 0, 640], [0, 950, 480], [0, 0, 1.]])
        image = render_sphere(center, 0.12, K)
        edge = extract_edges(image)
        f = fit_image(pixel_rays(edge.pixels, K), 0.12,
                      tangent_normals(edge.pixels, edge.tangents, K), iterations=100)
        # Finite-resolution rendering is an integration check, not a subpixel guarantee.
        self.assertLess(np.linalg.norm(f.center-center), 0.1)

    def test_manifest_image_to_extrinsics(self):
        centers = np.array([[-0.3, -0.2, 2.1], [0.4, -0.2, 2.5], [0.1, 0.3, 2.9], [-0.2, 0.3, 2.3]])
        K = np.array([[950, 0, 640], [0, 950, 480], [0, 0, 1.]])
        truth = Transform(Rotation.from_rotvec([0.05, -0.03, 0.02]).as_matrix(), np.array([0.2, -0.1, 0.15]))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frames = []
            for i, center in enumerate(centers):
                cv2.imwrite(str(root / f'{i}.png'), render_sphere(center, 0.12, K))
                np.save(root / f'{i}.npy', sphere_cloud(truth.inverse().apply(center), 0.12,
                                                       noise_m=0.0003, seed=i))
                frames.append(dict(image=f'{i}.png', cloud=f'{i}.npy'))
            manifest = dict(units="meters", pixel_origin=0, K=K.tolist(), radius_m=0.12, frames=frames)
            (root / 'frames.json').write_text(json.dumps(manifest))
            images, clouds, K, rois, meta = load_manifest(root / 'frames.json')
            result = calibrate(images, clouds, K, meta['radius_m'],
                               CalibrationConfig(iterations=60, cloud_threshold_m=0.003), rois)
            errors = transform_error(result.camera_from_lidar, truth)
            self.assertLess(errors['translation_m'], 0.08)
            self.assertLess(errors['rotation_rad'], 0.03)


if __name__ == '__main__':
    unittest.main()
