# Verified research and comparison review

Updated 2026-11-17. This file records claims checked against primary papers, official dataset pages or official code repositories. It is a living review, not a novelty opinion.

## Sphere estimation from images

Tóth and Hajder, *A Minimal Solution for Image-Based Sphere Estimation*, IJCV 131 (2023), 1428–1447, DOI [10.1007/s11263-023-01766-1](https://doi.org/10.1007/s11263-023-01766-1), proposes 3pFit, Ell2Sphere and Direct3pFit. Three calibrated contour points determine the constrained spherical projection; Direct3pFit also supports overdetermined point sets. The paper compares against Sun et al.'s LLSB and Shi et al.'s IMM, and compares robust image detection with general five-point/direct ellipse and circle-based approaches. Its synthetic study varies contour count and 0/1/2 px noise and includes adjacent-arc ill-conditioning tests. The current Python equal-cone implementation corresponds to Direct3pFit's center equation, but numerical MATLAB parity has not yet been established.

Sun, He and Zeng, *Global Calibration of Multiple Cameras Based on Sphere Targets*, Sensors 16(1):77 (2016), DOI [10.3390/s16010077](https://doi.org/10.3390/s16010077), is represented by the LLSB baseline reproduced from the SphereCalib dev research code. Shi et al., *Sphere Localization from a Minimal Number of Points in a Single Image* (CTISC 2019), DOI [10.5220/0008096300650070](https://doi.org/10.5220/0008096300650070), supplies the IMM comparison in the IJCV paper. IMM is not yet independently reimplemented in Python; the incomplete exploratory MATLAB version is insufficient for a parity claim.

The proposed oriented 1.5p method in this repository uses one contour position plus its tangent-plane orientation and a second contour position. No prior-art search completed so far establishes novelty. Its evaluation must treat tangent reliability as an additional stochastic measurement, report both roots/sign/visibility checks, and compare against two-oriented-point and overdetermined alternatives before a publication claim.

## Calibration methods and public data

Tóth, Pusztai and Hajder, *Automatic LiDAR-Camera Calibration of Extrinsic Parameters Using a Spherical Target*, ICRA 2020, DOI [10.1109/ICRA40945.2020.9197316](https://doi.org/10.1109/ICRA40945.2020.9197316), is the legacy MATLAB pipeline preserved in this repository. It detects the sphere in images and clouds, then registers corresponding 3D centers.

[CalibRGBD](https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD.html), version 2019_03, provides paired RGB/point-cloud data and processed measurements. Its synthetic set contains exact constructed sphere centers, radius, intrinsics and extrinsics, 20 sphere observations, eleven depth-noise levels and 100 repetitions. The real Random, Equidistant and Double sets use manual segmentation and fitted centers for reference; those fits must not be called surveyed sphere-center GT. The benchmark supplies results for DLT, Staranowicz and Boas variants. The official paper is [A Benchmark Dataset for RGB-D Sphere-Based Calibration](https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/article/submission.pdf).

The 2024 RA-L paper *Automatic Extrinsic Parameter Calibration for Camera-LiDAR Fusion Using Spherical Target*, DOI [10.1109/LRA.2024.3397072](https://doi.org/10.1109/LRA.2024.3397072), proposes curvature-consistency sphere detection for clouds and enhanced ellipse detection with perspective eccentricity compensation. No verified official code/data release has been integrated here, so it remains a paper-level comparison target.

[MARSCalib](https://github.com/sparolab/MARSCalib) is official code for an IROS 2025 spherical-target pipeline. It uses ROS2, PCL, OpenCV, Ceres, GTSAM and Segment Anything; its sample link covers Ouster OS1-32, Livox Mid-360 and MLX-120 sequences. Its image stage uses SAM masks followed by ellipse detection, while its cloud stage uses accumulated clouds, range-image Hough detection and sphere fitting. Running it is a separate environment-level reproduction, and its estimated centers are not independent surveyed GT.

## Additional evaluation datasets

[SHREC 2022](https://github.com/chiararomanengo/SHREC2022) supplies train/test point clouds for planes, cylinders, spheres, cones and tori with exact primitive parameters. It includes clean, noisy, undersampled, missing-part and locally deformed cases. It is suitable for sphere fitting and primitive recognition, not paired image/cloud calibration.

[BAGSFit](https://github.com/ai4ce/BAGSFit) supplies Blensor scenes with noisy/noiseless point coordinates, class/instance labels, camera poses and primitive parameters. It is relevant for cluttered multi-instance cloud segmentation, but not a paired RGB sphere-center calibration benchmark.

The [TUM RGB-D benchmark](https://webshare.cvg.cit.tum.de/g/rgbd/dataset/) lists `freiburg2_metallic_sphere` and `sphere2` sequences with a 150 mm diameter sphere, RGB/depth and camera-trajectory GT. It does not provide surveyed sphere centers per frame, so it supports qualitative robustness and consistency rather than exact center-accuracy claims.

## Current comparison boundary

The repository can now compare Direct3pFit, 1.5p, LLSB, conic/ellipse fitting, circle consensus, several cloud fitting modes, robust registration, and the official CalibRGBD output baselines. It cannot yet claim comparison with IMM, CCSD, MARSCalib inference, SAM, BAGSFit's trained model or recent learned edge detectors. Those require verified implementations, fixed weights/environments and identical data splits. Neural target proposal should be evaluated separately from subpixel boundary localization because a segmentation mask boundary is not automatically a calibrated edge measurement.
