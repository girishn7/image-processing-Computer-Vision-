# image-processing-Computer-Vision-

Camera Calibration using DLT (Direct Linear Transform)
This MATLAB script performs camera calibration from 3D-2D point correspondences using the Direct Linear Transform (DLT) algorithm. It estimates the projection matrix, then decomposes it into intrinsic and extrinsic camera parameters. Finally, it computes reprojection errors to evaluate the calibration accuracy.

Features
Manually select 2D image points from a calibration image.
Use known 3D world coordinates of calibration points.
Compute the projection matrix (P) using SVD.
Decompose P into:
K: Intrinsic camera matrix (focal length, principal point, skew).
R: Rotation matrix (orientation of the camera).
t: Translation vector (position of the camera).
Calculate reprojection errors for validation.
