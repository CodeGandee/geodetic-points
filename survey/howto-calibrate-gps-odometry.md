# How to Calibrate Extrinsics Between GPS and VIO (Offline)

This document covers estimating the rigid transform between the VIO local frame (`vio_base`) and a global Earth-fixed representation derived from GPS. We assume:

* Time synchronization has been performed (see `howto-sync-gps-odometry.md`).
* Rigid mounting between sensors ⇒ constant rotation R and translation t.
* VIO provides a 6-DoF pose (with potentially drifting yaw) relative to an arbitrary local origin.
* GPS provides global positions (WGS84) at lower rate; we convert them to ECEF then to a local ENU frame for numerical stability.

## Goal

Find transformation `T_ecef_vio = [ R | t ; 0 0 0 1 ]` such that:
`p_ecef ≈ R * p_vio + t`
for corresponding positions. If yaw in VIO drifts (no absolute heading), we rely on overlapping trajectory shape to estimate yaw alignment.

## Library choices

Use a mature geodesy library end‑to‑end to avoid hand‑rolled formulas:
- Python: pyproj (PROJ) or GeographicLib for LLA↔ECEF/ENU. For WGS84‑only workflows, pymap3d is a light alternative.
- C++: GeographicLib (Geocentric and LocalCartesian) for precise ECEF↔LLA and ENU.
See the curated guide: [about-geodetic-compute-library.md](gps-points/about-geodetic-compute-library.md).

## Pipeline

1. Convert all GPS fixes to ECEF ([Wikipedia: ECEF](https://en.wikipedia.org/wiki/ECEF)) then subtract first fix to center data (avoid large magnitudes).
2. Collect synchronized VIO positions at GPS timestamps (interpolate VIO using corrected time).
3. (Optional) Remove segments with low motion (near-zero velocity) to reduce degeneracy.
4. Use an off‑the‑shelf solver:
    - Rigid (SE(3), scale = 1): `open3d.pipelines.registration.TransformationEstimationPointToPoint` (avoid custom SVD).
    - Similarity (Sim(3), if scale uncertain): Umeyama via NumPy or Open3D helpers.
    - Robust option (outliers): TEASER++ global registration; or Open3D RANSAC to seed, then refine with point‑to‑point ICP.
    If scale ≈ 1, constrain to rigid.
5. Validate residuals and compute RMS error.

### Why ENU / Local Tangent?

Direct ECEF coordinates are large (~10^6 m). Subtracting an anchor (ECEF of first GPS fix) yields a local quasi-Cartesian frame comparable in scale to VIO output. See [Local tangent plane](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates).

## Mathematical Formulation (Rigid Alignment / Procrustes)

Given paired sets `{p_i}` (VIO) and `{q_i}` (GPS in local frame), find R, t minimizing:
`Σ_i || q_i - (R p_i + t) ||^2` subject to `R ∈ SO(3)`.

Instead of hand-coding Kabsch you can directly call `scipy.spatial.transform.Rotation.align_vectors(Q_demeaned, P_demeaned)` (note ordering) which returns a `Rotation` object and residual. Then compute translation.

References:
* Kabsch algorithm: https://en.wikipedia.org/wiki/Kabsch_algorithm
* Procrustes analysis: https://en.wikipedia.org/wiki/Procrustes_analysis
* Umeyama method (for similarity, scale): https://ieeexplore.ieee.org/document/88573

## Python Offline Calibration Script

Dependencies: `numpy`, `scipy` (for `Rotation.align_vectors`), `pyproj` or `geographiclib`, optionally `rosbag2_py` to read ROS 2 bags.

```python
#!/usr/bin/env python3
import numpy as np
from pyproj import Transformer
from scipy.spatial.transform import Rotation as R

def geodetic_to_ecef(lat, lon, alt):
    transformer_lla_to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    x, y, z = transformer_lla_to_ecef.transform(lon, lat, alt)
    return np.array([x, y, z])

def rigid_align(vio_xyz, gps_local):
    P = np.asarray(vio_xyz)
    Q = np.asarray(gps_local)
    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)
    P_c = P - mu_P
    Q_c = Q - mu_Q
    rot_obj, rssd = R.align_vectors(Q_c, P_c)  # Rotates P_c -> Q_c (ordering matters)
    R_mat = rot_obj.as_matrix()
    t = mu_Q - R_mat @ mu_P
    residuals = Q - (P @ R_mat.T + t)
    rms = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
    return R_mat, t, rms, rssd

def calibrate(vio_xyz, gps_lla):
    gps_ecef = np.array([geodetic_to_ecef(*lla) for lla in gps_lla])
    anchor = gps_ecef[0]
    gps_local = gps_ecef - anchor
    R_mat, t, rms, rssd = rigid_align(vio_xyz, gps_local)
    return R_mat, t, anchor, rms

if __name__ == '__main__':
    # Replace with real synchronized data loading
    vio_xyz = np.random.randn(200, 3)
    R_true = np.eye(3)
    t_true = np.array([5.0, -2.0, 3.0])
    gps_local = vio_xyz @ R_true.T + t_true + np.random.normal(0, 0.1, size=vio_xyz.shape)
    # Placeholder LLA (set all to origin then won't match real transform; demonstration only)
    gps_lla = [(0.0, 0.0, 0.0)] * len(gps_local)
    R_est, t_est, anchor, rms = calibrate(vio_xyz, gps_lla)
    print('R_est=', R_est)
    print('t_est=', t_est)
    print('RMS=', rms)
```

Replace the synthetic section with real synchronized arrays. In practice you'll:
1. Load `gps/fix` messages → (lat,lon,alt,time).
2. Load `vio/odometry` messages → (x,y,z,time).
3. Interpolate VIO positions at GPS times (using final time model from sync stage).
4. Execute the calibration function.

## Related Open Source Projects & Practical Examples

These resources show practical implementations of pose alignment, frame transforms, and GPS integration which complement the calibration process:

- robot_localization (for fused state output useful in validating extrinsics): https://github.com/cra-ros-pkg/robot_localization
- navsat_transform_node (reference for transforming GPS to local frames, check how it handles orientation & offsets): http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
- Open3D (for ICP / Umeyama similarity transform if scale needs verification): https://github.com/isl-org/Open3D
- TEASER++ (certifiably robust registration under heavy outliers): https://github.com/MIT-SPARK/TEASER-plusplus
- evo (trajectory alignment toolkit; compute best‑fit SE(3)/scale between synced trajectories and export the transform): https://github.com/MichaelGrupp/evo
- scipy.spatial (Rotation & alignment utilities): https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.align_vectors.html
- VIO systems providing trajectories for calibration datasets:
    - VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
    - ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- rosbag2 tooling for extracting synchronized pose/fix pairs: https://github.com/ros2/rosbag2
- tf2 tutorials (publishing static and dynamic transforms): https://docs.ros.org/

Suggested workflow reuse: first validate recovered R,t by re-projecting VIO trajectory onto GPS ENU points and computing RMS & max error; then feed static transform into robot_localization to confirm fused output remains stable. As a no‑code baseline, align the trajectories with `evo` to verify the SE(3) (and scale) agrees with your solver.

## Storing the Transform in ROS 2

Broadcast static transform (use `static_transform_publisher` or custom Python node). Example launch snippet:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Suppose R,t known; convert to quaternion
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                'tx','ty','tz','qx','qy','qz','qw',
                'ecef_frame','vio_base'
            ]
        )
    ])
```

Use quaternion conversion from R (e.g., `scipy.spatial.transform.Rotation`). Provide the real numeric values.

## Quality Metrics

* RMS residual < typical GPS noise (≈ 2–5 m for consumer GNSS) indicates consistency.
* Check residual distribution; large biases may mean yaw drift or unsynchronized segments.
* Re-run calibration excluding outliers (e.g., > 3σ) for refinement.

## References

* WGS84: https://en.wikipedia.org/wiki/World_Geodetic_System
* ECEF: https://en.wikipedia.org/wiki/ECEF
* Local tangent plane (ENU): https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
* Kabsch algorithm: https://en.wikipedia.org/wiki/Kabsch_algorithm
* Procrustes analysis: https://en.wikipedia.org/wiki/Procrustes_analysis
* Umeyama (1991): https://ieeexplore.ieee.org/document/88573
* SciPy Rotation.align_vectors: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.align_vectors.html
* Open3D registration: https://www.open3d.org/
* TEASER++: https://github.com/MIT-SPARK/TEASER-plusplus
* evo (trajectory evaluation/alignment): https://github.com/MichaelGrupp/evo
* scikit-learn RANSACRegressor: https://scikit-learn.org/stable/modules/linear_model.html#ransac-regression
* pyproj / PROJ: https://pyproj4.github.io/pyproj/stable/
* GeographicLib: https://geographiclib.sourceforge.io/

---
Next: Convert VIO 3D points to geodetic coordinates (see `howto-convert-points-to-gps.md`).
