# How to Synchronize GPS (`gps/fix`) and VIO Odometry (`vio/odometry`) in ROS 2 (Jazzy+)

This guide describes online (runtime) time alignment between an asynchronous low‑rate GPS receiver (publishing `sensor_msgs/NavSatFix` on `gps/fix`) and a higher‑rate VIO odometry stream (`nav_msgs/Odometry` on `vio/odometry`). We assume:

* ROS 2 Jazzy (or later)
* VIO frame: `vio_base` (local, gravity-aligned but yaw drifting) producing pose + velocity (no true global yaw)
* GPS data expressed in the WGS84 geodetic reference frame ([Wikipedia: WGS84](https://en.wikipedia.org/wiki/World_Geodetic_System))
* Streams are NOT hardware time-synchronized; we must estimate the time offset and optionally small clock drift.

## Library choices

To reduce custom code and improve accuracy, standardize on a single geodesy library across the project. Recommended: pyproj (PROJ) or GeographicLib for all LLA↔ECEF/ENU conversions. For a lightweight WGS84-only choice, pymap3d also works. See the curated overview in [about-geodetic-compute-library.md](gps-points/about-geodetic-compute-library.md).

## Core Concepts

* Timestamp offset: A constant (slowly varying) difference between the ROS time stamps assigned to VIO and GPS messages.
* Clock drift: Slow linear change in offset; model as `t_gps ≈ a * t_vio + b`.
* Measurement association: Pairing a GPS fix with the closest (interpolated) VIO pose at the *true* acquisition time.
* Fusion prerequisites: Accurate temporal alignment is critical before performing sensor fusion (e.g., extended Kalman filter) to avoid bias and inconsistent covariance usage.

## Algorithm Overview

We maintain a sliding buffer of recent VIO odometry messages. For each incoming GPS fix:

1. Predict its acquisition time in VIO clock domain using the current affine time model.
2. Interpolate VIO pose at that predicted time.
3. Compute residual between GPS-derived position (converted to local ENU) and VIO position.
4. Optimize the time offset (and optionally drift) by minimizing a scalar error objective over a short window. Instead of a hand‑rolled grid search we can use `scipy.optimize.minimize_scalar`.

### Coordinate Frames

To compare positions, convert GPS geodetic coordinates (lat, lon, alt) to Earth-Centered Earth-Fixed (ECEF) ([Wikipedia: ECEF](https://en.wikipedia.org/wiki/ECEF)) then to a local East-North-Up (ENU) tangent frame anchored at the first GPS fix. Use either:
- `pyproj` (Python interface to PROJ: https://pyproj4.github.io/pyproj/stable/) or
- `geographiclib` (Accurate geodesic computations: https://geographiclib.sourceforge.io/)
Pick one and use it consistently project-wide to avoid axis-order and datum inconsistencies. For tradeoffs and examples, see [about-geodetic-compute-library.md](gps-points/about-geodetic-compute-library.md).

Reference frames: `[earth]` (ECEF) → `[enu_anchor]` (local ENU) → `[vio_base]` (VIO local frame). During time sync we only need *relative* positions; a rough extrinsic transform is acceptable at this stage.

## Mathematical Model

Let VIO message i have ROS stamp `t_vi`. Real world (true) time is `T`. We assume VIO clock error is negligible compared to GPS, or we pick VIO as reference clock. GPS message j has ROS stamp `t_gj` and true acquisition time `T_gj`.

Model: `T_gj = a * t_gj + b` (affine correction). We want to express GPS time in VIO time domain: `t_gj_vio = a * t_gj + b`.

Given corrected times, we interpolate VIO position `p_vio(t_gj_vio)` and compare with GPS position (expressed in ENU) `p_gps_j`.

Minimize over window W of recent GPS fixes:

`J(a,b) = Σ_j || p_vio(a * t_gj + b) - p_gps_j ||^2`

We solve incrementally with linearization (Gauss-Newton) or treat as 1D search if drift is small (set a≈1, solve for b via correlation / cross-covariance peak).

## Practical Online Method (Scalar Optimization + Incremental Drift)

1. Maintain ring buffer of VIO poses with time stamps.
2. For each new GPS fix, define an error function `E(δ) = || p_vio(a * t_gps + b + δ) - p_gps ||`. Use `scipy.optimize.minimize_scalar` over a bounded interval (e.g. ±2 s) to find δ*.
3. Update offset: `b ← (1-α) b + α (b + δ*)` (smoothing).
4. Periodically (e.g. every N GPS fixes) estimate drift `a` by linear regression (using `scipy.stats.linregress` or `numpy.polyfit`) on collected (t_gps_ros, corrected_time) pairs.
5. Smooth (a,b) with exponential moving average to avoid jitter.

### Robust estimation options

- Replace simple `polyfit`/`linregress` with scikit-learn’s `HuberRegressor` or `TheilSenRegressor` to estimate the affine time model `t_corrected = a * t_gps + b` under outliers.
- Optionally run a small Kalman filter (e.g., `filterpy`) over `[a, b]` to smooth updates in streaming conditions, instead of bespoke EMA tuning.
- Keep SciPy’s bounded `minimize_scalar` for per-fix local delta search.

## ROS 2 Node Implementation (Python `rclpy`)

Key libraries and ROS components:
- `rclpy`: ROS 2 Python client library (https://docs.ros.org/)
- `pyproj` or `geographiclib` for geodetic transforms.
- `numpy` for vector math (https://numpy.org/)
- `scipy.optimize` for continuous offset minimization (https://docs.scipy.org/doc/scipy/reference/optimize.html)
- (Optional) scikit-learn (`HuberRegressor`, `TheilSenRegressor`) for robust drift/offset regression; `filterpy` for simple Kalman filter smoothing of `[a, b]`.
- ROS: `message_filters` (ApproximateTime/TimeSynchronizer) to coarsely associate GPS↔VIO; `robot_localization` `navsat_transform_node` to convert GPS to local frames and handle orientation, reducing custom frame logic.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import numpy as np
from collections import deque
from pyproj import Transformer

class GPSTimeSyncNode(Node):
    def __init__(self):
        super().__init__('gps_vio_time_sync')
        self.declare_parameter('buffer_seconds', 10.0)
        self.declare_parameter('max_offset_search', 2.0)
        self.buffer_seconds = self.get_parameter('buffer_seconds').get_parameter_value().double_value
        self.max_offset_search = self.get_parameter('max_offset_search').get_parameter_value().double_value
        self.vio_buffer = deque()
        self.anchor_ecef = None
        self.a = 1.0  # drift
        self.b = 0.0  # offset
        self.alpha = 0.05  # smoothing
        self.transformer_lla_to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)  # lon,lat,h -> ECEF
        self.vio_sub = self.create_subscription(Odometry, 'vio/odometry', self.vio_cb, 50)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self.gps_cb, 10)
        self.get_logger().info('GPSTimeSyncNode started')

    def vio_cb(self, msg: Odometry):
        t = self.stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.vio_buffer.append((t, np.array([p.x, p.y, p.z])))
        # Prune
        t_now = t
        while self.vio_buffer and (t_now - self.vio_buffer[0][0]) > self.buffer_seconds:
            self.vio_buffer.popleft()

    def gps_cb(self, msg: NavSatFix):
        if not self.vio_buffer:
            return
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        x_ecef, y_ecef, z_ecef = self.transformer_lla_to_ecef.transform(lon, lat, alt)
        if self.anchor_ecef is None:
            self.anchor_ecef = np.array([x_ecef, y_ecef, z_ecef])
        p_gps_ecef = np.array([x_ecef, y_ecef, z_ecef]) - self.anchor_ecef
        # For offset search, treat ECEF difference as approximate local frame.

        t_gps_ros = self.stamp_to_sec(msg.header.stamp)
        # Initial corrected time using current model
        t_corrected_nom = self.a * t_gps_ros + self.b
        # Use SciPy minimize_scalar for local offset refinement
        try:
            from scipy.optimize import minimize_scalar
            def err_fn(delta):
                t_candidate = t_corrected_nom + delta
                p_vio = self.interpolate_vio(t_candidate)
                if p_vio is None:
                    return 1e9
                return np.linalg.norm(p_vio - p_gps_ecef)
            res = minimize_scalar(err_fn, bounds=(-self.max_offset_search, self.max_offset_search), method='bounded')
            best_delta = res.x if res.success else 0.0
            best_err = res.fun if res.success else float('nan')
        except ImportError:
            # Fallback simple coarse search
            best_delta = 0.0
            best_err = float('inf')
            for delta in np.linspace(-self.max_offset_search, self.max_offset_search, 21):
                t_candidate = t_corrected_nom + delta
                p_vio = self.interpolate_vio(t_candidate)
                if p_vio is None:
                    continue
                err = np.linalg.norm(p_vio - p_gps_ecef)
                if err < best_err:
                    best_err = err
                    best_delta = delta
        implied_b = (t_corrected_nom + best_delta - t_gps_ros)  # since a≈1
        self.b = (1.0 - self.alpha) * self.b + self.alpha * implied_b
        self.get_logger().info(f"Updated offset b={self.b:.4f}s err={best_err:.2f}m (delta={best_delta:.4f})")

    def interpolate_vio(self, t_query: float):
        """Interpolate VIO position at query time using library utilities.

        Preference order:
        1. SciPy interp1d (vectorized, robust)
        2. NumPy searchsorted + manual linear blend (minimal logic, no explicit binary loop)
        """
        if len(self.vio_buffer) < 2:
            return None
        times = np.fromiter((t for t, _ in self.vio_buffer), dtype=float)
        poses = np.vstack([p for _, p in self.vio_buffer])  # Nx3
        if t_query < times[0] or t_query > times[-1]:
            return None
        # Try SciPy first
        try:
            from scipy.interpolate import interp1d
            if not hasattr(self, '_interp_fn') or self._interp_last_len != len(times):
                # Rebuild interpolator when buffer length changes
                self._interp_fn = interp1d(times, poses, axis=0, kind='linear', assume_sorted=True, copy=False)
                self._interp_last_len = len(times)
            return self._interp_fn(t_query)
        except Exception:
            # Fallback: NumPy searchsorted (still using vectorized ops)
            idx = np.searchsorted(times, t_query)
            if idx == 0:
                return poses[0]
            if idx == len(times):
                return poses[-1]
            t1 = times[idx - 1]; t2 = times[idx]
            p1 = poses[idx - 1]; p2 = poses[idx]
            if t2 == t1:
                return p1
            w = (t_query - t1) / (t2 - t1)
            return p1 * (1.0 - w) + p2 * w

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

def main():
    rclpy.init()
    node = GPSTimeSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch Snippet

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='gps_vio_time_sync',
            parameters=[{'buffer_seconds': 15.0, 'max_offset_search': 1.5}],
            output='screen'
        )
    ])
```

## Validation Steps

1. Record a rosbag with `gps/fix` and `vio/odometry`.
2. Run the node in offline mode (play the bag) and plot residual position error vs time offset to confirm convex shape.
3. Inspect published logs; offset should converge quickly (< few seconds) if motion is sufficiently excited (avoid static data).

## Related Open Source Projects & Practical Examples

Leverage these existing packages and repositories for deeper, production-grade examples of time synchronization and GPS/VIO integration:

* robot_localization (EKF/UKF fusion, includes `navsat_transform_node` logic for temporal + spatial alignment): https://github.com/cra-ros-pkg/robot_localization  
    Integration guide (GPS): http://docs.ros.org/indigo/api/robot_localization/html/integrating_gps.html
* navsat_transform_node API reference (shows handling of GPS → local frame transform & timestamp usage): http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
* message_filters ApproximateTime / TimeSynchronizer examples (conceptual baseline even if we roll custom sync): https://docs.ros.org/ (search for `message_filters`)
* rclpy tutorials (timer, subscription patterns used here): https://docs.ros.org/
* rosbag2 (ROS 2 bag I/O for offline evaluation of synchronization algorithms): https://github.com/ros2/rosbag2
* VIO pipelines providing odometry topics you may pair with GPS:  
    * VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion  
    * ORB-SLAM3 (supports inertial, multi-map, can output pose): https://github.com/UZ-SLAMLab/ORB_SLAM3
* SciPy optimization (used for offset refinement): https://docs.scipy.org/doc/scipy/reference/optimize.html

When adopting code snippets ensure license compatibility (most above are permissive) and confirm ROS 2 Jazzy compatibility (some repos may target earlier distros—update build files / QoS profiles as needed).

## References

* WGS84: https://en.wikipedia.org/wiki/World_Geodetic_System
* ECEF: https://en.wikipedia.org/wiki/ECEF
* ENU frame: https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
* pyproj / PROJ: https://pyproj4.github.io/pyproj/stable/
* GeographicLib: https://geographiclib.sourceforge.io/
* SciPy Optimize (minimize_scalar): https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize_scalar.html
* scikit-learn robust regression (Huber/Theil-Sen): https://scikit-learn.org/stable/modules/linear_model.html#robustness-regression-outliers
* filterpy Kalman filtering: https://filterpy.readthedocs.io/
* ROS 2 rclpy: https://docs.ros.org/

---
Next: Once time-aligned, proceed to extrinsic calibration (see `howto-calibrate-gps-odometry.md`).
