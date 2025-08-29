# How to Convert `vio/points3d` to Geodetic Coordinates and Publish (`vio/points3d_geodetic`, `vio/points3d_pts`)

We now have:
* Time-aligned VIO and GPS (see previous guides).
* Extrinsic transform `T_ecef_vio` (rotation R, translation t) mapping VIO local coordinates into an Earth-Centered Earth-Fixed (ECEF) frame.

Objective: Convert each `sensor_msgs/PointCloud2` in `vio/points3d` (local VIO frame) into:
1. A PointCloud2 in ECEF coordinates (`vio/points3d_geodetic`).
2. Individual geodetic points as `sensor_msgs/NavSatFix` messages on `vio/points3d_pts` (OPTIONAL stream; for visualization alongside GPS).

## Geospatial Concepts

* ECEF frame: Earth-centered Cartesian frame ([Wikipedia: ECEF](https://en.wikipedia.org/wiki/ECEF)).
* WGS84 ellipsoid: Reference model for Earth shape ([Wikipedia: WGS84](https://en.wikipedia.org/wiki/World_Geodetic_System)).
* Converting ECEF → geodetic (lat,lon,alt): Use iterative algorithms or library routines (e.g., `pyproj`, `geographiclib`).

## Data Flow

`PointCloud2 (VIO frame)` --(apply rigid transform)--> `PointCloud2 (ECEF)` --(per-point conversion)--> multiple `(lat,lon,alt)` → NavSatFix messages.

## Performance Considerations

* Per-point ECEF→LLA conversion is computationally heavier than rigid transform. To reduce overhead:
  - Downsample (`voxel_grid` or stride) for the NavSatFix stream while keeping full ECEF cloud.
  - Batch convert using vectorized `pyproj` transformer.
* Ensure you respect the coordinate precision; double precision recommended.

## ROS 2 Node (Python) Outline

Dependencies: `rclpy`, `sensor_msgs_py` helpers, `pyproj` or `geographiclib`, `numpy`, `scipy` (optional for quaternion / rotation utilities).

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, NavSatFix, NavSatStatus
from sensor_msgs_py import point_cloud2
import numpy as np
from pyproj import Transformer
from builtin_interfaces.msg import Time

class PointsToGeodetic(Node):
    def __init__(self):
        super().__init__('points_to_geodetic')
        # Parameters: rotation (row-major) and translation from vio->ecef
    self.declare_parameter('R', [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0])  # If you have quaternion, convert using scipy.spatial.transform.Rotation
        self.declare_parameter('t', [0.0,0.0,0.0])
        self.declare_parameter('anchor_ecef', [0.0,0.0,0.0])  # optional if transform already absolute
        self.declare_parameter('publish_navsat_stride', 50)  # every Nth point
        self.declare_parameter('frame_id_ecef', 'ecef_frame')
        self.publish_navsat_stride = self.get_parameter('publish_navsat_stride').value
        R_list = self.get_parameter('R').value
        self.R = np.array(R_list, dtype=float).reshape(3,3)
        self.t = np.array(self.get_parameter('t').value, dtype=float)
        self.anchor_ecef = np.array(self.get_parameter('anchor_ecef').value, dtype=float)
        self.frame_id_ecef = self.get_parameter('frame_id_ecef').value
        self.sub = self.create_subscription(PointCloud2, 'vio/points3d', self.cloud_cb, 10)
        self.pub_ecef = self.create_publisher(PointCloud2, 'vio/points3d_geodetic', 10)
        self.pub_navsat = self.create_publisher(NavSatFix, 'vio/points3d_pts', 10)
        self.transformer_ecef_to_lla = Transformer.from_crs('EPSG:4978', 'EPSG:4979', always_xy=True)

    def cloud_cb(self, msg: PointCloud2):
        points = np.array([p for p in point_cloud2.read_points(msg, skip_nans=True, field_names=("x","y","z"))])
        if points.size == 0:
            return
        # Transform to ECEF: p_ecef = R * p_vio + t + anchor
        p_ecef = (self.R @ points.T).T + self.t + self.anchor_ecef
        # Build ECEF PointCloud2
        cloud_ecef = point_cloud2.create_cloud_xyz32(msg.header, p_ecef.astype(np.float32))
        cloud_ecef.header.frame_id = self.frame_id_ecef
        self.pub_ecef.publish(cloud_ecef)
        # Publish subsampled NavSatFix points
        stride = self.publish_navsat_stride
        sel = p_ecef[::stride]
        if sel.shape[0] == 0:
            return
        # Vectorized ECEF->LLA (returns lon, lat, h)
        lon, lat, alt = self.transformer_ecef_to_lla.transform(sel[:,0], sel[:,1], sel[:,2])
        for la, lo, al in zip(lat, lon, alt):
            nav = NavSatFix()
            nav.header = msg.header
            nav.status.status = NavSatStatus.STATUS_FIX
            nav.status.service = NavSatStatus.SERVICE_GPS
            nav.latitude = la
            nav.longitude = lo
            nav.altitude = al
            # Covariance: optional; if unknown leave as zeros with COVARIANCE_TYPE_UNKNOWN
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub_navsat.publish(nav)

def main():
    rclpy.init()
    node = PointsToGeodetic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    R = [1,0,0,0,1,0,0,0,1]  # replace with calibrated
    t = [0,0,0]
    anchor = [0,0,0]
    return LaunchDescription([
        Node(
            package='your_package',
            executable='points_to_geodetic',
            parameters=[{'R': R, 't': t, 'anchor_ecef': anchor, 'publish_navsat_stride': 100}],
            output='screen'
        )
    ])
```

## Edge Cases & Notes

* If VIO scale is not exactly 1 (e.g., monocular VIO scale drift), ensure scale was resolved in calibration stage (Umeyama similarity) or incorporate additional scale factor.
* If VIO orientation drifts over long runs, recalibrate extrinsics or use online fusion (e.g., `robot_localization` EKF) to produce globally consistent pose, then transform points from fused frame.
* Large point clouds: consider publishing ECEF cloud with `sensor_msgs/PointField` in double precision (custom fields) if precision loss is observed.

## Validation

1. Visualize `vio/points3d_geodetic` with RViz using an ECEF -> ENU static transform or convert to ENU for display.
2. Compare downsampled `vio/points3d_pts` lat/lon with nearby GPS fixes to verify spatial consistency.
3. Compute bounding boxes / distances; mismatches > a few meters may indicate wrong extrinsics or time misalignment.

## RViz2 Quick Visualization Tips

* RViz2 does not natively include an Earth globe; for quick inspection convert ECEF points to a local ENU frame and publish as `PointCloud2` in a frame like `enu_map`.
* To visualize both original and transformed clouds, add two PointCloud2 displays and color by intensity or channel (add custom field if needed).
* To compare with GPS fixes, add the `TF` display and ensure the static transform tree includes: `earth` (ECEF) → `enu_map` → `vio_base`.
* For a full textured globe and global GPS track, see `howto-visualize-gps-globe.md`.

## References

* ECEF: https://en.wikipedia.org/wiki/ECEF
* WGS84: https://en.wikipedia.org/wiki/World_Geodetic_System
* Local tangent plane (ENU): https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
* pyproj / PROJ: https://pyproj4.github.io/pyproj/stable/
* GeographicLib: https://geographiclib.sourceforge.io/
* SciPy Rotation utilities: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
* sensor_msgs PointCloud2: https://docs.ros.org/

## Related Open Source Projects & Practical Examples

Use these projects and resources to see real-world implementations of GPS ↔ local frame transformations, sensor fusion, and point cloud handling:

* robot_localization (GitHub): https://github.com/cra-ros-pkg/robot_localization  
    Nonlinear state estimation (EKF/UKF) including `navsat_transform_node` which demonstrates transforming `NavSatFix` into a local frame. Docs: Integrating GPS Data – http://docs.ros.org/indigo/api/robot_localization/html/integrating_gps.html
* navsat_transform_node API docs (example of GPS → local conversion): http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
* Localization with ROS2 tutorials (robot_localization + SLAM examples): https://github.com/christiaantheunisse/localization-with-ROS
* Sensor fusion tutorial (Automatic Addison): https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/
* point_cloud2 helper (ROS2 port gist): https://gist.github.com/SebastianGrans/dc301fa507910a4f079a0016bbcbd10a  
    Reference for reading/writing PointCloud2 fields in Python.
* ecef2lla minimal conversion utility (pure math reference): https://github.com/kvenkman/ecef2lla  
    Can be compared against `pyproj` / `geographiclib` results for validation.
* pointcloud_to_laserscan (ROS perception): https://github.com/ros-perception/pointcloud_to_laserscan  
    Illustrates efficient PointCloud2 iteration + republishing patterns.

When adopting snippets, ensure ROS 2 Jazzy or later compatibility (some tutorials target earlier distros; adjust package names and QoS settings accordingly).

---
This completes the conversion pipeline from local VIO points to geodetic coordinates.
