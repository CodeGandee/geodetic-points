# Navigation system setup in ros2

We have a VIO+GPS system that provides odometry and GPS data, published via ROS2 topics, like the below:
- `vio/odometry` (nav_msgs/Odometry): VIO odometry data, IMU is 6-axis, so we do not have globally referenced orientation
- `vio/points3d` (sensor_msgs/PointCloud2): VIO observed 3D points in local frame
- `gps/fix` (sensor_msgs/NavSatFix): GPS data

these signals are NOT time synced.

we need to:
1, sync the data streams via online calibration
2, calibrate their extrinsics, for gps we use `ECEF` frame as the reference, compute the transform so that VIO odometry observed 3d points can be transformed to `ECEF` frame. The calibration can be done offline, as devices are mounted rigidly.
3, compute geodetic coordinates for the `vio/points3d` points, and publish them as:
  - `vio/points3d_geodetic` topic, 3d points in `ECEF` frame
  - `vio/points3d_pts` topic, 3d points in `sensor_msgs/NavSatFix` format, to be visualized with the gps signal

Now, you need to write several docs in `<workspace>/gps-points` folder:
- `howto-sync-gps-odometry.md`: how to sync the timestamps between gps and vio odometry online
- `howto-calibrate-gps-odometry.md`: how to calibrate the extrinsics between gps and vio odometry offline, assuming they are rigidly mounted and time synced.
- `howto-convert-points-to-gps.md`: how to convert the `vio/points3d` points to geodetic coordinates, and publish them as `vio/points3d_geodetic` and `vio/points3d_pts` topics

**Requirements:**
- use python mainly, c++ is ok if needed
- use ROS2 jazzy or later
- for all gis concepts, explain them with wikipedia links
- for all 3rd party libraries and algorithms, provide reference links