# GPS-VIO Integration Alignment Report

This document describes how the current implementation of `gps_vio_calibration_node.py` and `vio_earth_visualization_node.py` aligns with the integration plan specified in `doc/how-to-integrate-vio-pointcloud-ecef-visualization.md`.

## Executive Summary

Both nodes have been updated to fully comply with the integration plan:
- **Calibration Node**: Handles time synchronization and spatial calibration between GPS (earth frame) and VIO (odom frame)
- **Visualization Node**: Uses TF to transform and visualize VIO data in Earth coordinates

## GPS-VIO Calibration Node

### Functionality Alignment
✅ **Time Synchronization** (Phase 1)
- Subscribes to `/gps/fix` and `/odom` 
- Implements affine time model: `t_corrected = a * t_gps + b`
- Publishes time sync parameters to `/sync/time_model` and `/sync/residual_error`

✅ **Spatial Calibration** (Phase 2)
- Computes rigid body transform between GPS (ECEF/earth) and VIO (odom) coordinates
- Uses Kabsch algorithm with RANSAC outlier rejection
- Directly computes `earth -> odom` transform without intermediate anchors

✅ **TF Publishing** (Phase 3)
- Publishes static transform to `/tf_static`
- Frame relationship: `earth -> odom`
- Automatic publishing when calibration converges

### Topics Interface
| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/gps/fix` | NavSatFix | Subscribe | GPS position input |
| `/odom` | Odometry | Subscribe | VIO odometry input |
| `/tf_static` | TFMessage | Publish | Static earth->odom transform |
| `/calibration/transform_earth_odom` | TransformStamped | Publish | Calibrated transform |
| `/calibration/quality` | Float64 | Publish | RMS alignment error |
| `/sync/time_difference` | Float64 | Publish | Current GPS-VIO time difference (seconds) |
| `/sync/clock_drift_rate` | Float64 | Publish | Clock drift rate (ppm) |
| `/sync/time_model` | Float64MultiArray | Publish | [a, b] parameters |
| `/sync/residual_error` | Float64 | Publish | Current sync error |

### Key Implementation Details
1. **Direct Transform Computation**: Removed anchor-based approach for cleaner earth->odom transform
2. **Time Sync Integration**: Uses synchronized data for spatial calibration
3. **Quality Metrics**: Publishes comprehensive diagnostics for monitoring

## VIO-Earth Visualization Node

### Functionality Alignment
✅ **Coordinate Transformation** (Phase 3)
- Uses TF to lookup `earth -> odom` transform
- Transforms VIO poses from odom to earth frame
- Transforms point clouds from odom to earth frame

✅ **Enhanced Visualization** (Phase 4)
- Visualizes VIO trajectories with orientation arrows
- Displays point clouds in Earth coordinates
- Real-time performance with separate processing rates

### Topics Interface
| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/odom` | Odometry | Subscribe | VIO odometry input |
| `/vio/points3d` | PointCloud2 | Subscribe | VIO point cloud input |
| `/vio/pose_earth` | PoseStamped | Publish | VIO pose in earth frame |
| `/vio/points3d_earth` | PointCloud2 | Publish | Point cloud in earth frame |
| `/visualization/vio_markers` | MarkerArray | Publish | VIO trajectory visualization |
| `/visualization/point_cloud_markers` | MarkerArray | Publish | Point cloud visualization |
| `/visualization/sensor_info` | String | Publish | Status information |

### Key Implementation Details
1. **TF-Based Transform**: Uses standard ROS2 TF for all coordinate transformations
2. **Performance Optimization**: Separate timers for odometry (10Hz) and point clouds (2Hz)
3. **Orientation Visualization**: Displays VIO poses with position and orientation arrows

## Data Flow Architecture

```
GPS (/gps/fix) ─────┐
                    ├──> GPS-VIO Calibration ──> earth->odom TF
VIO (/odom) ────────┘                                │
                                                     │
VIO (/odom) ────────┐                                ▼
                    ├──> VIO-Earth Visualization ──> Transformed Data
Points (/vio/points3d) ─┘   (uses TF lookup)         & Visualization
```

## TF Tree Structure

As specified in the integration plan:
```
Before Calibration:          After Calibration:
odom -> base_link           earth -> odom -> base_link
```

## Parameter Compliance

Both nodes use parameter structures that match the integration plan:
- Time sync parameters under `time_sync.*`
- Calibration parameters under `calibration.*`
- Frame IDs configurable: `frame_id_earth`, `frame_id_odom`

## Summary

The implementation fully aligns with the integration plan:
1. ✅ GPS-VIO time synchronization with published diagnostics
2. ✅ Spatial calibration computing direct earth->odom transform
3. ✅ TF-based coordinate transformation system
4. ✅ Enhanced visualization with orientation information
5. ✅ All specified topics and data flows implemented

The nodes work together to establish and use the `earth -> odom` transform, enabling VIO data visualization in Earth coordinates alongside GPS tracks on the 3D globe.