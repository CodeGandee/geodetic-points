# VIO-Earth Integration Usage Guide

## Quick Start

### 1. Launch the Calibration System
```bash
# Launch both calibration and visualization nodes
ros2 launch geodetic_points integrated_vio_gps_viz.launch.py

# Or headless (without RViz)
ros2 launch geodetic_points integrated_vio_gps_headless.launch.py
```

### 2. Monitor Calibration Progress
```bash
# Watch time synchronization
ros2 topic echo /sync/time_difference    # Actual time offset in seconds
ros2 topic echo /sync/clock_drift_rate   # Clock drift in ppm
ros2 topic echo /sync/residual_error     # Sync quality

# Monitor spatial calibration
ros2 topic echo /calibration/quality
ros2 topic echo /calibration/transform_earth_odom

# Check TF tree
ros2 run tf2_tools view_frames
```

### 3. Verify Data Flow
```bash
# Confirm VIO data is being transformed
ros2 topic hz /vio/pose_earth
ros2 topic hz /vio/points3d_earth

# Check visualization markers
ros2 topic echo /visualization/sensor_info
```

## Key Topics

### Input Topics
- `/gps/fix` - GPS position fixes (sensor_msgs/NavSatFix)
- `/odom` - VIO odometry (nav_msgs/Odometry)
- `/vio/points3d` - VIO point clouds (sensor_msgs/PointCloud2)

### Output Topics
- `/vio/pose_earth` - VIO poses in Earth frame
- `/vio/points3d_earth` - Point clouds in Earth frame
- `/visualization/vio_markers` - VIO trajectory visualization
- `/visualization/point_cloud_markers` - Point cloud visualization

### Calibration Topics
- `/sync/time_difference` - GPS-VIO time difference (seconds)
- `/sync/clock_drift_rate` - Clock drift rate (ppm)
- `/sync/time_model` - Time sync parameters [a, b]
- `/calibration/quality` - RMS alignment error
- `/tf_static` - Earth->odom transform

## Parameter Tuning

### Time Synchronization
```yaml
time_sync:
  buffer_seconds: 15.0        # VIO buffer duration
  max_offset_search: 2.0      # Maximum time offset to search
  smoothing_alpha: 0.05       # Time model update smoothing
```

### Spatial Calibration
```yaml
calibration:
  min_pairs: 10               # Minimum data pairs for calibration
  max_error: 10.0             # Maximum acceptable RMS error (meters)
  convergence_threshold: 0.01 # Convergence criteria (meters)
```

## Troubleshooting

### No Calibration Output
- Ensure both GPS and VIO data are being published
- Check for sufficient motion (> 20m trajectory)
- Verify GPS has a valid fix

### High Calibration Error
- Check GPS accuracy and multipath conditions
- Ensure VIO is not drifting excessively
- Increase outlier rejection threshold

### No Visualization
- Confirm TF transform is published: `ros2 run tf2_ros tf2_echo earth odom`
- Check frame IDs match your system configuration
- Verify data is arriving at correct topics

## Expected Behavior

1. **Initial Phase**: System collects GPS-VIO pairs while estimating time sync
2. **Calibration Phase**: After sufficient motion, computes earth->odom transform
3. **Operational Phase**: Continuously transforms and visualizes VIO data in Earth frame

The calibration typically converges within 60 seconds of motion with good GPS reception.