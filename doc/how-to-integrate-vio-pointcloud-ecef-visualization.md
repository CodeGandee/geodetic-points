# VIO-GPS Calibration and Earth Coordinate System Integration

This document provides a comprehensive development plan to establish coordinate transformation between the VIO odometry frame (`odom`) and the Earth-Centered Earth-Fixed (ECEF) coordinate frame (`earth`) through GPS-VIO calibration. The system enables real-time transformation of VIO trajectories and point clouds to global geodetic coordinates for visualization alongside GPS tracks on a 3D Earth globe.

## Implementation Status

✅ **COMPLETED**: The integration plan has been fully implemented with two merged nodes:
- `gps_vio_calibration_node.py` - Combines time sync and spatial calibration functionality
- `vio_earth_visualization_node.py` - Combines coordinate transformation and visualization

See [integration_alignment_report.md](../geodetic_points/integration_alignment_report.md) for implementation details.

## Implemented Node Architecture

### 1. GPS-VIO Calibration Node (`gps_vio_calibration_node.py`)

**Purpose**: Performs complete GPS-VIO calibration by combining time synchronization and spatial calibration into a unified process.

**Core Functionality**:
- **Time Synchronization**: Estimates affine time model `t_corrected = a * t_gps + b` to align GPS and VIO timestamps
- **Spatial Calibration**: Computes rigid transform `T_earth_odom` using Kabsch algorithm with RANSAC outlier rejection
- **TF Publishing**: Publishes static `earth -> odom` transform for coordinate transformations

**Key Features**:
- Real-time time synchronization with drift estimation
- Robust spatial alignment using synchronized GPS-VIO pairs
- Automatic transform publishing when calibration converges
- Comprehensive diagnostics including time difference, drift rate, and calibration quality

### 2. VIO-Earth Visualization Node (`vio_earth_visualization_node.py`)

**Purpose**: Transforms and visualizes VIO data (odometry and point clouds) in Earth coordinates using the calibrated transform.

**Core Functionality**:
- **Coordinate Transformation**: Uses TF to lookup `earth -> odom` transform and transform VIO data
- **VIO Trajectory Visualization**: Creates trajectory markers with position and orientation arrows
- **Point Cloud Visualization**: Transforms and visualizes VIO point clouds in Earth frame
- **Performance Optimization**: Separate processing rates for odometry (10Hz) and point clouds (2Hz)

**Key Features**:
- Real-time transformation using TF2
- Adaptive point cloud downsampling based on distance
- Visual distinction between VIO (green) and GPS (red) trajectories
- Status information publishing for monitoring

## Node Cooperation and Data Flow

The two nodes work together in a producer-consumer pattern:

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│  GPS-VIO Calibration Node   │         │ VIO-Earth Visualization Node │
│                             │         │                              │
│ Inputs:                     │         │ Inputs:                      │
│ - /gps/fix                  │         │ - /odom                      │
│ - /odom                     │         │ - /vio/points3d              │
│                             │         │                              │
│ Process:                    │         │ Process:                     │
│ 1. Time synchronization     │         │ 1. Wait for TF available     │
│ 2. Spatial calibration      │         │ 2. Lookup earth->odom TF     │
│ 3. Publish earth->odom TF   ├────────►│ 3. Transform VIO to earth    │
│                             │   TF    │ 4. Visualize in earth frame │
│ Outputs:                    │         │                              │
│ - /tf_static                │         │ Outputs:                     │
│ - /sync/time_difference     │         │ - /vio/pose_earth            │
│ - /sync/clock_drift_rate    │         │ - /vio/points3d_earth        │
│ - /calibration/quality      │         │ - /visualization/vio_markers │
└─────────────────────────────┘         └─────────────────────────────┘
```

### Cooperation Mechanism

1. **Initialization Phase**:
   - Both nodes start independently
   - Calibration node begins collecting GPS and VIO data
   - Visualization node waits for TF transform to become available

2. **Calibration Phase**:
   - Calibration node performs time synchronization to align GPS-VIO timestamps
   - Using synchronized data, it computes the spatial transform between earth and odom frames
   - Quality metrics are continuously published for monitoring

3. **Operational Phase**:
   - Once calibration converges, the transform is published to `/tf_static`
   - Visualization node detects the available transform and begins operation
   - All VIO data is transformed to Earth coordinates in real-time
   - Both GPS and VIO trajectories are visualized in the unified Earth frame

### Published Topics for Inter-Node Communication

**From Calibration Node**:
- `/tf_static` - Static transform broadcaster publishing `earth -> odom`
- `/sync/time_difference` - Current GPS-VIO time offset (seconds)
- `/sync/clock_drift_rate` - Clock drift rate (parts per million)
- `/calibration/transform_earth_odom` - Full transform message
- `/calibration/quality` - RMS calibration error (meters)

**From Visualization Node**:
- `/vio/pose_earth` - VIO poses transformed to Earth frame
- `/vio/points3d_earth` - Point clouds in Earth coordinates
- `/visualization/vio_markers` - VIO trajectory visualization
- `/visualization/sensor_info` - System status information

## Practical Usage Guide

### Quick Start with Unified Launch

```bash
# Launch complete VIO-GPS integration system
ros2 launch geodetic_points vio_gps_integration.launch.py

# With custom parameters
ros2 launch geodetic_points vio_gps_integration.launch.py \
  scale:=10 \
  enable_calibration:=true \
  enable_visualization:=true \
  gps_topic:=/gps/fix \
  odom_topic:=/odom
```

### Monitoring System Operation

```bash
# 1. Check calibration progress
ros2 topic echo /calibration/quality   # Should decrease to < 10m
ros2 topic echo /sync/time_difference  # Time offset in seconds
ros2 topic echo /sync/clock_drift_rate # Drift in ppm

# 2. Verify TF is published
ros2 run tf2_tools view_frames         # Should show: earth -> odom -> base_link
ros2 run tf2_ros tf2_echo earth odom   # Monitor transform values

# 3. Check transformed data flow
ros2 topic hz /vio/pose_earth          # Should match /odom rate
ros2 topic hz /vio/points3d_earth      # Should be ~2 Hz

# 4. Monitor visualization
ros2 topic echo /visualization/sensor_info
```

### Configuration Best Practices

#### Time Synchronization Tuning
```yaml
# For systems with stable clocks
time_sync:
  buffer_seconds: 15.0        # Shorter buffer for faster sync
  smoothing_alpha: 0.05       # Lower value for more stability

# For systems with clock drift
time_sync:
  buffer_seconds: 30.0        # Longer buffer to estimate drift
  smoothing_alpha: 0.1        # Higher value for faster adaptation
```

#### Calibration Quality Control
```yaml
calibration:
  min_pairs: 20               # More pairs for better accuracy
  max_error: 10.0             # Reject if error > 10m
  convergence_threshold: 0.1  # Tighter convergence
  outlier_threshold: 2.0      # Stricter outlier rejection
```

#### Visualization Performance
```yaml
# For dense point clouds
cloud:
  downsample_factor: 20       # Aggressive downsampling
  adaptive_sizing: true       # Distance-based culling
  max_display_distance: 500.0 # Limit render distance

# For smooth trajectory display
vio:
  max_vio_points: 10000       # Larger trajectory buffer
  point_radius: 2.0           # Smaller markers
```

### Troubleshooting Common Issues

**No Transform Published**:
- Ensure sufficient motion (> 20m) for calibration
- Check GPS fix quality: `ros2 topic echo /gps/fix | grep status`
- Verify both GPS and VIO data are publishing

**High Calibration Error**:
- GPS multipath/urban canyon effects
- VIO drift or scale issues
- Solution: Collect data in open area with good GPS

**Visualization Not Appearing**:
- Transform not available yet - wait for calibration
- Frame ID mismatch - check parameter consistency
- RViz Fixed Frame should be set to `earth`

**Time Sync Failing**:
- Clock jump in system time
- GPS time not properly set
- Solution: Use NTP time sync, restart nodes after clock correction

## Technical Implementation Details

### Time Synchronization Algorithm

The GPS-VIO calibration node implements a two-stage time synchronization:

1. **Cross-Correlation Analysis**:
   - Buffers VIO positions with timestamps
   - For each GPS fix, searches for optimal time offset
   - Minimizes spatial distance after applying current transform estimate

2. **Affine Time Model**:
   ```python
   t_vio_corrected = a * t_gps + b
   ```
   - `a`: Clock drift factor (1.0 = no drift)
   - `b`: Time offset (seconds)
   - Updated using exponential moving average for stability

### Spatial Calibration Algorithm

The calibration uses the Kabsch algorithm with RANSAC:

1. **Data Collection**:
   - Synchronized GPS-VIO position pairs
   - Motion threshold ensures spatial diversity
   - Sliding window maintains recent history

2. **Rigid Alignment**:
   - Centers both point sets
   - SVD-based rotation estimation
   - Translation computed from centroids
   - RANSAC rejects outliers based on residual threshold

3. **Transform Direction**:
   - Algorithm computes: `odom -> earth`
   - TF requires: `earth -> odom` (parent -> child)
   - Published transform is inverted

### Coordinate Systems and Transformations

```
GPS (lat/lon/alt) → ECEF (x,y,z in earth frame)
                          ↓
                    Calibration finds
                    T_earth_odom
                          ↓
VIO (odom frame) → Transform via TF → Earth frame
```

**ECEF Conversion**:
- Uses WGS84 ellipsoid parameters
- pyproj for accurate transformation
- Fallback implementation for systems without pyproj

**TF Chain**:
```
earth (ECEF/fixed)
  └── odom (VIO local)
       └── base_link (robot body)
```

### Performance Optimizations

1. **Asynchronous Processing**:
   - Odometry: High-rate callback, immediate transform
   - Point Clouds: Buffered, processed at lower rate
   - Visualization: Independent timer for smooth display

2. **Adaptive Downsampling**:
   - Distance-based point cloud reduction
   - Close: Full resolution
   - Medium: 1/5 sampling
   - Far: 1/20 sampling

3. **Memory Management**:
   - Circular buffers with max size
   - Point cloud size limits
   - Automatic old data pruning

### System Requirements and Dependencies

**Required ROS2 Packages**:
- `tf2_ros`: Transform library
- `tf2_geometry_msgs`: Geometry message transforms
- `sensor_msgs_py`: Point cloud utilities

**Python Libraries**:
- `numpy`: Numerical computations
- `scipy` (optional): Optimized algorithms
- `pyproj` (optional): Geodetic conversions

**Performance Specifications**:
- Time sync: < 10ms per GPS update
- Calibration: < 100ms per iteration
- Transform lookup: < 1ms
- Visualization: 20+ FPS with full data

## Complete Example Workflow

### 1. System Startup Sequence

```bash
# Terminal 1: Launch the integrated system
ros2 launch geodetic_points vio_gps_integration.launch.py scale:=10

# Terminal 2: Monitor calibration progress
watch -n 1 'ros2 topic echo /calibration/quality --once'

# Terminal 3: Check time synchronization
ros2 topic echo /sync/time_difference
```

### 2. Calibration Data Collection

The system requires sufficient motion for calibration:

```bash
# Monitor data collection
ros2 topic echo /visualization/sensor_info

# Expected output progression:
# "VIO: 10 poses | Cloud: 0 points | Odom: 100 msgs | Clouds: 0 msgs"
# "VIO: 50 poses | Cloud: 1000 points | Odom: 500 msgs | Clouds: 10 msgs"
# ...calibration begins after sufficient motion...
```

### 3. Verify Calibration Success

```bash
# Check transform is published
ros2 run tf2_ros tf2_echo earth odom

# Expected output:
# At time X.X
# - Translation: [X.X, Y.Y, Z.Z]
# - Rotation: in Quaternion [X.X, Y.Y, Z.Z, W.W]

# Verify data transformation
ros2 topic hz /vio/pose_earth    # Should show ~10 Hz
ros2 topic hz /vio/points3d_earth # Should show ~2 Hz
```

### 4. Visualization in RViz

The system automatically launches RViz with proper configuration:
- Fixed Frame: `earth`
- GPS trajectory: Red trail and points
- VIO trajectory: Green trail with orientation arrows
- VIO point clouds: Blue points
- Earth globe: Textured sphere at origin

### 5. Recording Calibrated Data

```bash
# Record all calibrated data
ros2 bag record \
  /gps/fix \
  /vio/pose_earth \
  /vio/points3d_earth \
  /tf_static \
  /calibration/transform_earth_odom \
  -o calibrated_session

# Replay with visualization
ros2 launch geodetic_points vio_gps_integration.launch.py \
  bag_file:=calibrated_session.db3 \
  bag_rate:=1.0
```

## System Architecture Summary

The implemented system consists of two cooperating nodes that establish and use the `earth -> odom` coordinate relationship:

1. **Calibration Node**: Continuously estimates the transform between GPS (earth) and VIO (odom) coordinates
2. **Visualization Node**: Uses the calibrated transform to display VIO data in Earth coordinates

This architecture enables:
- Unified visualization of GPS and VIO trajectories on a 3D Earth globe
- Real-time transformation of VIO point clouds to global coordinates
- Robust handling of time synchronization and spatial alignment
- Scalable design for additional sensor integration

---

# Original Development Plan

The following sections contain the original detailed development plan that guided the implementation:

## Overview

Building upon the existing GPS globe visualization system with the current TF architecture (`odom -> base_link`), this integration establishes:

1. **GPS-VIO Time Synchronization**: Accurate temporal alignment between GPS fixes and VIO odometry
2. **Rigid Body Calibration**: Compute transformation matrix `T_earth_odom` through GPS-VIO spatial alignment
3. **Coordinate System Bridge**: Establish `earth -> odom` TF relationship for seamless coordinate transformation
4. **Enhanced Visualization**: Display VIO trajectories and point clouds in Earth coordinates with position and orientation

## Prerequisites

- Existing geodetic-points visualization system (GPS globe visualization with `frame_id=earth`)
- Current TF tree: `odom -> base_link` (VIO odometry providing nav_msgs/Odometry)
- VIO point cloud data on `/vio/points3d` (sensor_msgs/PointCloud2 in `odom` frame)
- GPS data on `/gps/fix` (sensor_msgs/NavSatFix) converted to ECEF coordinates in `earth` frame
- Mission requirement: Bridge `odom` and `earth` coordinate systems through calibration

## System Architecture

```
Current TF Tree:         Target TF Tree (after calibration):
                        
odom -> base_link       earth -> odom -> base_link
                              ↑
                        (established via GPS-VIO calibration)

┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│  VIO Odometry   │    │  Time Sync Node  │    │  Calibration Node   │
│                 │    │                  │    │                     │
│ odom->base_link ├───→│ Synchronize GPS  ├───→│ Compute T_earth_odom│
│ /odom (topic)   │    │ and VIO timestamps│    │ via spatial align   │
│ /vio/points3d   │    │                  │    │                     │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
         │                       │                        │
         │                       ▼                        ▼
         │              ┌──────────────────┐    ┌─────────────────────┐
         │              │  GPS-ECEF Node   │    │  TF Publisher Node  │
         │              │ (existing)       │    │                     │
         └─────────────→│ /gps/fix -> ECEF ├───→│ Publish earth->odom │
                        │ frame_id=earth   │    │ static transform    │
                        └──────────────────┘    └─────────────────────┘
                                 │                        │
                                 ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│ Coord Transform │    │   Globe Viz      │    │  Enhanced Markers   │
│                 │    │  (existing)      │    │                     │
│ odom->earth:    ├───→│ GPS tracks in    ├───→│ VIO poses with      │
│- VIO poses      │    │ earth frame      │    │ position+orientation│
│- Point clouds   │    │                  │    │ VIO point clouds    │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
```

## Development Phases

### Phase 1: GPS-VIO Time Synchronization Implementation

**Objective**: Establish accurate temporal alignment between GPS fixes and VIO odometry data

**Reference**: `survey/howto-sync-gps-odometry.md`

**Tasks**:
1. Create `gps_odom_time_sync_node.py`
   - Subscribe to `/gps/fix` and `/odom` topics
   - Implement sliding window buffer for VIO odometry poses
   - Detect and correct GPS timestamp offset and drift
2. Implement time alignment algorithm:
   - Cross-correlation analysis between GPS and VIO position changes
   - Affine time model: `t_vio_corrected = a * t_gps + b`
   - Rolling window estimation for long-term stability
3. Publish time synchronization diagnostics and corrected timestamps

**Deliverables**:
- ROS 2 node: `geodetic_points/gps_odom_time_sync_node.py`
- Time sync diagnostic topics: `/sync/time_offset`, `/sync/quality_metrics`
- Launch file: `launch/gps_odom_sync.launch.py`

### Phase 2: GPS-VIO Rigid Body Calibration Implementation

**Objective**: Compute transformation matrix `T_earth_odom` to establish coordinate relationship

**Reference**: `survey/howto-calibrate-gps-odometry.md`

**Tasks**:
1. Create `odom_earth_calibration_node.py`
   - Use synchronized GPS-VIO data from Phase 1
   - Extract VIO trajectory in `odom` frame from odometry integration
   - Convert GPS fixes to ECEF coordinates in `earth` frame (reuse existing functionality)
2. Implement spatial alignment algorithms:
   - Kabsch algorithm for rigid body alignment (rotation + translation)
   - RANSAC outlier rejection for robust calibration
   - Support for similarity transform (includes scale) if needed
3. Compute calibration quality metrics:
   - RMS alignment error between GPS and VIO trajectories
   - Temporal consistency of calibration parameters
   - Convergence indicators and confidence intervals

**Deliverables**:
- ROS 2 node: `geodetic_points/odom_earth_calibration_node.py`
- Calibration output: transformation parameters `T_earth_odom` (position + quaternion)
- Quality assessment tools and validation metrics
- Offline calibration script: `scripts/offline_odom_earth_calibration.py`

### Phase 3: TF-Based Coordinate Transformation Implementation

**Objective**: Utilize calibrated `earth -> odom` transform to convert VIO data to Earth coordinates

**Tasks**:
1. Create `earth_odom_tf_publisher_node.py`
   - Subscribe to calibrated transformation parameters from Phase 2
   - Publish static TF transform `earth -> odom` to TF tree
   - Provide transform validation and monitoring
2. Enhance existing `points_transform_node.py`
   - Subscribe to `/vio/points3d` (PointCloud2 in `odom` frame)
   - Use TF listener to transform point clouds from `odom` to `earth` frame
   - Publish transformed point clouds to `/vio/points3d_earth`
   - Add optional geodetic coordinate fields (lat, lon, alt)
3. Create `odom_to_earth_pose_node.py`
   - Subscribe to `/odom` topic (nav_msgs/Odometry)
   - Transform VIO poses from `odom` to `earth` frame using TF
   - Publish as geometry_msgs/PoseStamped to `/vio/pose_earth`

**Deliverables**:
- TF publisher node: `geodetic_points/earth_odom_tf_publisher_node.py`
- Enhanced transformation node supporting TF-based conversion
- VIO pose transformer: `geodetic_points/odom_to_earth_pose_node.py`

### Phase 4: Enhanced Earth-Frame Visualization

**Objective**: Integrate VIO trajectory and point clouds into existing GPS globe visualization

**Tasks**:
1. Create `vio_earth_marker_node.py`
   - Subscribe to VIO poses in Earth frame (`/vio/pose_earth`)
   - Generate trajectory markers with position AND orientation information
   - Use different visual styling from GPS tracks (color, size, shape)
   - Support real-time trajectory updating and smoothing
2. Enhance existing `gps_on_globe_node.py`
   - Add subscription to VIO point clouds in Earth frame (`/vio/points3d_earth`)
   - Implement point cloud marker visualization in ECEF coordinates
   - Support adaptive point cloud downsampling based on distance from camera
   - Integrate VIO and GPS data timing for synchronized visualization
3. Create unified visualization launcher
   - Combine GPS globe, VIO trajectory, and point cloud visualization
   - Provide camera controls that work with both GPS and VIO data
   - Support switching between different visualization modes

**Deliverables**:
- VIO trajectory visualization: `geodetic_points/vio_earth_marker_node.py`
- Enhanced globe visualization with point cloud support
- Integrated launch files: `launch/integrated_gps_vio_earth_viz.launch.py`
- RViz configuration files for multi-sensor Earth visualization

## Implementation Details

### Node Architecture

```python
# Core nodes to implement/enhance:

1. geodetic_points/gps_odom_time_sync_node.py
   - Subscribes: /gps/fix, /odom
   - Publishes: /sync/time_offset, /sync/quality_metrics
   - Function: GPS-VIO temporal alignment

2. geodetic_points/odom_earth_calibration_node.py
   - Subscribes: /gps/fix, /odom, /sync/time_offset
   - Publishes: /calibration/transform_earth_odom, /calibration/quality
   - Function: Compute T_earth_odom rigid body transform

3. geodetic_points/earth_odom_tf_publisher_node.py
   - Subscribes: /calibration/transform_earth_odom
   - Publishes: TF static_transform_publisher (earth -> odom)
   - Function: Establish TF relationship

4. geodetic_points/odom_to_earth_pose_node.py
   - Subscribes: /odom
   - Publishes: /vio/pose_earth (geometry_msgs/PoseStamped)
   - Function: Transform VIO poses to Earth frame via TF

5. geodetic_points/points_transform_node.py (enhanced)
   - Subscribes: /vio/points3d
   - Publishes: /vio/points3d_earth (PointCloud2 in earth frame)
   - Function: Transform point clouds via TF lookup

6. geodetic_points/vio_earth_marker_node.py
   - Subscribes: /vio/pose_earth, /vio/points3d_earth
   - Publishes: /vio/trajectory_markers, /vio/pointcloud_markers
   - Function: VIO data visualization in Earth coordinates

7. geodetic_points/gps_on_globe_node.py (enhanced)
   - Subscribes: /gps/fix, /vio/trajectory_markers, /vio/pointcloud_markers
   - Publishes: /gps_globe_points, /gps_globe_trail (existing)
   - Function: Integrated GPS+VIO globe visualization
```

### Data Flow Pipeline

```
GPS + VIO Sensors → Time Sync → Spatial Calibration → TF Publishing → Coordinate Transform → Visualization
       ↓                ↓             ↓                   ↓                 ↓                ↓
   Raw Topics    → Aligned Data → T_earth_odom →    earth->odom TF  →  Earth Coordinates → Globe Display
   /gps/fix           timestamps    parameters        (static)           /vio/pose_earth    (markers)
   /odom                                                                 /vio/points3d_earth
   /vio/points3d
```

### TF Tree Evolution

```
Before Calibration:              After Calibration:
                                
odom -> base_link               earth -> odom -> base_link
  ↑                                ↑       ↑
VIO system                     GPS-VIO   VIO system
                               calibration
```

### Configuration Management

**Parameters Structure**:
```yaml
# GPS-VIO time synchronization parameters
gps_odom_time_sync:
  odom_buffer_size: 1000           # VIO pose buffer size
  gps_buffer_size: 100             # GPS fix buffer size
  time_window_seconds: 30.0        # Analysis window for sync
  cross_correlation_threshold: 0.7 # Minimum correlation for valid sync
  affine_model_update_rate: 0.1    # Time model update smoothing
  publish_diagnostics: true        # Enable sync quality monitoring

# GPS-VIO spatial calibration parameters  
odom_earth_calibration:
  min_trajectory_length: 20.0      # Minimum motion for calibration (meters)
  min_time_span: 60.0             # Minimum time span (seconds)
  ransac_threshold: 5.0           # Outlier rejection threshold (meters)
  max_iterations: 1000            # RANSAC iterations
  convergence_tolerance: 0.01     # Spatial alignment convergence
  similarity_transform: false     # Use rigid (false) vs similarity (true)
  quality_check_interval: 5.0    # Calibration quality update rate (Hz)

# TF publication parameters
earth_odom_tf_publisher:
  static_tf_rate: 1.0             # Static TF publication rate (Hz)
  frame_id_earth: "earth"         # Earth/ECEF frame name
  frame_id_odom: "odom"           # VIO odometry frame name
  validate_transform: true        # Enable transform validation
  max_transform_age: 300.0        # Max age for valid calibration (seconds)

# Point cloud transformation parameters
points_transform:
  input_frame: "odom"             # Input point cloud frame
  output_frame: "earth"           # Output point cloud frame
  downsample_factor: 5            # Point reduction factor
  max_points_per_cloud: 50000     # Limit for performance
  tf_timeout_ms: 100              # TF lookup timeout
  add_geodetic_fields: true       # Add lat/lon/alt fields to output

# VIO trajectory visualization parameters
vio_earth_visualization:
  trajectory_color: [0.0, 1.0, 0.0, 0.9]    # Green with alpha
  trajectory_line_width: 0.05               # Meters
  pose_arrow_length: 2.0                    # Orientation arrow length
  pose_arrow_width: 0.5                     # Orientation arrow width
  trajectory_buffer_size: 5000              # Max poses to display
  update_rate: 10.0                         # Visualization update rate (Hz)
  
# Point cloud visualization parameters  
pointcloud_visualization:
  point_color: [0.2, 0.6, 1.0, 0.7]        # Light blue with alpha
  point_size: 0.02                          # Point marker size (meters)
  adaptive_sizing: true                     # Scale points by distance
  max_points_displayed: 100000              # Performance limit
  distance_culling: 1000.0                  # Max display distance (meters)

# Integrated globe visualization parameters
integrated_globe_viz:
  gps_color: [1.0, 0.1, 0.1, 0.9]         # Red for GPS
  vio_color: [0.1, 1.0, 0.1, 0.9]         # Green for VIO
  pointcloud_color: [0.1, 0.1, 1.0, 0.6]   # Blue for point clouds
  camera_follow_vio: true                   # Camera follows VIO trajectory
  sync_visualization_time: true            # Synchronize GPS and VIO display
  earth_frame: "earth"                      # Reference frame for globe
```

## Quality Assurance & Validation

### Testing Strategy

1. **GPS-VIO Time Synchronization Tests**:
   - Cross-correlation accuracy between GPS and VIO motion
   - Time drift model convergence over extended periods
   - Robustness to GPS outages and VIO resets

2. **Spatial Calibration Tests**:
   - Kabsch alignment convergence with synthetic datasets
   - RANSAC outlier rejection effectiveness
   - Calibration stability across different trajectory geometries

3. **Coordinate Transformation Tests**:
   - TF chain integrity and timing consistency
   - Point cloud transformation accuracy preservation
   - Real-time transformation performance under load

4. **Visualization Integration Tests**:
   - Multi-sensor data synchronization in Earth frame
   - Camera control responsiveness with VIO and GPS data
   - Marker rendering performance with large datasets

### Validation Metrics

- **Time Sync Quality**: 
  - Cross-correlation coefficient > 0.7
  - RMS timestamp error < 100ms after convergence
  - Time drift model stability over 30+ minute sessions

- **Spatial Calibration Quality**:
  - Alignment RMS error < 3m (within GPS accuracy bounds)
  - Transformation consistency over multiple calibration runs
  - RANSAC inlier ratio > 70% for valid calibration

- **System Performance**:
  - Point cloud transformation: Handle 50K+ points at 10Hz
  - TF lookup latency: < 5ms for coordinate transformations
  - Visualization frame rate: > 20 FPS with GPS + VIO + point clouds

### Performance Benchmarks

- **Time Synchronization**: < 15ms processing latency per message pair
- **Calibration Computation**: < 500ms for transform estimation
- **TF-Based Coordinate Transform**: < 2ms per pose/point cloud
- **Integrated Visualization**: Smooth rendering at 20+ FPS with all data streams

## Usage Examples

### Complete System Launch

```bash
# Launch GPS-VIO calibration and Earth visualization system
ros2 launch geodetic_points integrated_gps_vio_earth_system.launch.py

# Monitor time synchronization quality
ros2 topic echo /sync/time_offset
ros2 topic echo /sync/quality_metrics

# Monitor spatial calibration progress and quality  
ros2 topic echo /calibration/transform_earth_odom
ros2 topic echo /calibration/quality

# View TF tree to confirm earth->odom relationship
ros2 run tf2_tools view_frames
```

### Manual Calibration Process

```bash
# Step 1: Start time synchronization
ros2 run geodetic_points gps_odom_time_sync_node

# Step 2: Collect calibration data (drive with GPS and VIO active)
# Wait for "Sufficient motion detected" message

# Step 3: Run spatial calibration
ros2 run geodetic_points odom_earth_calibration_node

# Step 4: Publish TF relationship once calibration converges
ros2 run geodetic_points earth_odom_tf_publisher_node

# Step 5: Start coordinate transformation services
ros2 run geodetic_points odom_to_earth_pose_node
ros2 run geodetic_points points_transform_node

# Step 6: Launch visualization
ros2 run geodetic_points vio_earth_marker_node
ros2 run geodetic_points gps_on_globe_node
```

### Offline Calibration Workflow

```bash
# Record GPS and VIO data during calibration drive
ros2 bag record /gps/fix /odom /vio/points3d -o gps_vio_calibration_data

# Run offline time synchronization analysis
python3 scripts/offline_time_sync_analysis.py --bag gps_vio_calibration_data.db3

# Compute spatial calibration from recorded data
python3 scripts/offline_odom_earth_calibration.py \
  --bag gps_vio_calibration_data.db3 \
  --output calibration_results.yaml

# Apply computed calibration parameters
ros2 param load /earth_odom_tf_publisher calibration_results.yaml
ros2 run geodetic_points earth_odom_tf_publisher_node
```

### Development and Debugging

```bash
# Check TF tree structure
ros2 run tf2_tools view_frames

# Verify coordinate transformations
ros2 run tf2_ros tf2_echo earth odom
ros2 run tf2_ros tf2_echo earth base_link

# Monitor data flow rates
ros2 topic hz /gps/fix
ros2 topic hz /odom  
ros2 topic hz /vio/points3d
ros2 topic hz /vio/pose_earth
ros2 topic hz /vio/points3d_earth

# Debug calibration quality
ros2 topic echo /calibration/quality | grep -E "(rms_error|inlier_ratio|convergence)"
```

## Integration with Existing System

### Compatibility Requirements

- **GPS Globe Visualization**: Maintain full backward compatibility with existing `gps_on_globe_node.py`
- **Frame Consistency**: Reuse existing `earth` frame for GPS data, extend to support VIO data in same frame
- **TF Tree Extension**: Non-disruptive addition of `earth -> odom` relationship to existing TF structure  
- **Visualization Assets**: Reuse existing globe mesh, textures, and RViz configurations
- **Parameter Preservation**: Maintain existing GPS visualization parameters while adding VIO-specific options

### Data Flow Integration Points

- **GPS Processing**: Leverage existing GPS-to-ECEF conversion in `gps_on_globe_node.py`
- **Marker System**: Extend existing marker publishing to include VIO trajectory and point cloud markers
- **Camera Control**: Enhance existing camera tracking to work with both GPS and VIO trajectories
- **Debug System**: Integrate VIO calibration diagnostics with existing debug output framework

### Extension Points

The calibrated GPS-VIO system enables future enhancements:
- **Multi-Robot Coordination**: Share calibration parameters between multiple robots
- **SLAM Integration**: Transform SLAM maps to Earth coordinates for global localization
- **Sensor Fusion**: Integrate additional sensors (LiDAR, cameras) using established TF relationships
- **Export Capabilities**: Generate georeferenced datasets (KML, GeoJSON, PLY) with accurate Earth coordinates
- **Offline Analysis**: Post-process recorded data with improved calibration algorithms

## Dependencies

### Required Libraries

```yaml
Core Python Dependencies:
  - rclpy >= 3.0 (ROS 2 Python client)
  - numpy >= 1.20 (numerical computing)
  - scipy >= 1.7 (optimization, spatial transforms, Kabsch alignment)
  - pyproj >= 3.0 (geodetic coordinate transformations)
  - sensor_msgs_py (point cloud utilities)

Time Synchronization:
  - scikit-learn (robust regression for time model fitting)
  - matplotlib (optional, for time sync diagnostic plots)

Spatial Calibration:
  - scikit-spatial (geometric algorithms, RANSAC implementation)
  - transforms3d (alternative to scipy for 3D transformations)

Optional Performance Dependencies:
  - numba (JIT compilation for coordinate transformations)
  - open3d (advanced point cloud processing and visualization)
  - filterpy (Kalman filtering for calibration smoothing)
```

### ROS 2 Packages

```yaml
Core ROS 2 Packages:
  - geometry_msgs (pose, transform messages)
  - sensor_msgs (point cloud, GPS messages)  
  - nav_msgs (odometry messages)
  - visualization_msgs (marker arrays)
  - tf2_ros (coordinate frame transformations)
  - std_msgs (basic message types)

Optional ROS 2 Packages:
  - message_filters (time synchronization utilities)
  - rosbag2_py (offline data processing)
  - diagnostic_msgs (system health monitoring)
```

## Maintenance & Updates

### Monitoring Points

- **Time Synchronization Health**:
  - Cross-correlation quality between GPS and VIO motion
  - Time drift model stability over extended operation
  - GPS outage recovery and VIO reset handling

- **Calibration Parameter Stability**:
  - Transformation consistency across multiple calibration sessions  
  - Calibration degradation detection due to sensor changes
  - Environmental factor impacts (temperature, vibration)

- **System Performance Metrics**:
  - TF transformation computational load and latency
  - Point cloud processing throughput and memory usage
  - Visualization frame rate under varying data loads

- **Coordinate Accuracy Validation**:
  - Periodic GPS-VIO alignment quality checks
  - Ground truth validation when available
  - Consistency checks between independent calibration runs

### Update Procedures

1. **Calibration Parameter Updates**:
   - Regular recalibration based on system usage patterns
   - Environmental condition-based parameter adjustment
   - Sensor hardware change detection and recalibration triggers

2. **Performance Optimization**:
   - Point cloud downsampling algorithm improvements
   - TF lookup caching strategies for high-frequency operations
   - Memory management optimization for long-running sessions

3. **Algorithm Enhancements**:
   - Improved time synchronization algorithms (e.g., Kalman filtering)
   - Robust spatial calibration methods (e.g., weighted RANSAC)
   - Integration of newer geodetic computation libraries

4. **Compatibility Maintenance**:
   - ROS 2 version compatibility updates
   - Dependency version management and security updates
   - Backward compatibility preservation for existing configurations

## Summary

This comprehensive GPS-VIO calibration and Earth coordinate integration plan provides a structured approach to establishing the missing `earth -> odom` coordinate relationship. The system enables seamless transformation of VIO trajectories and point clouds to Earth coordinates, creating a unified multi-sensor geospatial visualization platform that extends the existing GPS globe visualization with rich VIO data including both position and orientation information.

**Key Achievements**:
- Bridge the gap between local VIO coordinates (`odom -> base_link`) and global Earth coordinates (`earth` frame)
- Enable real-time transformation of VIO poses and point clouds to Earth frame via TF
- Integrate VIO trajectory visualization with existing GPS globe display
- Provide robust calibration and time synchronization for long-term stability
- Maintain full backward compatibility with existing GPS visualization system