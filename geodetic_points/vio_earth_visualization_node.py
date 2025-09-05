#!/usr/bin/env python3
"""
VIO-Earth Visualization Node (Merged Transform + Visualization)

This node combines the functionality of points_transform_node and multi_sensor_viz_node
to provide unified VIO-to-Earth coordinate transformation and visualization capabilities.

Core Functions:
1. Transform VIO data (odometry, point clouds) from odom frame to earth frame using TF
2. Visualize VIO trajectories and point clouds in Earth coordinates
3. Support real-time VIO data transformation and visualization

Publishers:
  /vio/pose_earth (geometry_msgs/PoseStamped): VIO pose in earth frame
  /vio/points3d_earth (sensor_msgs/PointCloud2): Point cloud in earth frame
  /visualization/vio_markers (visualization_msgs/MarkerArray): VIO trajectory markers
  /visualization/point_cloud_markers (visualization_msgs/MarkerArray): Point cloud markers
  /visualization/sensor_info (std_msgs/String): Sensor status information

Subscribers:
  /odom (nav_msgs/Odometry): VIO odometry estimates
  /vio/points3d (sensor_msgs/PointCloud2): VIO point cloud in odom frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA, Header
from builtin_interfaces.msg import Time
import numpy as np
from collections import deque
import threading
import struct
import time

try:
    from sensor_msgs_py import point_cloud2
    SENSOR_MSGS_PY_AVAILABLE = True
except ImportError:
    SENSOR_MSGS_PY_AVAILABLE = False

try:
    from pyproj import Transformer
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False

try:
    import tf2_ros
    import tf2_geometry_msgs
    from tf2_ros import Buffer, TransformListener
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class VIOEarthVisualizationNode(Node):
    """
    Unified node for VIO-to-Earth transformation and visualization.
    
    This node:
    1. Transforms VIO data from odom to earth frame using calibrated TF
    2. Visualizes VIO trajectories and point clouds in Earth coordinates
    3. Provides real-time sensor status and quality metrics
    """
    
    def __init__(self):
        super().__init__('vio_earth_visualization')
        
        # === PARAMETERS ===
        # Debug logging
        self.declare_parameter('debug_logging', False)
        
        # Frame IDs
        self.declare_parameter('frame_id_earth', 'earth')
        self.declare_parameter('frame_id_odom', 'odom')
        self.declare_parameter('frame_id_base_link', 'base_link')
        
        # Transform parameters
        self.declare_parameter('tf_timeout_ms', 100)
        self.declare_parameter('use_tf_for_transform', True)
        
        # Visualization parameters
        self.declare_parameter('visualization_scale', 1.0)
        self.declare_parameter('max_vio_points', 5000)
        self.declare_parameter('max_cloud_points', 50000)
        
        # VIO visualization
        self.declare_parameter('vio.point_radius', 3.0)
        self.declare_parameter('vio.trail_width', 2.0)
        self.declare_parameter('vio.arrow_length', 10.0)
        self.declare_parameter('vio.arrow_width', 0.5)
        self.declare_parameter('vio.color', [0.1, 1.0, 0.1, 0.9])  # Green
        
        # Point cloud visualization
        self.declare_parameter('cloud.point_size', 1.0)
        self.declare_parameter('cloud.color', [0.1, 0.1, 1.0, 0.7])  # Blue
        self.declare_parameter('cloud.downsample_factor', 10)
        self.declare_parameter('cloud.adaptive_sizing', True)
        self.declare_parameter('cloud.max_display_distance', 1000.0)
        
        # Performance parameters
        self.declare_parameter('visualization_rate_hz', 10.0)
        self.declare_parameter('point_cloud_rate_hz', 2.0)
        
        # Get parameters
        self.debug_logging = self.get_parameter('debug_logging').value
        self.frame_id_earth = self.get_parameter('frame_id_earth').value
        self.frame_id_odom = self.get_parameter('frame_id_odom').value
        self.frame_id_base_link = self.get_parameter('frame_id_base_link').value
        
        self.tf_timeout_ms = self.get_parameter('tf_timeout_ms').value
        self.use_tf = self.get_parameter('use_tf_for_transform').value
        
        self.viz_scale = self.get_parameter('visualization_scale').value
        self.max_vio_points = self.get_parameter('max_vio_points').value
        self.max_cloud_points = self.get_parameter('max_cloud_points').value
        
        # VIO parameters
        self.vio_point_radius = self.get_parameter('vio.point_radius').value
        self.vio_trail_width = self.get_parameter('vio.trail_width').value
        self.vio_arrow_length = self.get_parameter('vio.arrow_length').value
        self.vio_arrow_width = self.get_parameter('vio.arrow_width').value
        vio_color = self.get_parameter('vio.color').value
        self.vio_color = ColorRGBA(r=vio_color[0], g=vio_color[1], b=vio_color[2], a=vio_color[3])
        
        # Point cloud parameters
        self.cloud_point_size = self.get_parameter('cloud.point_size').value
        cloud_color = self.get_parameter('cloud.color').value
        self.cloud_color = ColorRGBA(r=cloud_color[0], g=cloud_color[1], b=cloud_color[2], a=cloud_color[3])
        self.cloud_downsample = self.get_parameter('cloud.downsample_factor').value
        self.cloud_adaptive_sizing = self.get_parameter('cloud.adaptive_sizing').value
        self.cloud_max_distance = self.get_parameter('cloud.max_display_distance').value
        
        self.viz_rate_hz = self.get_parameter('visualization_rate_hz').value
        self.cloud_rate_hz = self.get_parameter('point_cloud_rate_hz').value
        
        # === STATE VARIABLES ===
        # Data storage
        self.vio_poses_earth = deque(maxlen=self.max_vio_points)
        self.current_cloud_earth = None
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # TF setup
        self.tf_buffer = None
        self.tf_listener = None
        if TF2_AVAILABLE and self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            if self.debug_logging:
                self.get_logger().debug('TF2 initialized successfully')
        else:
            if self.use_tf:
                self.get_logger().warn('TF2 not available or disabled - transformation will not work')
        
        # Coordinate transformation
        self.transformer_lla_to_ecef = None
        if PYPROJ_AVAILABLE:
            self.transformer_lla_to_ecef = Transformer.from_crs(
                "EPSG:4979", "EPSG:4978", always_xy=True
            )
        
        # === PUBLISHERS ===
        # Transformed data
        self.pose_earth_pub = self.create_publisher(
            PoseStamped, '/vio/pose_earth', 10
        )
        self.cloud_earth_pub = self.create_publisher(
            PointCloud2, '/vio/points3d_earth', 10
        )
        
        # Visualization
        self.vio_markers_pub = self.create_publisher(
            MarkerArray, '/visualization/vio_markers', 10
        )
        self.cloud_markers_pub = self.create_publisher(
            MarkerArray, '/visualization/point_cloud_markers', 10
        )
        self.info_pub = self.create_publisher(
            String, '/visualization/sensor_info', 10
        )
        
        # === SUBSCRIBERS ===
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 50
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/vio/points3d', self.cloud_callback, 10
        )
        
        if self.debug_logging:
            self.get_logger().debug(
                f'Subscribers created: odom_topic=/odom, cloud_topic=/vio/points3d'
            )
        
        # === TIMERS ===
        self.create_timer(1.0 / self.viz_rate_hz, self.visualization_timer_callback)
        self.create_timer(1.0 / self.cloud_rate_hz, self.cloud_processing_timer_callback)
        
        if self.debug_logging:
            self.get_logger().debug(
                f'Timers created: visualization={self.viz_rate_hz}Hz, '
                f'cloud_processing={self.cloud_rate_hz}Hz'
            )
        
        # Add a timer for periodic log flushing in debug mode
        if self.debug_logging:
            self.create_timer(2.0, self.flush_logs_callback)
        
        # Statistics
        self._odom_count = 0
        self._cloud_count = 0
        self._odom_success_count = 0
        self._cloud_success_count = 0
        self._last_cloud_to_process = None
        self._last_odom_timestamp = None
        self._last_cloud_timestamp = None
        
        # Error throttling
        self._last_odom_error_time = 0
        self._last_cloud_error_time = 0
        self._error_throttle_interval = 5.0  # seconds
        self._tf_available = False
        self._tf_availability_warned = False
        self._tf_check_count = 0
        self._tf_fail_count = 0
        self._startup_time = time.time()
        
        # Missing package warnings
        if not SENSOR_MSGS_PY_AVAILABLE:
            self.get_logger().warn(
                'sensor_msgs_py not available - point cloud processing disabled. '
                'Install with: sudo apt install ros-humble-sensor-msgs-py'
            )
        
        self.get_logger().info('VIO-Earth Visualization Node initialized')
        self.get_logger().info(f'Configuration:')
        self.get_logger().info(f'  - Debug logging: {self.debug_logging}')
        self.get_logger().info(f'  - Earth frame: {self.frame_id_earth}')
        self.get_logger().info(f'  - Odom frame: {self.frame_id_odom}')
        self.get_logger().info(f'  - TF timeout: {self.tf_timeout_ms}ms')
        self.get_logger().info(f'  - Use TF: {self.use_tf}')
        self.get_logger().info(f'  - Visualization rate: {self.viz_rate_hz}Hz')
        self.get_logger().info(f'  - Point cloud rate: {self.cloud_rate_hz}Hz')
        
        # Force flush of initialization logs
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
        
        if self.debug_logging:
            self.get_logger().info('Debug logging enabled - verbose output active')
            self.get_logger().debug('Node startup complete, waiting for data...')
    
    def flush_logs_callback(self):
        """Periodically flush logs to ensure they are written to file."""
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
        # Log flush confirmation every 10th call to avoid spam
        if not hasattr(self, '_flush_count'):
            self._flush_count = 0
        self._flush_count += 1
        if self._flush_count % 10 == 0:
            self.get_logger().debug(f'Log flush #{self._flush_count} completed')
    
    # === VIO CALLBACKS ===
    def odom_callback(self, msg: Odometry):
        """Transform VIO odometry to earth frame."""
        self._odom_count += 1  # Always increment for proper statistics
        
        # Log message reception in debug mode
        if self.debug_logging:
            current_time = time.time()
            time_since_start = current_time - self._startup_time
            self.get_logger().info(
                f'[{time_since_start:.2f}s] Odometry #{self._odom_count} received at '
                f't={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, '
                f'frame_id={msg.header.frame_id}'
            )
            
            # Log time delta between messages
            if self._last_odom_timestamp:
                delta = (msg.header.stamp.sec - self._last_odom_timestamp.sec) + \
                        (msg.header.stamp.nanosec - self._last_odom_timestamp.nanosec) * 1e-9
                if delta > 0.1:  # Log if gap is more than 100ms
                    self.get_logger().debug(f'  Large time gap: {delta:.3f}s since last odometry')
            
            self._last_odom_timestamp = msg.header.stamp
        
        if not self.use_tf or not self.tf_buffer:
            if self.debug_logging:
                self.get_logger().debug('  Skipping: TF disabled or not available')
            return
        
        try:
            # Track TF check attempts
            self._tf_check_count += 1
            
            if self.debug_logging:
                self.get_logger().debug(
                    f'  Checking TF availability #{self._tf_check_count}: '
                    f'{self.frame_id_earth} <- {self.frame_id_odom}'
                )
            
            # Check if transform is available before attempting lookup
            can_transform = self.tf_buffer.can_transform(
                self.frame_id_earth,
                self.frame_id_odom,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_ms / 1000.0)
            )
            
            if not can_transform:
                self._tf_fail_count += 1
                
                # Always log in debug mode
                if self.debug_logging:
                    self.get_logger().debug(
                        f'  TF check FAILED (#{self._tf_fail_count}/{self._tf_check_count}): '
                        f'{self.frame_id_earth} <- {self.frame_id_odom} at '
                        f't={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
                    )
                
                # Throttle warnings using time-based approach
                current_time = time.time()
                if current_time - self._last_odom_error_time > self._error_throttle_interval:
                    self._last_odom_error_time = current_time
                    fail_rate = (self._tf_fail_count / self._tf_check_count * 100) if self._tf_check_count > 0 else 0
                    self.get_logger().warn(
                        f'TF not available: {self.frame_id_earth} <- {self.frame_id_odom} '
                        f'at t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} '
                        f'(fail rate: {fail_rate:.1f}%, {self._tf_fail_count}/{self._tf_check_count} checks)'
                    )
                    if not self._tf_availability_warned:
                        self.get_logger().warn(
                            f'Calibration may not be ready yet. Waiting for {self.frame_id_earth}->{self.frame_id_odom} transform...'
                        )
                        self._tf_availability_warned = True
                return
            
            # Mark TF as available once we successfully get it
            if not self._tf_available:
                self._tf_available = True
                time_to_available = time.time() - self._startup_time
                self.get_logger().info(
                    f'✓ TF transform {self.frame_id_earth} <- {self.frame_id_odom} is now available '
                    f'(after {time_to_available:.2f}s, {self._tf_check_count} checks)'
                )
            
            if self.debug_logging:
                self.get_logger().info('  TF check SUCCESS - proceeding with transform')
            
            # Get transform from odom to earth
            transform = self.tf_buffer.lookup_transform(
                self.frame_id_earth,
                self.frame_id_odom,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_ms / 1000.0)
            )
            
            if self.debug_logging:
                trans = transform.transform.translation
                rot = transform.transform.rotation
                self.get_logger().info(
                    f'  Got TF transform: translation=[{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}], '
                    f'rotation=[{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]'
                )
            
            # Transform the pose - pass Pose object, not PoseStamped
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
            
            # Create PoseStamped with transformed pose
            pose_earth = PoseStamped()
            pose_earth.header.stamp = msg.header.stamp
            pose_earth.header.frame_id = self.frame_id_earth
            pose_earth.pose = transformed_pose
            
            # Debug log the transformed pose
            if self.debug_logging:
                self.get_logger().info(
                    f'  Transformed pose (odom->earth): '
                    f'position=[{transformed_pose.position.x:.3f}, '
                    f'{transformed_pose.position.y:.3f}, '
                    f'{transformed_pose.position.z:.3f}], '
                    f'orientation=[{transformed_pose.orientation.x:.3f}, '
                    f'{transformed_pose.orientation.y:.3f}, '
                    f'{transformed_pose.orientation.z:.3f}, '
                    f'{transformed_pose.orientation.w:.3f}]'
                )
            
            # Publish transformed pose
            self.pose_earth_pub.publish(pose_earth)
            
            self._odom_success_count += 1
            
            if self.debug_logging:
                if self._odom_success_count == 1:
                    self.get_logger().info(
                        f'  ✓ First odometry successfully transformed and published to {self.frame_id_earth} frame'
                    )
                elif self._odom_success_count % 100 == 0:
                    self.get_logger().debug(
                        f'  ✓ Successfully transformed and published odom #{self._odom_success_count} '
                        f'to {self.frame_id_earth} frame'
                    )
                elif self._odom_success_count % 10 == 0:
                    # Log every 10th pose in debug mode for monitoring
                    self.get_logger().info(
                        f'  Odom #{self._odom_success_count}: Published pose at '
                        f'[{pose_earth.pose.position.x:.3f}, {pose_earth.pose.position.y:.3f}, '
                        f'{pose_earth.pose.position.z:.3f}]'
                    )
            
            # Flush logs periodically in debug mode
            if self.debug_logging and self._odom_success_count % 50 == 0:
                import sys
                sys.stdout.flush()
                sys.stderr.flush()
            
            # Store for visualization
            position = np.array([
                pose_earth.pose.position.x,
                pose_earth.pose.position.y,
                pose_earth.pose.position.z
            ])
            
            orientation = pose_earth.pose.orientation
            
            with self.data_lock:
                self.vio_poses_earth.append((position, orientation))
                
                if self.debug_logging and len(self.vio_poses_earth) == 1:
                    self.get_logger().info(
                        f'  First pose stored in trajectory buffer at position '
                        f'[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
                    )
            
        except Exception as e:
            # Time-based throttling for errors
            current_time = time.time()
            if current_time - self._last_odom_error_time > self._error_throttle_interval:
                self._last_odom_error_time = current_time
                self.get_logger().warn(f'Failed to transform odometry: {e}')
                if self.debug_logging:
                    import traceback
                    self.get_logger().debug(f'  Error traceback: {traceback.format_exc()}')
    
    def cloud_callback(self, msg: PointCloud2):
        """Store latest point cloud for processing."""
        self._cloud_count += 1  # Always increment for proper statistics
        
        if self.debug_logging:
            current_time = time.time()
            time_since_start = current_time - self._startup_time
            self.get_logger().debug(
                f'[{time_since_start:.2f}s] Point cloud #{self._cloud_count} received at '
                f't={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, '
                f'frame_id="{msg.header.frame_id}", '
                f'size={msg.width}x{msg.height}'
            )
            
            # Warn about empty frame_id
            if not msg.header.frame_id:
                self.get_logger().debug('  ⚠ Point cloud has empty frame_id!')
        
        self._last_cloud_to_process = msg
    
    def cloud_processing_timer_callback(self):
        """Process and transform point cloud at lower rate."""
        if self.debug_logging and not hasattr(self, '_first_cloud_callback_logged'):
            self._first_cloud_callback_logged = True
            self.get_logger().debug('First cloud processing timer callback triggered')
        
        if self._last_cloud_to_process is None:
            if self.debug_logging and self._cloud_count == 0:
                # Log only once if no clouds received
                if not hasattr(self, '_no_cloud_warned'):
                    self._no_cloud_warned = True
                    self.get_logger().debug('No point clouds received yet')
            return
        
        msg = self._last_cloud_to_process
        self._last_cloud_to_process = None
        
        if self.debug_logging:
            self.get_logger().debug(
                f'Processing cloud from t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )
        
        if not SENSOR_MSGS_PY_AVAILABLE:
            return
        
        if not self.use_tf or not self.tf_buffer:
            return
        
        try:
            # Use actual frame_id from point cloud message instead of assuming odom
            source_frame = msg.header.frame_id
            if not source_frame:
                source_frame = self.frame_id_odom  # Fallback to odom if frame_id is empty
                if not hasattr(self, '_warned_empty_frame_id'):
                    self._warned_empty_frame_id = True
                    self.get_logger().warn(
                        f'Point cloud has empty frame_id, assuming {self.frame_id_odom}'
                    )
                if self.debug_logging:
                    self.get_logger().debug(
                        f'  Using fallback frame_id: {self.frame_id_odom}'
                    )
            elif self.debug_logging:
                self.get_logger().debug(f'  Source frame: "{source_frame}"')
            
            # Check if transform is available
            if self.debug_logging:
                self.get_logger().debug(
                    f'  Checking cloud TF: {self.frame_id_earth} <- {source_frame}'
                )
            
            can_transform = self.tf_buffer.can_transform(
                self.frame_id_earth,
                source_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_ms / 1000.0)
            )
            
            if not can_transform:
                if self.debug_logging:
                    self.get_logger().debug(
                        f'  Cloud TF check FAILED: {self.frame_id_earth} <- {source_frame}'
                    )
                
                # Throttle warnings
                current_time = time.time()
                if current_time - self._last_cloud_error_time > self._error_throttle_interval:
                    self._last_cloud_error_time = current_time
                    self.get_logger().warn(
                        f'TF not available for point cloud: {self.frame_id_earth} <- {source_frame} '
                        f'at t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
                    )
                return
            
            # Get transform from source frame to earth
            transform = self.tf_buffer.lookup_transform(
                self.frame_id_earth,
                source_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_ms / 1000.0)
            )
            
            # Extract points
            points_odom = self.extract_xyz_from_pointcloud(msg)
            if points_odom.shape[0] == 0:
                if self.debug_logging:
                    self.get_logger().debug('  No valid points extracted from cloud')
                return
            
            if self.debug_logging:
                self.get_logger().debug(f'  Extracted {points_odom.shape[0]} points from cloud')
            
            # Downsample
            if self.cloud_downsample > 1:
                points_odom = points_odom[::self.cloud_downsample, :]
            
            # Limit points
            if points_odom.shape[0] > self.max_cloud_points:
                indices = np.random.choice(points_odom.shape[0], self.max_cloud_points, replace=False)
                points_odom = points_odom[indices, :]
            
            # Transform points to earth frame
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            if SCIPY_AVAILABLE:
                rotation = R.from_quat([rot.x, rot.y, rot.z, rot.w])
                R_matrix = rotation.as_matrix()
            else:
                # Simple quaternion to matrix conversion
                R_matrix = self.quaternion_to_matrix(rot.x, rot.y, rot.z, rot.w)
            
            t_vector = np.array([trans.x, trans.y, trans.z])
            
            # Transform points
            points_earth = (R_matrix @ points_odom.T).T + t_vector
            
            # Create and publish earth frame point cloud
            cloud_earth = self.create_pointcloud2(points_earth, msg.header.stamp, self.frame_id_earth)
            self.cloud_earth_pub.publish(cloud_earth)
            
            self._cloud_success_count += 1
            
            if self.debug_logging:
                self.get_logger().debug(
                    f'  ✓ Successfully transformed {points_earth.shape[0]} cloud points '
                    f'to {self.frame_id_earth} frame (success #{self._cloud_success_count})'
                )
            
            # Store for visualization
            with self.data_lock:
                self.current_cloud_earth = points_earth
                
                if self.debug_logging and self._cloud_success_count == 1:
                    self.get_logger().info(
                        f'  First point cloud stored with {len(points_earth)} points'
                    )
            
        except Exception as e:
            # Time-based throttling for errors
            current_time = time.time()
            if current_time - self._last_cloud_error_time > self._error_throttle_interval:
                self._last_cloud_error_time = current_time
                self.get_logger().warn(f'Failed to transform point cloud: {e}')
                if self.debug_logging:
                    import traceback
                    self.get_logger().debug(f'  Cloud error traceback: {traceback.format_exc()}')
    
    # === COORDINATE CONVERSION ===
    def quaternion_to_matrix(self, x, y, z, w):
        """Convert quaternion to rotation matrix (fallback when scipy unavailable)."""
        # Normalize
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ])
        
        return R
    
    # === VISUALIZATION ===
    def visualization_timer_callback(self):
        """Publish all visualization markers."""
        if self.debug_logging and not hasattr(self, '_first_viz_callback_logged'):
            self._first_viz_callback_logged = True
            self.get_logger().debug('First visualization timer callback triggered')
        
        with self.data_lock:
            # VIO trajectory with orientation
            if len(self.vio_poses_earth) > 0:
                vio_markers = self.create_vio_trajectory_markers()
                self.vio_markers_pub.publish(vio_markers)
            
            # Point cloud
            if self.current_cloud_earth is not None and len(self.current_cloud_earth) > 0:
                cloud_markers = self.create_point_cloud_markers()
                self.cloud_markers_pub.publish(cloud_markers)
            
            # Status info with more diagnostics
            info_msg = String()
            cloud_size = 0 if self.current_cloud_earth is None else len(self.current_cloud_earth)
            tf_status = "OK" if self._tf_available else "Waiting"
            
            # Calculate success rates
            odom_success_rate = (self._odom_success_count / self._odom_count * 100) if self._odom_count > 0 else 0
            cloud_success_rate = (self._cloud_success_count / self._cloud_count * 100) if self._cloud_count > 0 else 0
            
            status_text = (f"VIO: {len(self.vio_poses_earth)} poses | "
                          f"Cloud: {cloud_size} pts | "
                          f"Odom: {self._odom_count} ({odom_success_rate:.0f}% ok) | "
                          f"Clouds: {self._cloud_count} ({cloud_success_rate:.0f}% ok) | "
                          f"TF: {tf_status}")
            
            if self.debug_logging and self._tf_check_count > 0:
                tf_success_rate = ((self._tf_check_count - self._tf_fail_count) / self._tf_check_count * 100)
                status_text += f" ({tf_success_rate:.0f}% success)"
            
            info_msg.data = status_text
            self.info_pub.publish(info_msg)
            
            # Log detailed statistics periodically in debug mode
            if self.debug_logging:
                # Log every 10 seconds
                if not hasattr(self, '_last_stats_log_time'):
                    self._last_stats_log_time = 0
                
                current_time = time.time()
                if current_time - self._last_stats_log_time > 10.0:
                    self._last_stats_log_time = current_time
                    uptime = current_time - self._startup_time
                    
                    self.get_logger().info(
                        f'Statistics after {uptime:.1f}s:\n'
                        f'  - Odometry: {self._odom_count} received, {self._odom_success_count} transformed ({odom_success_rate:.1f}%)\n'
                        f'  - Clouds: {self._cloud_count} received, {self._cloud_success_count} transformed ({cloud_success_rate:.1f}%)\n'
                        f'  - TF checks: {self._tf_check_count} total, {self._tf_fail_count} failed\n'
                        f'  - Trajectory: {len(self.vio_poses_earth)} poses stored\n'
                        f'  - Current cloud: {cloud_size} points'
                    )
    
    def create_vio_trajectory_markers(self):
        """Create VIO trajectory visualization with orientation arrows."""
        marker_array = MarkerArray()
        
        poses = list(self.vio_poses_earth)
        
        # Trail line
        trail_marker = Marker()
        trail_marker.header.frame_id = self.frame_id_earth
        trail_marker.header.stamp = self.get_clock().now().to_msg()
        trail_marker.ns = 'vio_trail'
        trail_marker.id = 0
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.scale.x = self.vio_trail_width * self.viz_scale
        trail_marker.color = self.vio_color
        trail_marker.pose.orientation.w = 1.0
        
        for pos, _ in poses:
            p = Point()
            p.x, p.y, p.z = float(pos[0]), float(pos[1]), float(pos[2])
            trail_marker.points.append(p)
        
        marker_array.markers.append(trail_marker)
        
        # Recent poses with orientation arrows
        recent_poses = poses[-50:] if len(poses) > 50 else poses
        stride = max(1, len(recent_poses) // 20)  # Show max 20 arrows
        
        for i, (pos, orientation) in enumerate(recent_poses[::stride]):
            # Position sphere
            sphere = Marker()
            sphere.header.frame_id = self.frame_id_earth
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'vio_points'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            
            sphere.scale.x = self.vio_point_radius * self.viz_scale
            sphere.scale.y = self.vio_point_radius * self.viz_scale
            sphere.scale.z = self.vio_point_radius * self.viz_scale
            
            sphere.color = self.vio_color
            marker_array.markers.append(sphere)
            
            # Orientation arrow
            arrow = Marker()
            arrow.header.frame_id = self.frame_id_earth
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'vio_arrows'
            arrow.id = i * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.pose.position.x = float(pos[0])
            arrow.pose.position.y = float(pos[1])
            arrow.pose.position.z = float(pos[2])
            arrow.pose.orientation = orientation
            
            arrow.scale.x = self.vio_arrow_length * self.viz_scale
            arrow.scale.y = self.vio_arrow_width * self.viz_scale
            arrow.scale.z = self.vio_arrow_width * self.viz_scale
            
            arrow.color = ColorRGBA(
                r=self.vio_color.r * 0.8,
                g=self.vio_color.g * 0.8,
                b=self.vio_color.b * 0.8,
                a=self.vio_color.a
            )
            
            marker_array.markers.append(arrow)
        
        return marker_array
    
    def create_point_cloud_markers(self):
        """Create point cloud visualization markers."""
        marker_array = MarkerArray()
        
        points = self.current_cloud_earth
        if points is None or len(points) == 0:
            return marker_array
        
        # Adaptive downsampling based on distance
        if self.cloud_adaptive_sizing:
            # Calculate distance from origin (camera)
            distances = np.linalg.norm(points, axis=1)
            
            # Create distance-based bins
            close_mask = distances < self.cloud_max_distance * 0.3
            medium_mask = (distances >= self.cloud_max_distance * 0.3) & (distances < self.cloud_max_distance * 0.7)
            far_mask = (distances >= self.cloud_max_distance * 0.7) & (distances < self.cloud_max_distance)
            
            # Sample differently for each distance range
            close_points = points[close_mask][::1]  # Keep all close points
            medium_points = points[medium_mask][::5]  # Sample medium points
            far_points = points[far_mask][::20]  # Heavily sample far points
            
            # Combine available point buckets
            buckets = [pts for pts in (close_points, medium_points, far_points) if pts.size > 0]
            sampled_points = np.vstack(buckets) if buckets else np.array([])
        else:
            # Simple uniform sampling
            max_display = min(len(points), 10000)
            stride = max(1, len(points) // max_display)
            sampled_points = points[::stride]
        
        if len(sampled_points) == 0:
            return marker_array
        
        # Create sphere list marker
        cloud_marker = Marker()
        cloud_marker.header.frame_id = self.frame_id_earth
        cloud_marker.header.stamp = self.get_clock().now().to_msg()
        cloud_marker.ns = 'point_cloud'
        cloud_marker.id = 0
        cloud_marker.type = Marker.SPHERE_LIST
        cloud_marker.action = Marker.ADD
        
        cloud_marker.scale.x = self.cloud_point_size * self.viz_scale
        cloud_marker.scale.y = self.cloud_point_size * self.viz_scale
        cloud_marker.scale.z = self.cloud_point_size * self.viz_scale
        
        cloud_marker.color = self.cloud_color
        cloud_marker.pose.orientation.w = 1.0
        
        for point in sampled_points:
            p = Point()
            p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
            cloud_marker.points.append(p)
        
        marker_array.markers.append(cloud_marker)
        
        return marker_array
    
    # === UTILITY FUNCTIONS ===
    def extract_xyz_from_pointcloud(self, cloud_msg):
        """Extract XYZ coordinates from PointCloud2 message."""
        try:
            points_gen = point_cloud2.read_points(
                cloud_msg, skip_nans=True, field_names=('x', 'y', 'z')
            )
            return np.array([[p[0], p[1], p[2]] for p in points_gen], dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f'Failed to extract points: {e}')
            return np.array([]).reshape(0, 3)
    
    def create_pointcloud2(self, points, stamp, frame_id):
        """Create PointCloud2 message from numpy array of points."""
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        
        # Create point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = fields
        cloud.point_step = 12  # 3 floats * 4 bytes
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = np.asarray(points, np.float32).tobytes()
        
        return cloud


def main(args=None):
    """Main entry point for VIO-Earth visualization node."""
    rclpy.init(args=args)
    
    try:
        node = VIOEarthVisualizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()