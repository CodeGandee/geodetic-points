#!/usr/bin/env python3
"""
GPS-VIO Calibration Node (Fixed Version with Enhanced Logging and TF Publishing)

This node performs complete GPS-VIO calibration by combining time synchronization
and spatial calibration into a unified process. It estimates both temporal alignment
(clock offset and drift) and spatial transformation (rotation and translation) between
VIO local coordinates and ECEF (Earth-Centered Earth-Fixed) global coordinates.

Core Functions:
1. Time Synchronization: Estimates affine time model t_corrected = a * t_gps + b
2. Spatial Calibration: Computes rigid transform T_earth_odom via Kabsch algorithm
3. TF Publishing: Publishes earth -> odom transform for coordinate transformations

Publishers:
  /tf_static (tf2_msgs/TFMessage): Static transform earth -> odom (with TRANSIENT_LOCAL QoS)
  /calibration/transform_earth_odom (geometry_msgs/TransformStamped): Calibrated transform
  /calibration/quality (std_msgs/Float64): Calibration quality (RMS error in meters)
  /sync/time_difference (std_msgs/Float64): Current GPS-VIO time difference in seconds
  /sync/clock_drift_rate (std_msgs/Float64): Clock drift rate in ppm (parts per million)
  /sync/time_model (std_msgs/Float64MultiArray): Time sync model parameters [a, b]
  /sync/residual_error (std_msgs/Float64): Current alignment error in meters

Subscribers:
  /gps/fix (sensor_msgs/NavSatFix): GPS position fixes
  /odom (nav_msgs/Odometry): VIO odometry estimates
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Float64MultiArray, Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from collections import deque
import threading
import os
import json
from datetime import datetime

try:
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import minimize_scalar
    from scipy.interpolate import interp1d
    from scipy.stats import linregress
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    from pyproj import Transformer
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False


class GPSVIOCalibrationNode(Node):
    """
    Unified GPS-VIO calibration node combining time sync and spatial calibration.
    
    This node performs:
    1. Real-time time synchronization between GPS and VIO streams
    2. Rigid body calibration to compute earth -> odom transformation
    3. Continuous quality monitoring and transform publishing
    """
    
    def __init__(self):
        super().__init__('gps_vio_calibration')
        
        # === PARAMETERS ===
        # Time sync parameters (with improved defaults)
        self.declare_parameter('time_sync.buffer_seconds', 60.0)  # Increased from 15.0
        self.declare_parameter('time_sync.max_offset_search', 5.0)  # Increased from 2.0
        self.declare_parameter('time_sync.smoothing_alpha', 0.05)
        self.declare_parameter('time_sync.drift_estimation_window', 50)
        self.declare_parameter('time_sync.min_motion_threshold', 0.05)  # Reduced from 0.1
        
        # Spatial calibration parameters (with improved defaults)
        self.declare_parameter('calibration.window_size', 100)
        self.declare_parameter('calibration.min_motion_threshold', 0.05)  # Reduced from 0.2
        self.declare_parameter('calibration.outlier_threshold', 3.0)
        self.declare_parameter('calibration.auto_publish_transform', True)
        self.declare_parameter('calibration.convergence_threshold', 0.5)  # Relaxed from 0.01
        self.declare_parameter('calibration.max_error', 10.0)
        self.declare_parameter('calibration.min_pairs', 5)  # Reduced from 10
        self.declare_parameter('calibration.publish_before_convergence', True)  # New parameter
        
        # Frame IDs
        self.declare_parameter('frame_id_earth', 'earth')
        self.declare_parameter('frame_id_odom', 'odom')
        
        # Debug/logging parameters
        self.declare_parameter('debug_logging', False)
        self.declare_parameter('enable_file_logging', True)
        self.declare_parameter('log_interval_seconds', 5.0)
        
        # Get parameters
        self.buffer_seconds = self.get_parameter('time_sync.buffer_seconds').value
        self.max_offset_search = self.get_parameter('time_sync.max_offset_search').value
        self.alpha = self.get_parameter('time_sync.smoothing_alpha').value
        self.drift_window = self.get_parameter('time_sync.drift_estimation_window').value
        self.min_motion_sync = self.get_parameter('time_sync.min_motion_threshold').value
        
        self.window_size = self.get_parameter('calibration.window_size').value
        self.min_motion_calib = self.get_parameter('calibration.min_motion_threshold').value
        self.outlier_thresh = self.get_parameter('calibration.outlier_threshold').value
        self.auto_publish = self.get_parameter('calibration.auto_publish_transform').value
        self.convergence_thresh = self.get_parameter('calibration.convergence_threshold').value
        self.max_error = self.get_parameter('calibration.max_error').value
        self.min_pairs = self.get_parameter('calibration.min_pairs').value
        self.publish_before_convergence = self.get_parameter('calibration.publish_before_convergence').value
        
        self.frame_id_earth = self.get_parameter('frame_id_earth').value
        self.frame_id_odom = self.get_parameter('frame_id_odom').value
        
        self.debug_logging = self.get_parameter('debug_logging').value
        self.enable_file_logging = self.get_parameter('enable_file_logging').value
        self.log_interval = self.get_parameter('log_interval_seconds').value
        
        # === STATE VARIABLES ===
        # Time sync state
        self.vio_buffer = deque()  # For time sync interpolation
        self.gps_corrections = deque(maxlen=self.drift_window)
        self.time_a = 1.0  # drift parameter
        self.time_b = 0.0  # offset parameter
        self.last_sync_position = None
        
        # Spatial calibration state
        self.sync_pairs = deque(maxlen=self.window_size)
        self.current_transform = np.eye(4)
        self.calibration_quality = float('inf')
        self.calibration_converged = False
        self.transform_published = False  # Track if we've published at least once
        
        # Thread safety
        self.time_sync_lock = threading.Lock()
        self.calibration_lock = threading.Lock()
        
        # Interpolation cache
        self._interp_fn = None
        self._interp_last_len = 0
        
        # Statistics
        self.interpolation_misses = 0
        self.interpolation_hits = 0
        
        # File logging setup
        self.log_file = None
        self.calibration_log_file = None
        if self.enable_file_logging:
            self.setup_file_logging()
        
        # === COORDINATE TRANSFORMATION ===
        if PYPROJ_AVAILABLE:
            self.transformer_lla_to_ecef = Transformer.from_crs(
                "EPSG:4979", "EPSG:4978", always_xy=True
            )
        else:
            self.get_logger().warn("pyproj not available, using simplified coordinate conversion")
        
        # === PUBLISHERS ===
        # Create QoS profile for static transforms (TRANSIENT_LOCAL)
        static_tf_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_tf_qos)
        self.transform_pub = self.create_publisher(
            TransformStamped, '/calibration/transform_earth_odom', 10
        )
        self.quality_pub = self.create_publisher(Float64, '/calibration/quality', 10)
        
        # Time sync publishers (redesigned for clarity)
        self.time_difference_pub = self.create_publisher(Float64, '/sync/time_difference', 10)
        self.clock_drift_pub = self.create_publisher(Float64, '/sync/clock_drift_rate', 10)
        self.time_model_pub = self.create_publisher(Float64MultiArray, '/sync/time_model', 10)
        self.sync_error_pub = self.create_publisher(Float64, '/sync/residual_error', 10)
        
        # === SUBSCRIBERS ===
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )
        self.vio_sub = self.create_subscription(
            Odometry, '/odom', self.vio_callback, 50
        )
        
        # === TIMERS ===
        # Separate timers for time sync and calibration
        self.create_timer(0.5, self.time_sync_timer_callback)  # 2 Hz
        self.create_timer(2.0, self.calibration_timer_callback)  # 0.5 Hz
        self.create_timer(1.0, self.time_sync_status_callback)  # 1 Hz for status
        self.create_timer(10.0, self.statistics_callback)  # 0.1 Hz for stats
        
        # File logging timer
        if self.enable_file_logging:
            self.create_timer(self.log_interval, self.file_logging_callback)
        
        self.get_logger().info('GPS-VIO Calibration Node initialized (Fixed Version)')
        self.get_logger().info(f'Earth frame: {self.frame_id_earth}, Odom frame: {self.frame_id_odom}')
        self.get_logger().info(f'Key parameters: buffer={self.buffer_seconds}s, convergence={self.convergence_thresh}m, min_pairs={self.min_pairs}')
        if not SCIPY_AVAILABLE:
            self.get_logger().warn('SciPy not available - using simplified algorithms')
    
    # === VIO CALLBACK ===
    def vio_callback(self, msg: Odometry):
        """Store VIO odometry for time synchronization and calibration."""
        t = self.stamp_to_sec(msg.header.stamp)
        pos = msg.pose.pose.position
        position = np.array([pos.x, pos.y, pos.z])
        
        with self.time_sync_lock:
            # Store before logging to ensure we have the data
            self.vio_buffer.append((t, position))
            
            # Log VIO push with window info
            if self.debug_logging and len(self.vio_buffer) > 1:
                self.get_logger().debug(
                    f"VIO push t={t:.3f} pos=[{position[0]:.3f} {position[1]:.3f} {position[2]:.3f}] "
                    f"len={len(self.vio_buffer)} window=[{self.vio_buffer[0][0]:.3f}, {self.vio_buffer[-1][0]:.3f}]"
                )
            
            # Prune old data
            t_cutoff = t - self.buffer_seconds
            initial_len = len(self.vio_buffer)
            while self.vio_buffer and self.vio_buffer[0][0] < t_cutoff:
                self.vio_buffer.popleft()
            
            # Log cleanup if data was removed
            if self.debug_logging and initial_len != len(self.vio_buffer):
                pruned = initial_len - len(self.vio_buffer)
                if len(self.vio_buffer) > 1:
                    self.get_logger().debug(
                        f"VIO cleanup: removed {pruned} old samples, "
                        f"new window=[{self.vio_buffer[0][0]:.3f}, {self.vio_buffer[-1][0]:.3f}] len={len(self.vio_buffer)}"
                    )
    
    # === GPS CALLBACK ===
    def gps_callback(self, msg: NavSatFix):
        """Process GPS fix for both time sync and calibration."""
        if len(self.vio_buffer) < 2:
            if self.debug_logging:
                self.get_logger().warn(f"GPS callback: VIO buffer too small ({len(self.vio_buffer)} < 2), skipping")
            else:
                self.get_logger().warn(f"GPS callback: VIO buffer too small ({len(self.vio_buffer)} < 2)")
            return
        
        # Convert GPS to ECEF coordinates
        gps_ecef = self.gps_to_ecef(msg)
        if gps_ecef is None:
            self.get_logger().error("GPS callback: Failed to convert GPS to ECEF")
            return
        
        t_gps_ros = self.stamp_to_sec(msg.header.stamp)
        
        # === TIME SYNCHRONIZATION ===
        with self.time_sync_lock:
            # Find optimal time offset
            if self.debug_logging:
                t_nominal = self.time_a * t_gps_ros + self.time_b
                self.get_logger().debug(
                    f"Time sync input: t_gps_ros={t_gps_ros:.6f}, t_nominal={t_nominal:.6f}, "
                    f"current model: a={self.time_a:.8f}, b={self.time_b:.6f}"
                )
            
            best_delta, best_error = self.optimize_time_offset(t_gps_ros, gps_ecef)
            
            if best_error < float('inf'):
                # Update time model
                old_b = self.time_b
                implied_b = self.time_b + best_delta
                self.time_b = (1.0 - self.alpha) * self.time_b + self.alpha * implied_b
                
                # Store correction for drift estimation
                t_corrected = self.time_a * t_gps_ros + self.time_b
                self.gps_corrections.append((t_gps_ros, t_corrected))
                
                # Log time model update
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Time model update: old_b={old_b:.6f}, implied_b={implied_b:.6f}, "
                        f"new_b={self.time_b:.6f} (alpha={self.alpha})"
                    )
                
                # Update drift parameter
                if len(self.gps_corrections) >= 10:
                    self.update_drift_parameter()
                
                # Calculate actual time difference for this measurement
                time_diff = t_corrected - t_gps_ros  # VIO time - GPS time
                
                self.get_logger().info(
                    f"Time sync updated: best_delta={best_delta:.4f}s, error={best_error:.3f}m, "
                    f"time_diff={time_diff:.4f}s, a={self.time_a:.8f}, b={self.time_b:.6f}"
                )
                
                # Publish time sync metrics
                self.publish_time_sync_metrics(best_error, t_gps_ros, time_diff)
        
        # === SPATIAL CALIBRATION ===
        # Use synchronized timestamp to get VIO position
        t_vio_corrected = self.time_a * t_gps_ros + self.time_b
        
        if self.debug_logging:
            self.get_logger().debug(
                f"Spatial calibration: attempting VIO interpolation at t_corrected={t_vio_corrected:.6f}"
            )
        
        vio_pos = self.interpolate_vio(t_vio_corrected)
        
        if vio_pos is not None:
            # Check motion requirement for calibration
            with self.calibration_lock:
                if len(self.sync_pairs) == 0:
                    # First pair, always add
                    self.sync_pairs.append((vio_pos, gps_ecef))
                    self.get_logger().info(
                        f"First pair added: vio=[{vio_pos[0]:.3f} {vio_pos[1]:.3f} {vio_pos[2]:.3f}] "
                        f"gps=[{gps_ecef[0]:.3f} {gps_ecef[1]:.3f} {gps_ecef[2]:.3f}] "
                        f"total_pairs={len(self.sync_pairs)}"
                    )
                else:
                    motion = np.linalg.norm(vio_pos - self.sync_pairs[-1][0])
                    if motion > self.min_motion_calib:
                        # Store synchronized pair (vio in odom frame, gps in ECEF/earth frame)
                        self.sync_pairs.append((vio_pos, gps_ecef))
                        self.get_logger().info(
                            f"Pair added: vio=[{vio_pos[0]:.3f} {vio_pos[1]:.3f} {vio_pos[2]:.3f}] "
                            f"gps=[{gps_ecef[0]:.3f} {gps_ecef[1]:.3f} {gps_ecef[2]:.3f}] "
                            f"motion={motion:.3f}m, total_pairs={len(self.sync_pairs)}"
                        )
                    elif self.debug_logging:
                        self.get_logger().debug(
                            f"Pair skipped: insufficient motion ({motion:.3f}m < {self.min_motion_calib:.3f}m)"
                        )
        else:
            self.interpolation_misses += 1  # Already incremented in interpolate_vio, but for clarity
            if self.debug_logging:
                with self.time_sync_lock:
                    if len(self.vio_buffer) > 1:
                        times = [t for t, _ in self.vio_buffer]
                        self.get_logger().debug(
                            f"VIO interpolation failed: t_query={t_vio_corrected:.3f} "
                            f"outside VIO buffer [{times[0]:.3f}, {times[-1]:.3f}], buffer_len={len(self.vio_buffer)}"
                        )
    
    # === COORDINATE CONVERSION ===
    def gps_to_ecef(self, gps_msg):
        """Convert GPS coordinates to ECEF."""
        lat, lon, alt = gps_msg.latitude, gps_msg.longitude, gps_msg.altitude
        
        if PYPROJ_AVAILABLE:
            try:
                x, y, z = self.transformer_lla_to_ecef.transform(lon, lat, alt)
                ecef_point = np.array([x, y, z])
                if self.debug_logging:
                    self.get_logger().debug(f"pyproj ECEF x={x:.3f} y={y:.3f} z={z:.3f}")
            except Exception as e:
                self.get_logger().error(f"Coordinate transformation failed: {e}")
                return None
        else:
            # Simplified ECEF conversion
            if self.debug_logging:
                self.get_logger().warn("pyproj unavailable, using simplified ECEF")
            ecef_point = self.simple_lla_to_ecef(lat, lon, alt)
            if self.debug_logging:
                self.get_logger().debug(
                    f"Simple ECEF x={ecef_point[0]:.3f} y={ecef_point[1]:.3f} z={ecef_point[2]:.3f}"
                )
        
        return ecef_point
    
    def simple_lla_to_ecef(self, lat, lon, alt):
        """Simple LLA to ECEF conversion (fallback when pyproj unavailable)."""
        a = 6378137.0  # WGS84 semi-major axis
        f = 1 / 298.257223563  # WGS84 flattening
        
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        
        e2 = 2 * f - f * f
        N = a / np.sqrt(1 - e2 * np.sin(lat_rad) ** 2)
        
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1 - e2) + alt) * np.sin(lat_rad)
        
        return np.array([x, y, z])
    
    # === TIME SYNCHRONIZATION ===
    def optimize_time_offset(self, t_gps_ros, gps_ecef):
        """Find optimal time offset using spatial alignment."""
        t_nominal = self.time_a * t_gps_ros + self.time_b
        
        if self.debug_logging:
            self.get_logger().debug(
                f"optimize_time_offset entry: t_gps_ros={t_gps_ros:.6f}, t_nominal={t_nominal:.6f}, "
                f"search_range=[{-self.max_offset_search:.2f}, {self.max_offset_search:.2f}]s"
            )
        
        if SCIPY_AVAILABLE:
            # Use SciPy optimization
            def error_function(delta):
                t_candidate = t_nominal + delta
                vio_pos = self.interpolate_vio(t_candidate)
                if vio_pos is None:
                    return 1e9
                # For time sync, use distance between positions after current transform estimate
                # Transform VIO from odom to earth frame using odom -> earth transform
                # current_transform stores earth -> odom, so we need to use the inverse
                R_eo = self.current_transform[:3, :3]
                t_eo = self.current_transform[:3, 3]
                # Compute odom -> earth transform: R_oe = R_eo^T, t_oe = -R_oe * t_eo
                R_oe = R_eo.T
                t_oe = -R_oe @ t_eo
                vio_transformed = R_oe @ vio_pos + t_oe
                return np.linalg.norm(vio_transformed - gps_ecef)
            
            try:
                result = minimize_scalar(
                    error_function,
                    bounds=(-self.max_offset_search, self.max_offset_search),
                    method='bounded'
                )
                if result.success:
                    if self.debug_logging:
                        self.get_logger().info(
                            f"Time offset optimization (scipy): success, delta={result.x:.4f}s, error={result.fun:.3f}m"
                        )
                    else:
                        self.get_logger().info(f"Offset result (scipy): delta={result.x:.4f} error={result.fun:.3f}")
                    return result.x, result.fun
                else:
                    if self.debug_logging:
                        self.get_logger().warn(f"SciPy optimization failed (success=False), falling back to grid search")
                    else:
                        self.get_logger().warn("SciPy optimization failed, falling back to grid search")
            except Exception as e:
                if self.debug_logging:
                    self.get_logger().warn(f"SciPy optimization exception: {e}, falling back to grid search")
                else:
                    self.get_logger().warn(f"SciPy optimization failed: {e}, falling back to grid search")
        
        # Fallback: grid search
        best_delta = 0.0
        best_error = float('inf')
        
        search_range = np.linspace(-self.max_offset_search, self.max_offset_search, 41)  # Increased from 21
        for delta in search_range:
            t_candidate = t_nominal + delta
            vio_pos = self.interpolate_vio(t_candidate)
            if vio_pos is None:
                continue
            
            # For time sync, use distance between positions after current transform estimate
            # Transform VIO from odom to earth frame using odom -> earth transform
            # current_transform stores earth -> odom, so we need to use the inverse
            R_eo = self.current_transform[:3, :3]
            t_eo = self.current_transform[:3, 3]
            # Compute odom -> earth transform: R_oe = R_eo^T, t_oe = -R_oe * t_eo
            R_oe = R_eo.T
            t_oe = -R_oe @ t_eo
            vio_transformed = R_oe @ vio_pos + t_oe
            error = np.linalg.norm(vio_transformed - gps_ecef)
            if error < best_error:
                best_error = error
                best_delta = delta
        
        if self.debug_logging:
            self.get_logger().info(
                f"Time offset optimization (grid search): delta={best_delta:.4f}s, error={best_error:.3f}m, "
                f"tested {len(search_range)} points"
            )
        elif not SCIPY_AVAILABLE:
            self.get_logger().info(f"Offset result (grid): delta={best_delta:.4f} error={best_error:.3f}")
        
        return best_delta, best_error
    
    def interpolate_vio(self, t_query):
        """Interpolate VIO position at query time."""
        if len(self.vio_buffer) < 2:
            return None
        
        times = np.array([t for t, _ in self.vio_buffer])
        positions = np.array([p for _, p in self.vio_buffer])
        
        if t_query < times[0] or t_query > times[-1]:
            self.interpolation_misses += 1
            if self.debug_logging:
                self.get_logger().debug(
                    f"Interp miss: t_query={t_query:.3f} outside [{times[0]:.3f}, {times[-1]:.3f}] (len={len(times)})"
                )
            return None
        
        self.interpolation_hits += 1
        
        if SCIPY_AVAILABLE:
            try:
                # Rebuild interpolator if buffer changed
                if self._interp_fn is None or self._interp_last_len != len(times):
                    self._interp_fn = interp1d(
                        times, positions, axis=0, kind='linear',
                        assume_sorted=True, copy=False
                    )
                    self._interp_last_len = len(times)
                result = self._interp_fn(t_query)
                if self.debug_logging:
                    self.get_logger().debug(
                        f"scipy interpolation success: t_query={t_query:.3f}, "
                        f"result=[{result[0]:.3f}, {result[1]:.3f}, {result[2]:.3f}]"
                    )
                return result
            except Exception as e:
                if self.debug_logging:
                    self.get_logger().debug(f"scipy interp failed ({e}), fallback to manual interpolation")
        
        # Manual linear interpolation
        idx = np.searchsorted(times, t_query)
        if idx == 0:
            if self.debug_logging:
                self.get_logger().debug(f"Manual interp: t_query at start, returning first position")
            return positions[0]
        if idx == len(times):
            if self.debug_logging:
                self.get_logger().debug(f"Manual interp: t_query at end, returning last position")
            return positions[-1]
        
        t1, t2 = times[idx-1], times[idx]
        p1, p2 = positions[idx-1], positions[idx]
        
        if self.debug_logging:
            dt = t2 - t1
            self.get_logger().debug(
                f"Manual interp: idx={idx}/{len(times)}, t1={t1:.3f}, t2={t2:.3f}, dt={dt:.3f}s"
            )
        
        if t2 == t1:
            return p1
        
        w = (t_query - t1) / (t2 - t1)
        return p1 * (1.0 - w) + p2 * w
    
    def update_drift_parameter(self):
        """Estimate clock drift using linear regression."""
        if len(self.gps_corrections) < 10:
            return
        
        times_ros = np.array([t_ros for t_ros, _ in self.gps_corrections])
        times_corrected = np.array([t_corr for _, t_corr in self.gps_corrections])
        
        # Center timestamps to improve numerical stability
        t_mean = np.mean(times_ros)
        times_ros_centered = times_ros - t_mean
        times_corrected_centered = times_corrected - t_mean
        
        if SCIPY_AVAILABLE:
            try:
                slope, intercept, r_value, _, std_err = linregress(times_ros_centered, times_corrected_centered)
                
                # Update with smoothing
                new_a = slope
                # Adjust intercept back from centered coordinates
                new_b = intercept + t_mean * (1.0 - slope)
                self.time_a = (1.0 - self.alpha) * self.time_a + self.alpha * new_a
                self.time_b = (1.0 - self.alpha) * self.time_b + self.alpha * new_b
                
                if self.debug_logging:
                    self.get_logger().info(
                        f"Drift estimation (scipy): new_a={new_a:.8f}, new_b={new_b:.6f}, "
                        f"drift={1e6*(self.time_a-1.0):.1f}ppm, offset={self.time_b:.4f}s, "
                        f"r²={r_value**2:.4f}, std_err={std_err:.6f}, samples={len(times_ros)}"
                    )
                else:
                    self.get_logger().info(
                        f"Drift update: drift={1e6*(self.time_a-1.0):.1f}ppm, offset={self.time_b:.4f}s, "
                        f"r²={r_value**2:.4f}, std_err={std_err:.6f}"
                    )
            except Exception as e:
                if self.debug_logging:
                    self.get_logger().warn(f"Drift estimation (scipy) failed: {e}, keeping current values")
                else:
                    self.get_logger().warn(f"Drift estimation failed: {e}")
        else:
            # Simple slope calculation
            if len(times_ros_centered) >= 2:
                dt_ros = times_ros_centered[-1] - times_ros_centered[0]
                dt_corr = times_corrected_centered[-1] - times_corrected_centered[0]
                if dt_ros > 1e-6:
                    slope = dt_corr / dt_ros
                    old_a = self.time_a
                    self.time_a = (1.0 - self.alpha) * self.time_a + self.alpha * slope
                    if self.debug_logging:
                        self.get_logger().info(
                            f"Drift estimation (simple): slope={slope:.8f}, old_a={old_a:.8f}, "
                            f"new_a={self.time_a:.8f}, dt_ros={dt_ros:.3f}s, samples={len(times_ros)}"
                        )
    
    # === SPATIAL CALIBRATION ===
    def calibration_timer_callback(self):
        """Periodic spatial calibration computation."""
        with self.calibration_lock:
            num_pairs = len(self.sync_pairs)
            
            if self.debug_logging:
                self.get_logger().debug(
                    f"Calibration timer tick: pairs={num_pairs}/{self.window_size} "
                    f"(min_required={self.min_pairs}), converged={self.calibration_converged}"
                )
            
            if num_pairs < self.min_pairs:
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Calibration skipped: insufficient pairs ({num_pairs} < {self.min_pairs}), "
                        f"waiting for more GPS-VIO synchronized data"
                    )
                else:
                    self.get_logger().debug(f"Calibration skipped: not enough pairs ({num_pairs} < {self.min_pairs})")
                return
            
            # Extract VIO positions (in odom frame) and GPS positions (in earth/ECEF frame)
            vio_positions = np.array([pair[0] for pair in self.sync_pairs])
            gps_positions = np.array([pair[1] for pair in self.sync_pairs])
            
            # Perform rigid alignment to find transform from odom to earth
            try:
                R_matrix, t_vector, rms_error = self.rigid_alignment(vio_positions, gps_positions)
                
                self.get_logger().info(
                    f"Rigid alignment completed: rms={rms_error:.3f}m (threshold={self.max_error:.3f}m), "
                    f"pairs_used={len(vio_positions)}"
                )
                
                if rms_error < self.max_error:
                    # The rigid_alignment computes transform from odom to earth
                    # But TF expects earth -> odom, so we need the inverse
                    R_earth_odom = R_matrix.T
                    t_earth_odom = -R_earth_odom @ t_vector
                    
                    self.current_transform[:3, :3] = R_earth_odom
                    self.current_transform[:3, 3] = t_earth_odom
                    self.calibration_quality = rms_error
                    
                    if self.debug_logging:
                        det_R = np.linalg.det(R_earth_odom)
                        t_norm = np.linalg.norm(t_earth_odom)
                        # Extract rotation angle from rotation matrix
                        trace_R = np.trace(R_earth_odom)
                        angle_rad = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
                        angle_deg = np.degrees(angle_rad)
                        self.get_logger().debug(
                            f"Transform details: det(R)={det_R:.6f}, |t|={t_norm:.3f}m, "
                            f"rotation_angle={angle_deg:.2f}°, t=[{t_earth_odom[0]:.3f}, {t_earth_odom[1]:.3f}, {t_earth_odom[2]:.3f}]"
                        )
                    
                    # Check convergence
                    was_converged = self.calibration_converged
                    if rms_error < self.convergence_thresh:
                        self.calibration_converged = True
                    
                    self.get_logger().info(
                        f"Convergence check: converged={self.calibration_converged} "
                        f"(rms={rms_error:.3f}m {'<' if self.calibration_converged else '>='} threshold={self.convergence_thresh:.3f}m)"
                    )
                    
                    # Publish results
                    self.publish_calibration_results(R_earth_odom, t_earth_odom, rms_error)
                    
                    if not was_converged and self.calibration_converged:
                        self.get_logger().info(
                            f"CALIBRATION CONVERGED! RMS={rms_error:.3f}m, "
                            f"pairs={len(self.sync_pairs)}"
                        )
                else:
                    self.get_logger().warn(
                        f"Calibration error too high: rms={rms_error:.3f}m > max_error={self.max_error:.3f}m, "
                        f"transform not updated"
                    )
            
            except Exception as e:
                self.get_logger().error(f"Calibration computation failed: {e}", exc_info=self.debug_logging)
    
    def rigid_alignment(self, P, Q):
        """
        Compute rigid alignment between point sets P (VIO) and Q (GPS).
        Returns rotation matrix R, translation vector t, and RMS error.
        """
        if self.debug_logging:
            self.get_logger().debug(f"Align start: P={len(P)} Q={len(Q)}")
        
        if len(P) != len(Q):
            error_msg = f"Point sets size mismatch: P={len(P)}, Q={len(Q)}"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        if len(P) < 3:
            error_msg = f"Need at least 3 corresponding points, got {len(P)}"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        # Remove outliers
        P_clean, Q_clean = self.remove_outliers(P, Q)
        
        if len(P_clean) < 3:
            error_msg = f"Too few points after outlier removal: {len(P_clean)} (original: {len(P)})"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        if SCIPY_AVAILABLE:
            return self.scipy_rigid_alignment(P_clean, Q_clean)
        else:
            return self.simple_rigid_alignment(P_clean, Q_clean)
    
    def remove_outliers(self, P, Q):
        """Remove outlier pairs based on residual distance."""
        if np.allclose(self.current_transform, np.eye(4)):
            # No outlier removal on first iteration
            if self.debug_logging:
                self.get_logger().debug("Outlier removal skipped (initial transform)")
            return P, Q
        
        residuals = []
        # current_transform stores earth -> odom, but we need odom -> earth to transform VIO points
        R_eo = self.current_transform[:3, :3]
        t_eo = self.current_transform[:3, 3]
        # Compute odom -> earth transform: R_oe = R_eo^T, t_oe = -R_oe * t_eo
        R_oe = R_eo.T
        t_oe = -R_oe @ t_eo
        
        for i in range(len(P)):
            # Transform VIO point to earth frame using odom -> earth transform
            p_transformed = R_oe @ P[i] + t_oe
            residual = np.linalg.norm(Q[i] - p_transformed)
            residuals.append(residual)
        
        residuals = np.array(residuals)
        
        # Remove points beyond threshold sigma
        mean_residual = np.mean(residuals)
        std_residual = np.std(residuals)
        
        if std_residual > 1e-6:  # Avoid division by zero
            threshold = mean_residual + self.outlier_thresh * std_residual
            valid_mask = residuals <= threshold
            kept = np.sum(valid_mask)
            if self.debug_logging:
                self.get_logger().debug(
                    f"Outlier removal stats: mean_residual={mean_residual:.3f}m, std={std_residual:.3f}m, "
                    f"threshold={threshold:.3f}m (outlier_thresh={self.outlier_thresh} sigma), "
                    f"kept={kept}/{len(P)} points"
                )
            return P[valid_mask], Q[valid_mask]
        else:
            return P, Q
    
    def scipy_rigid_alignment(self, P, Q):
        """Rigid alignment using SciPy's Rotation.align_vectors."""
        # Center the point sets
        mu_P = P.mean(axis=0)
        mu_Q = Q.mean(axis=0)
        P_centered = P - mu_P
        Q_centered = Q - mu_Q
        
        # Use SciPy for robust rotation estimation
        # align_vectors(A, B) finds R such that R @ B ≈ A
        # We want R @ P ≈ Q (transform VIO to GPS), so parameters should be (Q, P)
        rot_obj, rssd = R.align_vectors(Q_centered, P_centered)
        R_matrix = rot_obj.as_matrix()
        
        if self.debug_logging:
            self.get_logger().debug(
                f"scipy_rigid_alignment: rssd={rssd:.6f}, det(R)={np.linalg.det(R_matrix):.6f}, "
                f"points_aligned={len(P)}"
            )
        
        # Compute translation
        t_vector = mu_Q - R_matrix @ mu_P
        
        # Compute RMS error
        P_aligned = (R_matrix @ P.T).T + t_vector
        residuals = Q - P_aligned
        rms_error = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        
        return R_matrix, t_vector, rms_error
    
    def simple_rigid_alignment(self, P, Q):
        """Simplified rigid alignment using SVD (fallback)."""
        # Center the point sets
        mu_P = P.mean(axis=0)
        mu_Q = Q.mean(axis=0)
        P_centered = P - mu_P
        Q_centered = Q - mu_Q
        
        # Compute cross-covariance matrix
        H = P_centered.T @ Q_centered
        
        # SVD decomposition
        U, S, Vt = np.linalg.svd(H)
        
        # Compute rotation matrix
        R_matrix = Vt.T @ U.T
        
        # Ensure proper rotation (det(R) = 1)
        if np.linalg.det(R_matrix) < 0:
            Vt[-1, :] *= -1
            R_matrix = Vt.T @ U.T
        
        det_R = np.linalg.det(R_matrix)
        if self.debug_logging:
            self.get_logger().debug(
                f"simple_rigid_alignment (SVD): det(R)={det_R:.6f}, "
                f"singular_values={S}, points_aligned={len(P)}"
            )
        
        # Compute translation
        t_vector = mu_Q - R_matrix @ mu_P
        
        # Compute RMS error
        P_aligned = (R_matrix @ P.T).T + t_vector
        residuals = Q - P_aligned
        rms_error = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        
        if self.debug_logging:
            self.get_logger().debug(
                f"simple_rigid_alignment result: rms={rms_error:.3f}m, "
                f"|t|={np.linalg.norm(t_vector):.3f}m"
            )
        
        return R_matrix, t_vector, rms_error
    
    # === PUBLISHING ===
    def publish_time_sync_metrics(self, sync_error, t_gps_current, time_diff_current):
        """Publish time synchronization metrics."""
        # Current time difference (VIO - GPS) in seconds
        time_diff_msg = Float64()
        time_diff_msg.data = time_diff_current
        self.time_difference_pub.publish(time_diff_msg)
        
        # Clock drift rate in parts per million (ppm)
        # drift_rate = (a - 1.0), convert to ppm: 1e6 * (a - 1.0)
        drift_ppm = 1e6 * (self.time_a - 1.0)
        drift_msg = Float64()
        drift_msg.data = drift_ppm
        self.clock_drift_pub.publish(drift_msg)
        
        # Time model parameters [a, b] for advanced users/debugging
        time_model_msg = Float64MultiArray()
        time_model_msg.data = [self.time_a, self.time_b]
        self.time_model_pub.publish(time_model_msg)
        
        # Sync residual error
        error_msg = Float64()
        error_msg.data = sync_error
        self.sync_error_pub.publish(error_msg)
    
    def publish_calibration_results(self, R_matrix, t_vector, rms_error):
        """Publish calibration results including transform and quality metrics."""
        self.get_logger().info(
            f"Publishing calibration results: rms={rms_error:.3f}m, "
            f"transform {self.frame_id_earth} -> {self.frame_id_odom}"
        )
        
        # Publish quality metric
        quality_msg = Float64()
        quality_msg.data = rms_error
        self.quality_pub.publish(quality_msg)
        
        # Create transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.frame_id_earth
        transform_msg.child_frame_id = self.frame_id_odom
        
        # Set translation
        transform_msg.transform.translation.x = float(t_vector[0])
        transform_msg.transform.translation.y = float(t_vector[1])
        transform_msg.transform.translation.z = float(t_vector[2])
        
        # Convert rotation to quaternion
        if SCIPY_AVAILABLE:
            rot_obj = R.from_matrix(R_matrix)
            quat = rot_obj.as_quat()  # [x, y, z, w]
        else:
            quat = self.matrix_to_quaternion(R_matrix)
        
        transform_msg.transform.rotation.x = float(quat[0])
        transform_msg.transform.rotation.y = float(quat[1])
        transform_msg.transform.rotation.z = float(quat[2])
        transform_msg.transform.rotation.w = float(quat[3])
        
        # Publish transform message
        self.transform_pub.publish(transform_msg)
        
        # Decide whether to publish to TF
        should_publish_tf = False
        
        if self.auto_publish:
            if self.calibration_converged:
                should_publish_tf = True
                if self.debug_logging:
                    self.get_logger().info(
                        f"Static TF decision: auto_publish={self.auto_publish}, converged={self.calibration_converged} "
                        f"-> WILL PUBLISH (converged)"
                    )
            elif self.publish_before_convergence and rms_error < self.max_error:
                should_publish_tf = True
                if self.debug_logging:
                    self.get_logger().info(
                        f"Static TF decision: auto_publish={self.auto_publish}, converged={self.calibration_converged}, "
                        f"publish_before_convergence={self.publish_before_convergence}, rms={rms_error:.3f}m < max={self.max_error:.3f}m "
                        f"-> WILL PUBLISH (pre-convergence)"
                    )
            else:
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Static TF decision: NOT publishing (converged={self.calibration_converged}, "
                        f"convergence_threshold={self.convergence_thresh:.3f}m, rms={rms_error:.3f}m)"
                    )
        
        if should_publish_tf:
            tf_msg = TFMessage()
            tf_msg.transforms.append(transform_msg)
            self.tf_static_pub.publish(tf_msg)
            
            if not self.transform_published:
                self.get_logger().info(
                    "Static TF published for FIRST TIME (using TRANSIENT_LOCAL QoS - late subscribers will receive it)"
                )
                self.transform_published = True
            elif self.debug_logging:
                self.get_logger().debug("Static TF re-published (update)")
    
    def matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion (simplified)."""
        trace = np.trace(R)
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2  # S=4*qw
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qw = (R[2,1] - R[1,2]) / S
                qx = 0.25 * S
                qy = (R[0,1] + R[1,0]) / S
                qz = (R[0,2] + R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qw = (R[0,2] - R[2,0]) / S
                qx = (R[0,1] + R[1,0]) / S
                qy = 0.25 * S
                qz = (R[1,2] + R[2,1]) / S
            else:
                S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qw = (R[1,0] - R[0,1]) / S
                qx = (R[0,2] + R[2,0]) / S
                qy = (R[1,2] + R[2,1]) / S
                qz = 0.25 * S
        
        return np.array([qx, qy, qz, qw])
    
    def time_sync_timer_callback(self):
        """Periodic time sync processing (currently unused but kept for future extensions)."""
        pass
    
    def time_sync_status_callback(self):
        """Publish current time sync status at 1Hz."""
        if self.time_a != 1.0 or self.time_b != 0.0:
            # Get current ROS time
            current_time = self.get_clock().now().nanoseconds * 1e-9
            
            # Calculate current time difference
            time_diff = (self.time_a - 1.0) * current_time + self.time_b
            
            if self.debug_logging:
                self.get_logger().debug(
                    f"Status: time_diff={time_diff:.4f}s drift={1e6*(self.time_a-1.0):.1f}ppm "
                    f"a={self.time_a:.8f} b={self.time_b:.6f}"
                )
            
            # Publish current time difference
            time_diff_msg = Float64()
            time_diff_msg.data = time_diff
            self.time_difference_pub.publish(time_diff_msg)
            
            # Publish clock drift rate
            drift_ppm = 1e6 * (self.time_a - 1.0)
            drift_msg = Float64()
            drift_msg.data = drift_ppm
            self.clock_drift_pub.publish(drift_msg)
    
    def statistics_callback(self):
        """Periodic statistics reporting."""
        total_attempts = self.interpolation_hits + self.interpolation_misses
        if total_attempts > 0:
            hit_rate = 100.0 * self.interpolation_hits / total_attempts
            if self.debug_logging:
                self.get_logger().info(
                    f"=== STATISTICS UPDATE ===\n"
                    f"  Interpolation: hits={self.interpolation_hits}, misses={self.interpolation_misses}, "
                    f"hit_rate={hit_rate:.1f}%\n"
                    f"  VIO buffer: size={len(self.vio_buffer)}, max={self.buffer_seconds}s\n"
                    f"  Time sync: a={self.time_a:.8f}, b={self.time_b:.6f}, drift={1e6*(self.time_a-1.0):.1f}ppm"
                )
            else:
                self.get_logger().info(
                    f"Interpolation stats: hits={self.interpolation_hits}, misses={self.interpolation_misses}, "
                    f"hit_rate={hit_rate:.1f}%"
                )
        
        with self.calibration_lock:
            quality_str = f"{self.calibration_quality:.3f}m" if self.calibration_quality != float('inf') else "N/A"
            if self.debug_logging:
                self.get_logger().info(
                    f"  Calibration: pairs={len(self.sync_pairs)}/{self.window_size}, "
                    f"quality={quality_str}, converged={self.calibration_converged}, "
                    f"tf_published={self.transform_published}\n"
                    f"========================="
                )
            else:
                self.get_logger().info(
                    f"Calibration stats: pairs={len(self.sync_pairs)}/{self.window_size}, "
                    f"quality={quality_str}, converged={self.calibration_converged}"
                )
    
    def setup_file_logging(self):
        """Setup file logging to ROS_LOG_DIR."""
        # Get ROS_LOG_DIR from environment or use default
        log_dir = os.environ.get('ROS_LOG_DIR', './log')
        
        # Create log directory if it doesn't exist
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Create timestamp for log files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Create main log file for all data
        log_file_path = os.path.join(log_dir, f'gps_vio_calibration_{timestamp}.log')
        self.log_file = open(log_file_path, 'w')
        self.get_logger().info(f'File logging enabled: {log_file_path}')
        
        # Create calibration quality log file (JSON format for easy parsing)
        calib_log_path = os.path.join(log_dir, f'calibration_quality_{timestamp}.json')
        self.calibration_log_file = open(calib_log_path, 'w')
        self.get_logger().info(f'Calibration quality logging: {calib_log_path}')
        
        # Write headers
        self.log_file.write(f"GPS-VIO Calibration Log Started: {datetime.now().isoformat()}\n")
        self.log_file.write(f"Earth frame: {self.frame_id_earth}, Odom frame: {self.frame_id_odom}\n")
        self.log_file.write(f"Parameters: buffer={self.buffer_seconds}s, convergence={self.convergence_thresh}m, min_pairs={self.min_pairs}\n")
        self.log_file.write("="*80 + "\n")
        self.log_file.flush()
        
        # Initialize calibration log
        self.calibration_log_entries = []
    
    def file_logging_callback(self):
        """Periodic callback to log data to file."""
        if not self.log_file:
            return
        
        try:
            timestamp = datetime.now().isoformat()
            current_time_ros = self.get_clock().now().nanoseconds * 1e-9
            
            # Prepare log entry
            log_entry = {
                'timestamp': timestamp,
                'ros_time': current_time_ros,
                'time_sync': {
                    'a_drift': self.time_a,
                    'b_offset': self.time_b,
                    'drift_ppm': 1e6 * (self.time_a - 1.0),
                    'time_difference': (self.time_a - 1.0) * current_time_ros + self.time_b,
                    'corrections_count': len(self.gps_corrections)
                },
                'calibration': {
                    'pairs_count': len(self.sync_pairs),
                    'quality_rms_m': self.calibration_quality if self.calibration_quality != float('inf') else None,
                    'converged': self.calibration_converged,
                    'transform_published': self.transform_published
                },
                'interpolation': {
                    'hits': self.interpolation_hits,
                    'misses': self.interpolation_misses,
                    'hit_rate': 100.0 * self.interpolation_hits / max(1, self.interpolation_hits + self.interpolation_misses)
                },
                'vio_buffer_size': len(self.vio_buffer)
            }
            
            # Add transform details if available
            if not np.allclose(self.current_transform, np.eye(4)):
                R = self.current_transform[:3, :3]
                t = self.current_transform[:3, 3]
                
                # Extract rotation angles
                trace_R = np.trace(R)
                angle_rad = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
                angle_deg = np.degrees(angle_rad)
                
                log_entry['transform'] = {
                    'translation_m': t.tolist(),
                    'translation_norm_m': float(np.linalg.norm(t)),
                    'rotation_angle_deg': float(angle_deg),
                    'det_R': float(np.linalg.det(R))
                }
            
            # Write to main log file (human-readable format)
            self.log_file.write(f"\n[{timestamp}]\n")
            self.log_file.write(f"Time Sync: drift={log_entry['time_sync']['drift_ppm']:.1f}ppm, "
                              f"offset={log_entry['time_sync']['b_offset']:.4f}s, "
                              f"time_diff={log_entry['time_sync']['time_difference']:.4f}s\n")
            self.log_file.write(f"Calibration: pairs={log_entry['calibration']['pairs_count']}/{self.window_size}, ")
            
            if log_entry['calibration']['quality_rms_m'] is not None:
                self.log_file.write(f"quality={log_entry['calibration']['quality_rms_m']:.3f}m, ")
            else:
                self.log_file.write("quality=N/A, ")
                
            self.log_file.write(f"converged={log_entry['calibration']['converged']}\n")
            
            if 'transform' in log_entry:
                self.log_file.write(f"Transform: translation=[{log_entry['transform']['translation_m'][0]:.3f}, "
                                  f"{log_entry['transform']['translation_m'][1]:.3f}, "
                                  f"{log_entry['transform']['translation_m'][2]:.3f}]m, "
                                  f"rotation={log_entry['transform']['rotation_angle_deg']:.2f}°\n")
            
            self.log_file.write(f"Interpolation: hit_rate={log_entry['interpolation']['hit_rate']:.1f}% "
                              f"(hits={log_entry['interpolation']['hits']}, misses={log_entry['interpolation']['misses']})\n")
            self.log_file.flush()
            
            # Write to calibration log file (JSON format)
            if self.calibration_log_file:
                self.calibration_log_entries.append(log_entry)
                # Write as JSON lines format for easy streaming/parsing
                json.dump(log_entry, self.calibration_log_file)
                self.calibration_log_file.write('\n')
                self.calibration_log_file.flush()
                
        except Exception as e:
            self.get_logger().error(f"Error in file logging: {e}")
    
    @staticmethod
    def stamp_to_sec(stamp):
        """Convert ROS timestamp to seconds."""
        return stamp.sec + stamp.nanosec * 1e-9
    
    def __del__(self):
        """Cleanup file handles on node destruction."""
        if hasattr(self, 'log_file') and self.log_file:
            self.log_file.close()
        if hasattr(self, 'calibration_log_file') and self.calibration_log_file:
            self.calibration_log_file.close()


def main():
    """Main entry point for GPS-VIO calibration node."""
    rclpy.init()
    
    try:
        node = GPSVIOCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()