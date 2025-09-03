#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from std_msgs.msg import String
from collections import deque
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import threading
import time
try:
    from pyproj import Transformer
except ImportError:
    Transformer = None

# Import rotation library for accurate euler-to-quaternion conversion
SCIPY_AVAILABLE = False
TF_TRANSFORMATIONS_AVAILABLE = False

try:
    from scipy.spatial.transform import Rotation
    SCIPY_AVAILABLE = True
except ImportError:
    try:
        # Fallback to tf_transformations if available
        import tf_transformations
        TF_TRANSFORMATIONS_AVAILABLE = True
    except ImportError:
        pass  # Both libraries unavailable, will use manual conversion

class SimpleTransformer:
    """Fallback transformer for basic lat/lon to ECEF conversion when pyproj is not available"""
    def __init__(self):
        self.a = 6378137.0  # WGS84 semi-major axis
        self.f = 1/298.257223563  # WGS84 flattening
        self.e2 = 2*self.f - self.f*self.f  # first eccentricity squared
        
    def transform(self, lon, lat, alt):
        """Convert WGS84 lat/lon/alt to ECEF coordinates"""
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        N = self.a / math.sqrt(1 - self.e2 * math.sin(lat_rad)**2)
        
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - self.e2) + alt) * math.sin(lat_rad)
        
        return x, y, z

class GpsOnGlobeNode(Node):
    def __init__(self):
        super().__init__('gps_on_globe')
        # Parameters
        self.declare_parameter('frame_id', 'earth')
        self.declare_parameter('gps_topic', 'gps/fix')
        self.declare_parameter('max_points', 2000)
        self.declare_parameter('point_radius_m', 50000.0)  # visual radius of each GPS point
        self.declare_parameter('trail_width_m', 80000.0)   # width of the trail line
        self.declare_parameter('publish_every_n', 1)       # decimate high-rate GPS
        self.declare_parameter('scale', 1.0)               # scale factor for coordinate conversion
        self.declare_parameter('point_color_r', 1.0)      # GPS point red component
        self.declare_parameter('point_color_g', 0.1)      # GPS point green component  
        self.declare_parameter('point_color_b', 0.1)      # GPS point blue component
        self.declare_parameter('trail_color_r', 1.0)      # GPS trail red component
        self.declare_parameter('trail_color_g', 0.85)     # GPS trail green component
        self.declare_parameter('trail_color_b', 0.1)      # GPS trail blue component
        self.declare_parameter('enable_debug_output', True)  # Enable debug coordinate output
        self.declare_parameter('camera_height', 100.0)    # Camera height above GPS point in meters
        self.declare_parameter('camera_window_size', 10)   # Number of GPS points for camera center calculation
        self.declare_parameter('camera_min_update_distance', 5.0)  # Minimum distance to update camera position (meters)
        self.declare_parameter('camera_yaw', 2.0454)      # Camera yaw angle in radians (horizontal rotation)
        self.declare_parameter('camera_pitch', 1.4304)    # Camera pitch angle in radians (vertical tilt)
        self.declare_parameter('camera_roll', 0.0)        # Camera roll angle in radians (rotation around viewing axis)
        self.declare_parameter('camera_frame_id', 'camera_view')  # TF frame ID for camera (empty string disables camera TF)
        self.declare_parameter('marker_calc_frequency', 2.0)  # Frequency for marker size calculation (Hz)

        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)
        base_point_radius = float(self.get_parameter('point_radius_m').value)
        base_trail_width = float(self.get_parameter('trail_width_m').value)
        self.publish_every_n = int(self.get_parameter('publish_every_n').value)
        scale = float(self.get_parameter('scale').value)
        
        # Type checking and validation for scale parameter
        if not isinstance(scale, (int, float)) or scale <= 0:
            self.get_logger().error(f"Invalid scale parameter: {scale}. Must be a positive number.")
            raise ValueError(f"Scale parameter must be positive, got: {scale}")
        
        # Get color parameters
        self.point_color_r = float(self.get_parameter('point_color_r').value)
        self.point_color_g = float(self.get_parameter('point_color_g').value)
        self.point_color_b = float(self.get_parameter('point_color_b').value)
        self.trail_color_r = float(self.get_parameter('trail_color_r').value)
        self.trail_color_g = float(self.get_parameter('trail_color_g').value)
        self.trail_color_b = float(self.get_parameter('trail_color_b').value)
        
        # Apply scale to all affected variables once during initialization
        self.coordinate_scale = scale  # For ECEF coordinate transformation
        self.point_radius_scaled = base_point_radius * scale  # Scaled point radius
        self.trail_width_scaled = base_trail_width * scale    # Scaled trail width
        
        # Store original values for logging
        self.base_point_radius = base_point_radius
        self.base_trail_width = base_trail_width
        self.scale_factor = scale
        self.gps_topic = self.get_parameter('gps_topic').value
        self.enable_debug_output = self.get_parameter('enable_debug_output').value
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_window_size = int(self.get_parameter('camera_window_size').value)
        self.camera_min_update_distance = float(self.get_parameter('camera_min_update_distance').value)
        self.camera_yaw = float(self.get_parameter('camera_yaw').value)
        self.camera_pitch = float(self.get_parameter('camera_pitch').value)
        self.camera_roll = float(self.get_parameter('camera_roll').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)
        self.marker_calc_frequency = float(self.get_parameter('marker_calc_frequency').value)
        

        
        # Centralized parameter logging
        self._log_initialization_parameters()

        # LLA -> ECEF transformer
        if Transformer is not None:
            # Using pyproj (EPSG:4979 to EPSG:4978), always_xy = lon,lat,h order
            self.transformer = Transformer.from_crs('EPSG:4979', 'EPSG:4978', always_xy=True)
            self.get_logger().info("Using pyproj for coordinate transformation")
        else:
            # Using fallback simple transformer
            self.transformer = SimpleTransformer()
            self.get_logger().warn("pyproj not available, using simple coordinate transformation")

        # Log rotation library status and validate quaternion conversion
        if SCIPY_AVAILABLE:
            self.get_logger().info("Using scipy.spatial.transform.Rotation for quaternion conversion")
        elif TF_TRANSFORMATIONS_AVAILABLE:
            self.get_logger().info("Using tf_transformations for quaternion conversion")  
        else:
            self.get_logger().warn("Using manual quaternion conversion - consider installing scipy for better accuracy")
        
        # Validate quaternion conversion with test values
        self._validate_quaternion_conversion()

        self.points_ecef = deque(maxlen=self.max_points)
        self.fix_count = 0
        
        # TF broadcaster for camera control
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Camera positioning with windowed center
        self.camera_window_points = deque(maxlen=self.camera_window_size)
        self.current_camera_center = [0.0, 0.0, 0.0]
        self.last_camera_position = [0.0, 0.0, 0.0]
        self.has_gps_fix = False
        
        # Thread-safe marker size calculation (initialize flag first)
        self.marker_calc_running = True  # Must be set before any thread operations
        self.marker_lock = threading.RLock()
        self.cached_point_size = self.base_point_radius * self.coordinate_scale
        self.cached_trail_width = self.base_trail_width * self.coordinate_scale
        self.marker_calc_thread = None

        self.sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, 10)
        self.pub_points = self.create_publisher(Marker, 'gps_globe_points', 1)
        self.pub_trail = self.create_publisher(Marker, 'gps_globe_trail', 1)
        
        # Debug publishers for coordinate transformation
        if self.enable_debug_output:
            self.pub_debug_ecef_transformed = self.create_publisher(PointStamped, 'debug/gps_ecef_transformed', 10)
            self.pub_debug_info = self.create_publisher(String, 'debug/gps_transform_info', 10)

        self.get_logger().info(f"GpsOnGlobeNode started: topic={self.gps_topic} frame={self.frame_id}")

    def _log_initialization_parameters(self):
        """Centralized logging function for initialization parameters"""
        self.get_logger().info("=== GPS On Globe Node Initialization ===")
        self.get_logger().info(f"GPS Topic: {self.gps_topic}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info(f"Scale Factor (received): {self.scale_factor}")
        
        if self.scale_factor == 1.0:
            self.get_logger().info("Scale = 1.0: Using real Earth size and ECEF coordinates")
        else:
            self.get_logger().info(f"Scale = {self.scale_factor}: Coordinates and markers scaled by factor {self.scale_factor}")
            if self.scale_factor < 1.0:
                self.get_logger().info(f"Display will be {1/self.scale_factor:.0f}x smaller than real size")
            else:
                self.get_logger().info(f"Display will be {self.scale_factor:.0f}x larger than real size")
        
        self.get_logger().info(f"Max Points: {self.max_points}")
        self.get_logger().info(f"Point Radius (base): {self.base_point_radius} m")
        self.get_logger().info(f"Point Radius (scaled): {self.point_radius_scaled} m")
        self.get_logger().info(f"Trail Width (base): {self.base_trail_width} m")
        self.get_logger().info(f"Trail Width (scaled): {self.trail_width_scaled} m")
        self.get_logger().info(f"Point Color RGB: ({self.point_color_r}, {self.point_color_g}, {self.point_color_b})")
        self.get_logger().info(f"Trail Color RGB: ({self.trail_color_r}, {self.trail_color_g}, {self.trail_color_b})")
        self.get_logger().info(f"Publish Every N: {self.publish_every_n}")
        self.get_logger().info(f"Debug Output Enabled: {self.enable_debug_output}")
        self.get_logger().info(f"Camera Height: {self.camera_height} m")
        self.get_logger().info(f"Camera Window Size: {self.camera_window_size} points")
        self.get_logger().info(f"Camera Min Update Distance: {self.camera_min_update_distance} m")
        self.get_logger().info(f"Camera Angles - Yaw: {self.camera_yaw:.4f} rad ({math.degrees(self.camera_yaw):.1f}°)")
        self.get_logger().info(f"                Pitch: {self.camera_pitch:.4f} rad ({math.degrees(self.camera_pitch):.1f}°)")
        self.get_logger().info(f"                Roll: {self.camera_roll:.4f} rad ({math.degrees(self.camera_roll):.1f}°)")
        self.get_logger().info(f"Camera Frame ID: {self.camera_frame_id} {'(camera TF disabled)' if not self.camera_frame_id else ''}")
        self.get_logger().info(f"Marker Calc Frequency: {self.marker_calc_frequency} Hz")
        if self.enable_debug_output:
            self.get_logger().info("Debug Topics:")
            self.get_logger().info("  - /debug/gps_ecef_transformed (PointStamped): Transformed ECEF coordinates")
            self.get_logger().info("  - /debug/gps_transform_info (String): Transformation details")
        self.get_logger().info("=========================================")
        
        # Start marker calculation thread
        self.start_marker_calculation_thread()
    
    def validate_gps_data(self, msg: NavSatFix):
        """Validate GPS data and return True if valid, False otherwise"""
        lon = msg.longitude
        lat = msg.latitude
        alt = msg.altitude
        
        # Check for NaN values
        if math.isnan(lon) or math.isnan(lat) or math.isnan(alt):
            self.get_logger().warn(f"GPS Fix {self.fix_count}: NaN values detected - lon:{lon}, lat:{lat}, alt:{alt}")
            return False
        
        # Check for infinite values
        if math.isinf(lon) or math.isinf(lat) or math.isinf(alt):
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Infinite values detected - lon:{lon}, lat:{lat}, alt:{alt}")
            return False
        
        # Check longitude range [-180, 180]
        if lon < -180.0 or lon > 180.0:
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Invalid longitude {lon} (must be between -180 and 180)")
            return False
        
        # Check latitude range [-90, 90]
        if lat < -90.0 or lat > 90.0:
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Invalid latitude {lat} (must be between -90 and 90)")
            return False
        
        # Check altitude range (reasonable values)
        if alt < -500.0 or alt > 50000.0:  # -500m (Dead Sea) to 50km (stratosphere)
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Suspicious altitude {alt}m (outside -500 to 50000m range)")
            return False
        
        # Check for zero coordinates (often invalid)
        if abs(lon) < 1e-6 and abs(lat) < 1e-6:
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Zero coordinates detected - likely invalid GPS fix")
            return False
        
        return True
    
    def start_marker_calculation_thread(self):
        """Start the marker size calculation thread"""
        if self.marker_calc_frequency <= 0:
            self.get_logger().info("Marker calculation thread disabled (frequency <= 0)")
            return
        
        # Ensure the running flag is set before creating thread
        self.marker_calc_running = True
        
        self.marker_calc_thread = threading.Thread(
            target=self._marker_calculation_worker,
            daemon=True,
            name=f"MarkerCalc-{self.get_name()}"  # Named thread for debugging
        )
        self.marker_calc_thread.start()
        self.get_logger().info(f"Marker calculation thread started at {self.marker_calc_frequency} Hz")
    
    def _marker_calculation_worker(self):
        """Worker thread for calculating adaptive marker sizes"""
        try:
            calc_period = 1.0 / self.marker_calc_frequency
        except (AttributeError, ZeroDivisionError) as e:
            self.get_logger().error(f"Invalid marker calculation frequency: {e}")
            return
        
        self.get_logger().info(f"Marker calculation worker started with period {calc_period:.3f}s")
        
        # Safety check for attribute existence (handle race conditions)
        while getattr(self, 'marker_calc_running', False):
            try:
                # Ensure required attributes exist before proceeding
                if not hasattr(self, 'points_ecef') or not hasattr(self, 'marker_lock'):
                    self.get_logger().warn("Required attributes not yet initialized, waiting...")
                    time.sleep(0.1)
                    continue
                
                # Calculate new marker sizes
                new_point_size, new_trail_width = self._calculate_adaptive_sizes()
                
                # Update cached values with write lock
                with self.marker_lock:
                    self.cached_point_size = new_point_size
                    self.cached_trail_width = new_trail_width
                
                time.sleep(calc_period)
                
            except Exception as e:
                self.get_logger().error(f"Error in marker calculation thread: {e}")
                time.sleep(max(calc_period, 1.0))  # Longer sleep on error
        
        self.get_logger().info("Marker calculation worker thread terminated")
    
    def _calculate_adaptive_sizes(self):
        """Calculate adaptive marker sizes (thread-safe internal method)"""
        if len(self.points_ecef) < 2:
            return self.base_point_radius * self.coordinate_scale, self.base_trail_width * self.coordinate_scale
        
        # Create a copy for thread-safe access
        points_copy = list(self.points_ecef)
        
        # Calculate trajectory bounds
        x_coords = [p[0] for p in points_copy]
        y_coords = [p[1] for p in points_copy]
        z_coords = [p[2] for p in points_copy]
        
        # Calculate trajectory span
        extent_x = max(x_coords) - min(x_coords) if x_coords else 0
        extent_y = max(y_coords) - min(y_coords) if y_coords else 0
        extent_z = max(z_coords) - min(z_coords) if z_coords else 0
        
        trajectory_span = max(extent_x, extent_y, extent_z)
        
        if trajectory_span > 0:
            # GPS point size: 2% of trajectory span
            adaptive_point_size = trajectory_span * 0.02
            # Trail width: 0.8% of trajectory span  
            adaptive_trail_width = trajectory_span * 0.008
            
            # Set reasonable limits
            min_point_size = 0.1
            max_point_size = trajectory_span * 0.05
            min_trail_width = 0.05
            max_trail_width = trajectory_span * 0.02
            
            adaptive_point_size = max(min_point_size, min(adaptive_point_size, max_point_size))
            adaptive_trail_width = max(min_trail_width, min(adaptive_trail_width, max_trail_width))
            
            return adaptive_point_size, adaptive_trail_width
        
        return self.base_point_radius * self.coordinate_scale, self.base_trail_width * self.coordinate_scale
    
    def get_cached_marker_sizes(self):
        """Get cached marker sizes with read lock"""
        with self.marker_lock:
            return self.cached_point_size, self.cached_trail_width
    
    def update_camera_center(self, new_point):
        """Update camera center based on windowed GPS points"""
        self.camera_window_points.append(new_point)
        
        if len(self.camera_window_points) < 2:
            self.current_camera_center = list(new_point)
            return
        
        # Calculate center of window points
        x_sum = sum(p[0] for p in self.camera_window_points)
        y_sum = sum(p[1] for p in self.camera_window_points)
        z_sum = sum(p[2] for p in self.camera_window_points)
        
        count = len(self.camera_window_points)
        new_center = [x_sum / count, y_sum / count, z_sum / count]
        
        # Check if update is needed based on minimum distance
        if self._camera_distance(self.current_camera_center, new_center) >= self.camera_min_update_distance * self.coordinate_scale:
            self.current_camera_center = new_center
            return True  # Updated
        
        return False  # No update needed
    
    def _camera_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two 3D positions"""
        return math.sqrt(
            (pos1[0] - pos2[0])**2 + 
            (pos1[1] - pos2[1])**2 + 
            (pos1[2] - pos2[2])**2
        )
    
    def euler_to_quaternion(self, yaw, pitch, roll=0.0):
        """Convert Euler angles to quaternion using reliable library
        
        Args:
            yaw: Rotation around Z-axis (radians)
            pitch: Rotation around Y-axis (radians) 
            roll: Rotation around X-axis (radians, default 0.0)
            
        Returns:
            tuple: (qx, qy, qz, qw) quaternion components
        """
        try:
            if SCIPY_AVAILABLE:
                # Use scipy (most reliable) - 'ZYX' extrinsic rotation order
                r = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=False)
                quat = r.as_quat()  # Returns [qx, qy, qz, qw]
                return quat[0], quat[1], quat[2], quat[3]
            
            elif TF_TRANSFORMATIONS_AVAILABLE:
                # Use tf_transformations as fallback
                quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')
                return quat[0], quat[1], quat[2], quat[3]  # tf returns [qx, qy, qz, qw]
            
            else:
                # Manual implementation as last resort (improved version)
                self.get_logger().warn("Using manual quaternion conversion - consider installing scipy")
                return self._manual_euler_to_quaternion(yaw, pitch, roll)
                
        except Exception as e:
            self.get_logger().error(f"Quaternion conversion failed: {e}")
            # Return identity quaternion as safe fallback
            return 0.0, 0.0, 0.0, 1.0
    
    def _manual_euler_to_quaternion(self, yaw, pitch, roll):
        """Manual Euler to quaternion conversion (improved implementation)
        
        This uses the ZYX intrinsic rotation sequence which is standard for
        aerospace applications and matches ROS conventions.
        """
        # Convert to half angles
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        # ZYX intrinsic rotation order
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def _validate_quaternion_conversion(self):
        """Validate quaternion conversion with known test cases"""
        try:
            # Test identity rotation (should give identity quaternion)
            qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, 0.0)
            identity_error = abs(qx) + abs(qy) + abs(qz) + abs(qw - 1.0)
            
            if identity_error > 1e-10:
                self.get_logger().warn(f"Quaternion identity test failed: error={identity_error}")
            
            # Test 90-degree rotations
            # Yaw 90° should give (0, 0, sin(π/4), cos(π/4)) ≈ (0, 0, 0.707, 0.707)
            qx, qy, qz, qw = self.euler_to_quaternion(math.pi/2, 0.0, 0.0)
            expected_z = math.sin(math.pi/4)
            expected_w = math.cos(math.pi/4)
            error = abs(qx) + abs(qy) + abs(qz - expected_z) + abs(qw - expected_w)
            
            if error > 1e-6:
                self.get_logger().warn(f"Quaternion 90° yaw test failed: error={error}")
            else:
                self.get_logger().debug("Quaternion conversion validation passed")
                
            # Test the actual screenshot values
            qx, qy, qz, qw = self.euler_to_quaternion(self.camera_yaw, self.camera_pitch, self.camera_roll)
            # Normalize to unit quaternion
            magnitude = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if abs(magnitude - 1.0) > 1e-6:
                self.get_logger().warn(f"Camera quaternion not normalized: magnitude={magnitude}")
            else:
                self.get_logger().info(f"Camera quaternion validated: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")
                
        except Exception as e:
            self.get_logger().error(f"Quaternion validation failed: {e}")
    
    def publish_camera_tf(self):
        """Publish camera transform at configured height above windowed center position"""
        # Skip camera TF publishing if camera_frame_id is empty or not set
        if not self.camera_frame_id or not self.camera_frame_id.strip():
            return
            
        if not self.has_gps_fix or len(self.camera_window_points) == 0:
            return
        
        # Calculate camera position: windowed center + height offset
        camera_height_scaled = self.camera_height * self.coordinate_scale
        camera_x = self.current_camera_center[0]
        camera_y = self.current_camera_center[1] 
        camera_z = self.current_camera_center[2] + camera_height_scaled
        
        # Create transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.camera_frame_id
        
        # Set camera position
        t.transform.translation.x = camera_x
        t.transform.translation.y = camera_y
        t.transform.translation.z = camera_z
        
        # Set camera orientation using configured angles
        # Default values from screenshot: Yaw=2.0454 rad (~117.2°), Pitch=1.4304 rad (~82.0°)
        # This creates a top-down angled view looking at the GPS trajectory
        
        # Convert Euler angles to quaternion using configured parameters
        qx, qy, qz, qw = self.euler_to_quaternion(self.camera_yaw, self.camera_pitch, self.camera_roll)
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().debug(f"Camera positioned above windowed center: ({camera_x:.2f}, {camera_y:.2f}, {camera_z:.2f})")

    def gps_cb(self, msg: NavSatFix):
        self.fix_count += 1
        
        # Validate GPS data first
        if not self.validate_gps_data(msg):
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Invalid GPS data, skipping processing")
            return
        
        if self.publish_every_n > 1 and (self.fix_count % self.publish_every_n) != 0:
            return

        # Convert LLA -> ECEF (meters) and apply pre-calculated scale
        lon = msg.longitude
        lat = msg.latitude
        alt = msg.altitude
        
        try:
            x, y, z = self.transformer.transform(lon, lat, alt)
        except Exception as e:
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Coordinate transformation failed:{msg.header.stamp} {lat} {lon} {alt}")
            return
        
        # Apply pre-calculated coordinate scale for display
        x_scaled = x * self.coordinate_scale
        y_scaled = y * self.coordinate_scale  
        z_scaled = z * self.coordinate_scale
        
        # Validate transformed coordinates
        if math.isnan(x_scaled) or math.isnan(y_scaled) or math.isnan(z_scaled):
            self.get_logger().warn(f"GPS Fix {self.fix_count}: NaN in transformed coordinates")
            return
        
        if math.isinf(x_scaled) or math.isinf(y_scaled) or math.isinf(z_scaled):
            self.get_logger().warn(f"GPS Fix {self.fix_count}: Infinite values in transformed coordinates")
            return
        
        # Add to trajectory
        scaled_point = (x_scaled, y_scaled, z_scaled)
        self.points_ecef.append(scaled_point)
        
        # Update camera positioning with windowed center
        camera_updated = self.update_camera_center(scaled_point)
        self.has_gps_fix = True
        
        # Publish debug information if enabled
        if self.enable_debug_output:
            self.publish_debug_coordinates(msg, x, y, z, x_scaled, y_scaled, z_scaled)
            self.get_logger().info(f"GPS Fix {self.fix_count}: ({x_scaled:.2f}, {y_scaled:.2f}, {z_scaled:.2f})")
            if camera_updated:
                self.get_logger().info(f"Camera center updated: ({self.current_camera_center[0]:.2f}, {self.current_camera_center[1]:.2f}, {self.current_camera_center[2]:.2f})")

        # Publish markers
        self.publish_points_marker()
        self.publish_trail_marker()
        
        # Publish camera TF (only if camera center was updated or this is first fix)
        if camera_updated or len(self.camera_window_points) == 1:
            self.publish_camera_tf()

    def publish_points_marker(self):
        """发布GPS点标记 - 使用缓存的自适应大小"""
        # Get cached marker sizes with read lock
        adaptive_point_size, _ = self.get_cached_marker_sizes()
        
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'gps'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        
        # 使用自适应大小替代固定大小
        m.scale.x = adaptive_point_size
        m.scale.y = adaptive_point_size
        m.scale.z = adaptive_point_size
        
        m.color.a = 0.9
        m.color.r = self.point_color_r
        m.color.g = self.point_color_g
        m.color.b = self.point_color_b
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.points_ecef]
        self.pub_points.publish(m)

    def publish_trail_marker(self):
        """发布GPS轨迹标记 - 使用缓存的自适应大小"""
        # Get cached marker sizes with read lock
        _, adaptive_trail_width = self.get_cached_marker_sizes()
        
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'gps'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        
        # 使用自适应线宽替代固定大小
        m.scale.x = adaptive_trail_width
        
        m.color.a = 0.8
        m.color.r = self.trail_color_r
        m.color.g = self.trail_color_g
        m.color.b = self.trail_color_b
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.points_ecef]
        self.pub_trail.publish(m)
    
    def publish_debug_coordinates(self, original_msg, x_ecef, y_ecef, z_ecef, x_scaled, y_scaled, z_scaled):
        """Publish debug information about coordinate transformation"""
        # Publish transformed ECEF point (after scaling)
        ecef_point = PointStamped()
        ecef_point.header.frame_id = self.frame_id
        ecef_point.header.stamp = self.get_clock().now().to_msg()
        ecef_point.point.x = x_scaled
        ecef_point.point.y = y_scaled
        ecef_point.point.z = z_scaled
        self.pub_debug_ecef_transformed.publish(ecef_point)
        
        # Publish detailed transformation information
        debug_info = String()
        debug_info.data = (
            f"GPS_FIX_{self.fix_count}: "
            f"LLA_IN: [{original_msg.longitude:.8f}, {original_msg.latitude:.8f}, {original_msg.altitude:.3f}] | "
            f"ECEF_RAW: [{x_ecef:.3f}, {y_ecef:.3f}, {z_ecef:.3f}] | "
            f"ECEF_SCALED: [{x_scaled:.3f}, {y_scaled:.3f}, {z_scaled:.3f}] | "
            f"SCALE: {self.coordinate_scale:.8f}"
        )
        self.pub_debug_info.publish(debug_info)
        
        # Also log debug info at DEBUG level
        self.get_logger().debug(
            f"Coordinate Transform Debug - Fix {self.fix_count}: "
            f"LLA({original_msg.longitude:.8f}, {original_msg.latitude:.8f}, {original_msg.altitude:.3f}) -> "
            f"ECEF_RAW({x_ecef:.3f}, {y_ecef:.3f}, {z_ecef:.3f}) -> "
            f"ECEF_SCALED({x_scaled:.3f}, {y_scaled:.3f}, {z_scaled:.3f}) [scale={self.coordinate_scale:.8f}]"
        )
    
    def cleanup(self):
        """Cleanup resources, especially the marker calculation thread"""
        # Safely stop the marker calculation thread
        if hasattr(self, 'marker_calc_running'):
            self.marker_calc_running = False
        
        if hasattr(self, 'marker_calc_thread') and self.marker_calc_thread and self.marker_calc_thread.is_alive():
            self.marker_calc_thread.join(timeout=2.0)  # Wait up to 2 seconds
            if self.marker_calc_thread.is_alive():
                self.get_logger().warn("Marker calculation thread did not terminate gracefully")
            else:
                self.get_logger().info("Marker calculation thread terminated successfully")
        else:
            self.get_logger().info("Marker calculation thread was not running or already terminated")


def main():
    rclpy.init()
    node = GpsOnGlobeNode()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()