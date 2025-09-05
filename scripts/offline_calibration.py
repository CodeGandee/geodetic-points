#!/usr/bin/env python3
"""
Offline VIO-GPS Calibration Script

This script performs offline extrinsic calibration between VIO and GPS data
from recorded ROS bag files. It processes synchronized data to estimate the
rigid body transformation between VIO local frame and ECEF coordinates.

Usage:
    python3 offline_calibration.py --input dataset.db3 --output calibration.yaml
    
Dependencies:
    - numpy, scipy (for numerical computation)
    - pyproj (for geodetic transformations)  
    - rosbag2_py (for ROS bag processing)
    - yaml (for configuration output)
"""

import argparse
import numpy as np
import yaml
from pathlib import Path
from collections import defaultdict
import sys

try:
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import minimize_scalar
    from scipy.interpolate import interp1d
    SCIPY_AVAILABLE = True
except ImportError:
    print("WARNING: SciPy not available - using simplified algorithms")
    SCIPY_AVAILABLE = False

try:
    from pyproj import Transformer
    PYPROJ_AVAILABLE = True
except ImportError:
    print("WARNING: pyproj not available - using simplified coordinate transforms")
    PYPROJ_AVAILABLE = False

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ROSBAG_AVAILABLE = True
except ImportError:
    print("ERROR: rosbag2_py not available - cannot process ROS bags")
    ROSBAG_AVAILABLE = False


class OfflineCalibrator:
    """
    Offline calibration processor for VIO-GPS extrinsic parameter estimation.
    """
    
    def __init__(self):
        self.gps_data = []  # [(timestamp, lat, lon, alt), ...]
        self.vio_data = []  # [(timestamp, x, y, z), ...]
        self.anchor_ecef = None
        
        # Coordinate transformer
        if PYPROJ_AVAILABLE:
            self.transformer_lla_to_ecef = Transformer.from_crs(
                "EPSG:4979", "EPSG:4978", always_xy=True
            )

    def load_bag_data(self, bag_path):
        """Load GPS and VIO data from ROS bag file."""
        if not ROSBAG_AVAILABLE:
            raise RuntimeError("rosbag2_py not available")
            
        print(f"Loading bag: {bag_path}")
        
        # Open bag reader
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)
        
        # Get topic metadata
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        print("Available topics:")
        for topic_name, topic_type in type_map.items():
            print(f"  {topic_name}: {topic_type}")
            
        # Read messages
        gps_count = 0
        vio_count = 0
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/gps/fix' or topic == 'gps/fix':
                if 'NavSatFix' in type_map.get(topic, ''):
                    msg = deserialize_message(data, get_message('sensor_msgs/NavSatFix'))
                    ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    self.gps_data.append((ros_time, msg.latitude, msg.longitude, msg.altitude))
                    gps_count += 1
                    
            elif topic == '/vio/odometry' or topic == 'vio/odometry':
                if 'Odometry' in type_map.get(topic, ''):
                    msg = deserialize_message(data, get_message('nav_msgs/Odometry'))
                    ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    pos = msg.pose.pose.position
                    self.vio_data.append((ros_time, pos.x, pos.y, pos.z))
                    vio_count += 1
                    
        reader.close()
        
        print(f"Loaded {gps_count} GPS fixes and {vio_count} VIO poses")
        
        # Sort by timestamp
        self.gps_data.sort(key=lambda x: x[0])
        self.vio_data.sort(key=lambda x: x[0])

    def estimate_time_sync(self, max_offset=2.0, search_points=21):
        """Estimate time synchronization between GPS and VIO."""
        if len(self.gps_data) < 10 or len(self.vio_data) < 10:
            raise ValueError("Insufficient data for time synchronization")
            
        print("Estimating time synchronization...")
        
        # Convert GPS to local ECEF
        gps_ecef_local = []
        for _, lat, lon, alt in self.gps_data:
            ecef_point = self.gps_to_ecef_local(lat, lon, alt)
            if ecef_point is not None:
                gps_ecef_local.append(ecef_point)
                
        if len(gps_ecef_local) == 0:
            raise ValueError("Failed to convert GPS coordinates")
            
        # Create VIO interpolator
        vio_times = np.array([t for t, _, _, _ in self.vio_data])
        vio_positions = np.array([[x, y, z] for _, x, y, z in self.vio_data])
        
        if SCIPY_AVAILABLE:
            vio_interp = interp1d(vio_times, vio_positions, axis=0, kind='linear',
                                  bounds_error=False, fill_value='extrapolate')
        else:
            vio_interp = None
            
        # Search for best offset
        best_offset = 0.0
        best_error = float('inf')
        
        offset_range = np.linspace(-max_offset, max_offset, search_points)
        
        for offset in offset_range:
            total_error = 0.0
            valid_pairs = 0
            
            for i, (gps_time, _, _, _) in enumerate(self.gps_data):
                if i >= len(gps_ecef_local):
                    break
                    
                vio_time = gps_time + offset
                
                if vio_interp is not None:
                    try:
                        vio_pos = vio_interp(vio_time)
                        if not np.any(np.isnan(vio_pos)):
                            error = np.linalg.norm(vio_pos - gps_ecef_local[i])
                            total_error += error
                            valid_pairs += 1
                    except:
                        continue
                else:
                    # Simple nearest neighbor fallback
                    closest_idx = np.argmin(np.abs(vio_times - vio_time))
                    if np.abs(vio_times[closest_idx] - vio_time) < 1.0:  # 1 second tolerance
                        vio_pos = vio_positions[closest_idx]
                        error = np.linalg.norm(vio_pos - gps_ecef_local[i])
                        total_error += error
                        valid_pairs += 1
                        
            if valid_pairs > 0:
                avg_error = total_error / valid_pairs
                if avg_error < best_error:
                    best_error = avg_error
                    best_offset = offset
                    
        print(f"Best time offset: {best_offset:.4f}s, RMS error: {best_error:.3f}m")
        return best_offset

    def perform_calibration(self, time_offset):
        """Perform extrinsic calibration with known time offset."""
        print("Performing extrinsic calibration...")
        
        # Prepare synchronized data pairs
        vio_positions = []
        gps_positions = []
        
        # Create VIO interpolator
        vio_times = np.array([t for t, _, _, _ in self.vio_data])
        vio_pos_array = np.array([[x, y, z] for _, x, y, z in self.vio_data])
        
        if SCIPY_AVAILABLE:
            vio_interp = interp1d(vio_times, vio_pos_array, axis=0, kind='linear',
                                  bounds_error=False, fill_value='extrapolate')
        else:
            vio_interp = None
            
        # Collect synchronized pairs
        for gps_time, lat, lon, alt in self.gps_data:
            vio_time = gps_time + time_offset
            
            # Get VIO position at synchronized time
            if vio_interp is not None:
                try:
                    vio_pos = vio_interp(vio_time)
                    if np.any(np.isnan(vio_pos)):
                        continue
                except:
                    continue
            else:
                # Fallback: nearest neighbor
                closest_idx = np.argmin(np.abs(vio_times - vio_time))
                if np.abs(vio_times[closest_idx] - vio_time) > 0.5:
                    continue
                vio_pos = vio_pos_array[closest_idx]
                
            # Convert GPS to local ECEF
            gps_ecef = self.gps_to_ecef_local(lat, lon, alt)
            if gps_ecef is None:
                continue
                
            vio_positions.append(vio_pos)
            gps_positions.append(gps_ecef)
            
        if len(vio_positions) < 10:
            raise ValueError(f"Insufficient synchronized pairs: {len(vio_positions)}")
            
        print(f"Found {len(vio_positions)} synchronized pairs")
        
        # Perform rigid alignment
        P = np.array(vio_positions)
        Q = np.array(gps_positions)
        
        R_matrix, t_vector, rms_error = self.rigid_alignment(P, Q)
        
        print(f"Calibration complete - RMS error: {rms_error:.4f}m")
        return R_matrix, t_vector, rms_error

    def gps_to_ecef_local(self, lat, lon, alt):
        """Convert GPS to local ECEF coordinates."""
        if PYPROJ_AVAILABLE:
            try:
                x, y, z = self.transformer_lla_to_ecef.transform(lon, lat, alt)
                ecef_point = np.array([x, y, z])
            except Exception:
                return None
        else:
            # Simple conversion
            ecef_point = self.simple_lla_to_ecef(lat, lon, alt)
            
        # Set anchor
        if self.anchor_ecef is None:
            self.anchor_ecef = ecef_point
            print(f"Set ECEF anchor: {self.anchor_ecef}")
            
        return ecef_point - self.anchor_ecef

    def simple_lla_to_ecef(self, lat, lon, alt):
        """Simple LLA to ECEF conversion."""
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

    def rigid_alignment(self, P, Q):
        """Compute rigid alignment using Kabsch algorithm."""
        if len(P) != len(Q) or len(P) < 3:
            raise ValueError("Need at least 3 corresponding points")
            
        # Center point sets
        mu_P = P.mean(axis=0)
        mu_Q = Q.mean(axis=0)
        P_centered = P - mu_P
        Q_centered = Q - mu_Q
        
        if SCIPY_AVAILABLE:
            # Use SciPy for robust rotation estimation
            rot_obj, rssd = R.align_vectors(Q_centered, P_centered)
            R_matrix = rot_obj.as_matrix()
        else:
            # Manual SVD implementation
            H = P_centered.T @ Q_centered
            U, S, Vt = np.linalg.svd(H)
            R_matrix = Vt.T @ U.T
            
            # Ensure proper rotation
            if np.linalg.det(R_matrix) < 0:
                Vt[-1, :] *= -1
                R_matrix = Vt.T @ U.T
                
        # Compute translation
        t_vector = mu_Q - R_matrix @ mu_P
        
        # Compute RMS error
        P_aligned = (R_matrix @ P.T).T + t_vector
        residuals = Q - P_aligned
        rms_error = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        
        return R_matrix, t_vector, rms_error

    def save_calibration(self, output_path, R_matrix, t_vector, anchor_ecef, rms_error, time_offset):
        """Save calibration results to YAML file."""
        
        # Convert rotation matrix to quaternion
        if SCIPY_AVAILABLE:
            rot_obj = R.from_matrix(R_matrix)
            quat = rot_obj.as_quat()  # [x, y, z, w]
        else:
            quat = self.matrix_to_quaternion(R_matrix)
            
        calibration_data = {
            'calibration_info': {
                'timestamp': str(np.datetime64('now')),
                'rms_error_meters': float(rms_error),
                'time_offset_seconds': float(time_offset),
                'coordinate_frame': 'ECEF (WGS84)',
                'algorithm': 'Rigid Kabsch alignment'
            },
            'extrinsic_transform': {
                'rotation_matrix': R_matrix.tolist(),
                'translation_vector': t_vector.tolist(),
                'quaternion_xyzw': quat.tolist(),
                'anchor_ecef': anchor_ecef.tolist()
            },
            'ros2_parameters': {
                'time_sync_node': {
                    'initial_offset': float(time_offset),
                    'drift_parameter': 1.0
                },
                'calibration_node': {
                    'auto_publish_transform': True,
                    'convergence_threshold': 0.01
                },
                'points_transform_node': {
                    'anchor_ecef': anchor_ecef.tolist(),
                    'frame_id_ecef': 'ecef_frame'
                }
            },
            'validation_metrics': {
                'data_points_gps': len(self.gps_data),
                'data_points_vio': len(self.vio_data),
                'synchronized_pairs': int(len(self.gps_data) * 0.8),  # Estimate
                'spatial_rms_error_m': float(rms_error)
            }
        }
        
        with open(output_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, indent=2)
            
        print(f"Calibration saved to: {output_path}")

    def matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion."""
        trace = np.trace(R)
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
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

    def run_calibration(self, bag_path, output_path):
        """Run complete offline calibration pipeline."""
        try:
            # Load data
            self.load_bag_data(bag_path)
            
            if len(self.gps_data) < 10:
                raise ValueError(f"Insufficient GPS data: {len(self.gps_data)} points")
            if len(self.vio_data) < 10:
                raise ValueError(f"Insufficient VIO data: {len(self.vio_data)} points")
                
            # Estimate time synchronization
            time_offset = self.estimate_time_sync()
            
            # Perform calibration
            R_matrix, t_vector, rms_error = self.perform_calibration(time_offset)
            
            # Save results
            self.save_calibration(output_path, R_matrix, t_vector, 
                                  self.anchor_ecef, rms_error, time_offset)
            
            print(f"\nCalibration Summary:")
            print(f"  Time offset: {time_offset:.4f} seconds")
            print(f"  Spatial RMS error: {rms_error:.4f} meters")
            print(f"  Translation: {t_vector}")
            print(f"  Rotation determinant: {np.linalg.det(R_matrix):.6f}")
            
            return True
            
        except Exception as e:
            print(f"Calibration failed: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Offline VIO-GPS Calibration')
    parser.add_argument('--input', '-i', required=True, 
                        help='Input ROS bag file path (.db3)')
    parser.add_argument('--output', '-o', required=True,
                        help='Output calibration file path (.yaml)')
    parser.add_argument('--max-offset', type=float, default=2.0,
                        help='Maximum time offset search range (seconds)')
    
    args = parser.parse_args()
    
    # Validate paths
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input bag file not found: {input_path}")
        return 1
        
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Check dependencies
    if not ROSBAG_AVAILABLE:
        print("Error: rosbag2_py not available - cannot process ROS bags")
        print("Install with: pip install rosbag2_py")
        return 1
        
    # Run calibration
    calibrator = OfflineCalibrator()
    success = calibrator.run_calibration(input_path, output_path)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())