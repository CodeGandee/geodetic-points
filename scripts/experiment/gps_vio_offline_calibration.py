#!/usr/bin/env python3
"""
GPS-VIO Offline Calibration (ROS2-Free Implementation)

This script performs GPS-VIO calibration using pre-extracted data from ROS2 bags.
It combines time synchronization and spatial calibration to compute precise
transformations between VIO local coordinates and GPS global coordinates.

Core Features:
1. Time Synchronization: Estimates affine time model t_corrected = a * t_gps + b
2. Spatial Calibration: Computes rigid transform T_earth_odom via Kabsch algorithm  
3. Coordinate Conversion: Transforms between LLA, ECEF, and local VIO frames
4. Quality Assessment: Provides comprehensive calibration metrics and validation
5. Visualization: Generates plots for analysis and verification

Input Data Format:
- GPS data: CSV with columns [topic_name, timestamp_ms, ros_timestamp_ms, latitude, longitude, altitude, ...]
- Odometry data: CSV with columns [topic_name, timestamp_ms, ros_timestamp_ms, position_x, position_y, position_z, ...]

Output:
- Transformation matrix (4x4 homogeneous transform)
- Time synchronization parameters  
- Quality metrics and statistics
- Visualization plots (optional)

Usage:
    python3 gps_vio_offline_calibration.py --gps_data /path/to/gps_data.csv --odom_data /path/to/odom_data.csv
    python3 gps_vio_offline_calibration.py --data_dir /path/to/results/odom_output --output_dir ./calibration_results

Author: Offline GPS-VIO Calibration System
Created: September 2025
Version: 1.0
"""

import argparse
import os
import sys
import json
import numpy as np
import pandas as pd
from datetime import datetime
from pathlib import Path
import logging
from collections import deque
import warnings

# Optional dependencies with fallbacks
try:
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import minimize_scalar
    from scipy.interpolate import interp1d
    from scipy.stats import linregress
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️  SciPy not available, using simplified algorithms")

try:
    from pyproj import Transformer
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False
    print("⚠️  PyProj not available, using simplified coordinate conversion")

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from matplotlib.gridspec import GridSpec
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️  Matplotlib not available, visualization disabled")

# Suppress warnings for cleaner output
warnings.filterwarnings("ignore", category=RuntimeWarning)


class GPSVIOOfflineCalibrator:
    """
    Offline GPS-VIO calibration system for pre-extracted bag data.
    
    This class performs complete GPS-VIO calibration including:
    1. Time synchronization between GPS and VIO timestamps
    2. Coordinate transformations (LLA -> ECEF -> Local)
    3. Rigid body alignment using Kabsch algorithm
    4. Quality assessment and validation
    """
    
    def __init__(self, config=None):
        """
        Initialize the calibrator with configuration parameters.
        
        Args:
            config (dict): Configuration parameters for calibration
        """
        # Default configuration
        self.config = {
            # Time synchronization parameters
            'time_sync': {
                'max_offset_search': 5.0,  # seconds
                'smoothing_alpha': 0.05,
                'drift_estimation_window': 50,
                'min_motion_threshold': 0.05  # meters
            },
            
            # Spatial calibration parameters  
            'calibration': {
                'window_size': 100,
                'min_motion_threshold': 0.05,  # meters
                'outlier_threshold': 3.0,  # sigma
                'convergence_threshold': 0.5,  # meters
                'max_error': 10.0,  # meters
                'min_pairs': 5
            },
            
            # Coordinate frames
            'frames': {
                'earth_frame': 'earth',
                'odom_frame': 'odom'
            },
            
            # Output settings
            'output': {
                'save_plots': True,
                'save_detailed_logs': True,
                'export_transform': True
            }
        }
        
        # Update with user config
        if config:
            self._update_config(self.config, config)
        
        # Initialize state variables
        self.reset_state()
        
        # Setup coordinate transformers
        self._setup_coordinate_transformers()
        
        # Setup logging
        self._setup_logging()
    
    def _update_config(self, base_config, new_config):
        """Recursively update configuration."""
        for key, value in new_config.items():
            if key in base_config and isinstance(base_config[key], dict) and isinstance(value, dict):
                self._update_config(base_config[key], value)
            else:
                base_config[key] = value
    
    def reset_state(self):
        """Reset all internal state variables."""
        # Time synchronization state
        self.time_a = 1.0  # drift parameter
        self.time_b = 0.0  # offset parameter
        self.gps_corrections = deque(maxlen=self.config['time_sync']['drift_estimation_window'])
        
        # Spatial calibration state
        self.sync_pairs = deque(maxlen=self.config['calibration']['window_size'])
        self.current_transform = np.eye(4)
        self.calibration_quality = float('inf')
        self.calibration_converged = False
        
        # Data storage
        self.gps_data = None
        self.odom_data = None
        self.processed_pairs = []
        
        # Statistics
        self.interpolation_hits = 0
        self.interpolation_misses = 0
        self.time_sync_iterations = 0
        
        # Results
        self.final_transform = None
        self.final_quality = None
        self.calibration_results = {}
    
    def _setup_coordinate_transformers(self):
        """Initialize coordinate transformation utilities."""
        if PYPROJ_AVAILABLE:
            self.transformer_lla_to_ecef = Transformer.from_crs(
                "EPSG:4979", "EPSG:4978", always_xy=True
            )
            self.transformer_ecef_to_lla = Transformer.from_crs(
                "EPSG:4978", "EPSG:4979", always_xy=True  
            )
            self.logger.info("PyProj coordinate transformers initialized")
        else:
            self.logger.warning("PyProj not available, using simplified coordinate conversion")
    
    def _setup_logging(self):
        """Setup logging configuration."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.logger = logging.getLogger(__name__)
    
    def load_data(self, gps_file=None, odom_file=None, data_dir=None):
        """
        Load GPS and odometry data from CSV files.
        
        Args:
            gps_file (str): Path to GPS data CSV file
            odom_file (str): Path to odometry data CSV file  
            data_dir (str): Directory containing both data files (alternative to individual files)
        
        Returns:
            bool: True if data loaded successfully, False otherwise
        """
        try:
            # Handle data directory input
            if data_dir:
                data_dir = Path(data_dir)
                if not data_dir.exists():
                    self.logger.error(f"Data directory does not exist: {data_dir}")
                    return False
                
                gps_file = data_dir / "gps_data_extracted.csv"
                odom_file = data_dir / "odom_data_extracted.csv"
                
                self.logger.info(f"Loading data from directory: {data_dir}")
            
            # Validate file paths
            if not gps_file or not odom_file:
                self.logger.error("Both GPS and odometry files must be specified")
                return False
            
            gps_path = Path(gps_file)
            odom_path = Path(odom_file)
            
            if not gps_path.exists():
                self.logger.error(f"GPS data file not found: {gps_path}")
                return False
            
            if not odom_path.exists():
                self.logger.error(f"Odometry data file not found: {odom_path}")
                return False
            
            # Load GPS data
            self.logger.info(f"Loading GPS data: {gps_path}")
            self.gps_data = pd.read_csv(gps_path)
            
            # Validate GPS data columns
            required_gps_cols = ['timestamp_ms', 'ros_timestamp_ms', 'latitude', 'longitude', 'altitude']
            missing_cols = [col for col in required_gps_cols if col not in self.gps_data.columns]
            if missing_cols:
                self.logger.error(f"Missing GPS columns: {missing_cols}")
                return False
            
            # Load odometry data
            self.logger.info(f"Loading odometry data: {odom_path}")
            self.odom_data = pd.read_csv(odom_path)
            
            # Validate odometry data columns
            required_odom_cols = ['timestamp_ms', 'ros_timestamp_ms', 'position_x', 'position_y', 'position_z']
            missing_cols = [col for col in required_odom_cols if col not in self.odom_data.columns]
            if missing_cols:
                self.logger.error(f"Missing odometry columns: {missing_cols}")
                return False
            
            # Data validation and preprocessing
            initial_gps_count = len(self.gps_data)
            initial_odom_count = len(self.odom_data)
            
            # Remove invalid GPS coordinates
            self.gps_data = self.gps_data.dropna(subset=['latitude', 'longitude', 'altitude'])
            valid_gps_mask = (
                (self.gps_data['latitude'].abs() <= 90) &
                (self.gps_data['longitude'].abs() <= 180) &
                (self.gps_data['altitude'].abs() < 50000)  # Reasonable altitude limit
            )
            self.gps_data = self.gps_data[valid_gps_mask]
            
            # Remove invalid odometry positions
            self.odom_data = self.odom_data.dropna(subset=['position_x', 'position_y', 'position_z'])
            valid_odom_mask = (
                (self.odom_data['position_x'].abs() < 100000) &  # 100km limit
                (self.odom_data['position_y'].abs() < 100000) &
                (self.odom_data['position_z'].abs() < 1000)     # 1km altitude limit
            )
            self.odom_data = self.odom_data[valid_odom_mask]
            
            # Sort by timestamp for interpolation
            self.gps_data = self.gps_data.sort_values('timestamp_ms').reset_index(drop=True)
            self.odom_data = self.odom_data.sort_values('timestamp_ms').reset_index(drop=True)
            
            # Log data statistics
            self.logger.info(f"Data loaded successfully:")
            self.logger.info(f"  GPS points: {len(self.gps_data)}/{initial_gps_count} (filtered out {initial_gps_count - len(self.gps_data)})")
            self.logger.info(f"  Odometry points: {len(self.odom_data)}/{initial_odom_count} (filtered out {initial_odom_count - len(self.odom_data)})")
            
            # Time range analysis
            gps_time_range = (self.gps_data['timestamp_ms'].min(), self.gps_data['timestamp_ms'].max())
            odom_time_range = (self.odom_data['timestamp_ms'].min(), self.odom_data['timestamp_ms'].max())
            
            gps_duration = (gps_time_range[1] - gps_time_range[0]) / 1000.0  # seconds
            odom_duration = (odom_time_range[1] - odom_time_range[0]) / 1000.0  # seconds
            
            self.logger.info(f"  GPS time range: {gps_time_range[0]:.0f} - {gps_time_range[1]:.0f} ms ({gps_duration:.1f}s)")
            self.logger.info(f"  Odom time range: {odom_time_range[0]:.0f} - {odom_time_range[1]:.0f} ms ({odom_duration:.1f}s)")
            
            # Check for time overlap
            overlap_start = max(gps_time_range[0], odom_time_range[0])
            overlap_end = min(gps_time_range[1], odom_time_range[1])
            
            if overlap_end <= overlap_start:
                self.logger.error("No time overlap between GPS and odometry data")
                return False
            
            overlap_duration = (overlap_end - overlap_start) / 1000.0
            self.logger.info(f"  Time overlap: {overlap_duration:.1f}s ({overlap_start:.0f} - {overlap_end:.0f} ms)")
            
            if overlap_duration < 10.0:  # Less than 10 seconds
                self.logger.warning(f"Short time overlap ({overlap_duration:.1f}s) may affect calibration quality")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error loading data: {e}")
            return False
    
    def gps_to_ecef(self, lat, lon, alt):
        """
        Convert GPS coordinates (LLA) to ECEF.
        
        Args:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            alt (float): Altitude in meters
        
        Returns:
            np.ndarray: ECEF coordinates [x, y, z] in meters
        """
        if PYPROJ_AVAILABLE:
            try:
                x, y, z = self.transformer_lla_to_ecef.transform(lon, lat, alt)
                return np.array([x, y, z])
            except Exception as e:
                self.logger.error(f"PyProj coordinate transformation failed: {e}")
                return self._simple_lla_to_ecef(lat, lon, alt)
        else:
            return self._simple_lla_to_ecef(lat, lon, alt)
    
    def _simple_lla_to_ecef(self, lat, lon, alt):
        """Simple LLA to ECEF conversion (fallback when PyProj unavailable)."""
        # WGS84 parameters
        a = 6378137.0  # semi-major axis
        f = 1 / 298.257223563  # flattening
        
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        
        e2 = 2 * f - f * f
        N = a / np.sqrt(1 - e2 * np.sin(lat_rad) ** 2)
        
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1 - e2) + alt) * np.sin(lat_rad)
        
        return np.array([x, y, z])
    
    def interpolate_odometry(self, t_query, odom_data=None):
        """
        Interpolate odometry position at query time.
        
        Args:
            t_query (float): Query time in milliseconds
            odom_data (pd.DataFrame): Odometry data (uses self.odom_data if None)
        
        Returns:
            np.ndarray or None: Interpolated position [x, y, z] or None if outside range
        """
        if odom_data is None:
            odom_data = self.odom_data
        
        if len(odom_data) < 2:
            self.interpolation_misses += 1
            return None
        
        times = odom_data['timestamp_ms'].values
        positions = odom_data[['position_x', 'position_y', 'position_z']].values
        
        if t_query < times[0] or t_query > times[-1]:
            self.interpolation_misses += 1
            return None
        
        self.interpolation_hits += 1
        
        if SCIPY_AVAILABLE:
            try:
                interp_func = interp1d(times, positions, axis=0, kind='linear', assume_sorted=True)
                return interp_func(t_query)
            except Exception as e:
                self.logger.debug(f"SciPy interpolation failed: {e}, using manual interpolation")
        
        # Manual linear interpolation
        idx = np.searchsorted(times, t_query)
        if idx == 0:
            return positions[0]
        if idx == len(times):
            return positions[-1]
        
        t1, t2 = times[idx-1], times[idx]
        p1, p2 = positions[idx-1], positions[idx]
        
        if t2 == t1:
            return p1
        
        w = (t_query - t1) / (t2 - t1)
        return p1 * (1.0 - w) + p2 * w
    
    def optimize_time_offset(self, t_gps, gps_ecef, search_range=None):
        """
        Find optimal time offset for GPS-VIO synchronization.
        
        Args:
            t_gps (float): GPS timestamp in milliseconds
            gps_ecef (np.ndarray): GPS position in ECEF coordinates
            search_range (tuple): (min_offset, max_offset) in seconds
        
        Returns:
            tuple: (best_offset, best_error) where offset is in seconds
        """
        if search_range is None:
            max_search = self.config['time_sync']['max_offset_search']
            search_range = (-max_search, max_search)
        
        # Convert to seconds for search
        t_nominal = (self.time_a * t_gps + self.time_b)
        
        if SCIPY_AVAILABLE:
            def error_function(delta_sec):
                """Error function for optimization (delta in seconds)."""
                t_candidate = t_nominal + delta_sec * 1000.0  # Convert to ms
                vio_pos = self.interpolate_odometry(t_candidate)
                if vio_pos is None:
                    return 1e9
                
                # Transform VIO from odom to earth frame
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
                    bounds=search_range,
                    method='bounded'
                )
                if result.success:
                    return result.x, result.fun
            except Exception as e:
                self.logger.debug(f"SciPy optimization failed: {e}, using grid search")
        
        # Fallback: grid search
        best_delta = 0.0
        best_error = float('inf')
        
        search_points = np.linspace(search_range[0], search_range[1], 41)
        for delta_sec in search_points:
            t_candidate = t_nominal + delta_sec * 1000.0  # Convert to ms
            vio_pos = self.interpolate_odometry(t_candidate)
            if vio_pos is None:
                continue
            
            # Transform VIO from odom to earth frame
            R_eo = self.current_transform[:3, :3]
            t_eo = self.current_transform[:3, 3]
            # Compute odom -> earth transform
            R_oe = R_eo.T
            t_oe = -R_oe @ t_eo
            vio_transformed = R_oe @ vio_pos + t_oe
            
            error = np.linalg.norm(vio_transformed - gps_ecef)
            if error < best_error:
                best_error = error
                best_delta = delta_sec
        
        return best_delta, best_error
    
    def update_time_model(self, t_gps, best_delta):
        """
        Update time synchronization model parameters.
        
        Args:
            t_gps (float): GPS timestamp in milliseconds
            best_delta (float): Optimal time offset in seconds
        """
        alpha = self.config['time_sync']['smoothing_alpha']
        
        # Update offset parameter
        old_b = self.time_b
        implied_b = self.time_b + best_delta * 1000.0  # Convert to ms
        self.time_b = (1.0 - alpha) * self.time_b + alpha * implied_b
        
        # Store correction for drift estimation
        t_corrected = self.time_a * t_gps + self.time_b
        self.gps_corrections.append((t_gps, t_corrected))
        
        # Update drift parameter if we have enough data
        if len(self.gps_corrections) >= 10:
            self._update_drift_parameter()
        
        self.time_sync_iterations += 1
    
    def _update_drift_parameter(self):
        """Estimate clock drift using linear regression."""
        if len(self.gps_corrections) < 10:
            return
        
        times_gps = np.array([t_gps for t_gps, _ in self.gps_corrections])
        times_corrected = np.array([t_corr for _, t_corr in self.gps_corrections])
        
        # Center timestamps for numerical stability
        t_mean = np.mean(times_gps)
        times_gps_centered = times_gps - t_mean
        times_corrected_centered = times_corrected - t_mean
        
        if SCIPY_AVAILABLE:
            try:
                slope, intercept, r_value, _, std_err = linregress(times_gps_centered, times_corrected_centered)
                
                # Update drift parameter with smoothing
                alpha = self.config['time_sync']['smoothing_alpha']
                new_a = slope
                new_b = intercept + t_mean * (1.0 - slope)
                
                self.time_a = (1.0 - alpha) * self.time_a + alpha * new_a
                self.time_b = (1.0 - alpha) * self.time_b + alpha * new_b
                
                drift_ppm = 1e6 * (self.time_a - 1.0)
                self.logger.debug(f"Drift updated: {drift_ppm:.1f}ppm, offset: {self.time_b:.4f}ms, R²: {r_value**2:.4f}")
                
            except Exception as e:
                self.logger.warning(f"Drift estimation failed: {e}")
        else:
            # Simple slope calculation
            if len(times_gps_centered) >= 2:
                dt_gps = times_gps_centered[-1] - times_gps_centered[0]
                dt_corr = times_corrected_centered[-1] - times_corrected_centered[0]
                if abs(dt_gps) > 1e-6:
                    slope = dt_corr / dt_gps
                    alpha = self.config['time_sync']['smoothing_alpha']
                    self.time_a = (1.0 - alpha) * self.time_a + alpha * slope
    
    def rigid_alignment(self, P, Q):
        """
        Compute rigid alignment between point sets P (VIO) and Q (GPS/ECEF).
        
        Args:
            P (np.ndarray): VIO positions in odom frame (N x 3)
            Q (np.ndarray): GPS positions in ECEF frame (N x 3)
        
        Returns:
            tuple: (R_matrix, t_vector, rms_error) where R and t transform P to Q
        """
        if len(P) != len(Q):
            raise ValueError(f"Point sets size mismatch: P={len(P)}, Q={len(Q)}")
        
        if len(P) < 3:
            raise ValueError(f"Need at least 3 corresponding points, got {len(P)}")
        
        # Remove outliers
        P_clean, Q_clean = self._remove_outliers(P, Q)
        
        if len(P_clean) < 3:
            raise ValueError(f"Too few points after outlier removal: {len(P_clean)}")
        
        if SCIPY_AVAILABLE:
            return self._scipy_rigid_alignment(P_clean, Q_clean)
        else:
            return self._simple_rigid_alignment(P_clean, Q_clean)
    
    def _remove_outliers(self, P, Q):
        """Remove outlier pairs based on residual distance."""
        if np.allclose(self.current_transform, np.eye(4)):
            # No outlier removal on first iteration
            return P, Q
        
        # Transform VIO points using current transform (odom -> earth)
        R_eo = self.current_transform[:3, :3]
        t_eo = self.current_transform[:3, 3]
        R_oe = R_eo.T
        t_oe = -R_oe @ t_eo
        
        residuals = []
        for i in range(len(P)):
            p_transformed = R_oe @ P[i] + t_oe
            residual = np.linalg.norm(Q[i] - p_transformed)
            residuals.append(residual)
        
        residuals = np.array(residuals)
        
        # Remove points beyond threshold sigma
        mean_residual = np.mean(residuals)
        std_residual = np.std(residuals)
        
        if std_residual > 1e-6:  # Avoid division by zero
            threshold = mean_residual + self.config['calibration']['outlier_threshold'] * std_residual
            valid_mask = residuals <= threshold
            removed_count = len(P) - np.sum(valid_mask)
            if removed_count > 0:
                self.logger.debug(f"Removed {removed_count}/{len(P)} outlier points (threshold: {threshold:.3f}m)")
            return P[valid_mask], Q[valid_mask]
        else:
            return P, Q
    
    def _scipy_rigid_alignment(self, P, Q):
        """Rigid alignment using SciPy's Rotation.align_vectors."""
        # Center the point sets
        mu_P = P.mean(axis=0)
        mu_Q = Q.mean(axis=0)
        P_centered = P - mu_P
        Q_centered = Q - mu_Q
        
        # Use SciPy for robust rotation estimation
        rot_obj, rssd = R.align_vectors(Q_centered, P_centered)
        R_matrix = rot_obj.as_matrix()
        
        # Compute translation
        t_vector = mu_Q - R_matrix @ mu_P
        
        # Compute RMS error
        P_aligned = (R_matrix @ P.T).T + t_vector
        residuals = Q - P_aligned
        rms_error = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        
        return R_matrix, t_vector, rms_error
    
    def _simple_rigid_alignment(self, P, Q):
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
        
        # Compute translation
        t_vector = mu_Q - R_matrix @ mu_P
        
        # Compute RMS error
        P_aligned = (R_matrix @ P.T).T + t_vector
        residuals = Q - P_aligned
        rms_error = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        
        return R_matrix, t_vector, rms_error
    
    def calibrate(self):
        """
        Perform complete GPS-VIO calibration.
        
        Returns:
            dict: Calibration results including transform, quality metrics, and statistics
        """
        if self.gps_data is None or self.odom_data is None:
            raise ValueError("Data not loaded. Call load_data() first.")
        
        self.logger.info("Starting GPS-VIO offline calibration...")
        
        # Reset state
        self.reset_state()
        
        # Phase 1: Time synchronization and pair collection
        self.logger.info("Phase 1: Time synchronization and data pairing")
        self._collect_synchronized_pairs()
        
        if len(self.sync_pairs) < self.config['calibration']['min_pairs']:
            raise RuntimeError(f"Insufficient synchronized pairs: {len(self.sync_pairs)} < {self.config['calibration']['min_pairs']}")
        
        # Phase 2: Spatial calibration
        self.logger.info("Phase 2: Spatial calibration")
        self._perform_spatial_calibration()
        
        # Phase 3: Results compilation and validation
        self.logger.info("Phase 3: Results compilation and validation")
        self._compile_results()
        
        self.logger.info(f"Calibration completed successfully!")
        self.logger.info(f"  Final RMS error: {self.final_quality:.3f}m")
        self.logger.info(f"  Synchronized pairs: {len(self.sync_pairs)}")
        self.logger.info(f"  Time sync iterations: {self.time_sync_iterations}")
        self.logger.info(f"  Calibration converged: {self.calibration_converged}")
        
        return self.calibration_results
    
    def _collect_synchronized_pairs(self):
        """Collect GPS-VIO synchronized pairs using iterative time synchronization."""
        min_motion = self.config['time_sync']['min_motion_threshold']
        
        synchronized_count = 0
        processed_count = 0
        
        for _, gps_row in self.gps_data.iterrows():
            processed_count += 1
            
            # Convert GPS to ECEF
            gps_ecef = self.gps_to_ecef(gps_row['latitude'], gps_row['longitude'], gps_row['altitude'])
            t_gps = gps_row['timestamp_ms']
            
            # Time synchronization optimization
            best_delta, best_error = self.optimize_time_offset(t_gps, gps_ecef)
            
            if best_error < float('inf'):
                # Update time model
                self.update_time_model(t_gps, best_delta)
                
                # Get synchronized VIO position
                t_vio_corrected = self.time_a * t_gps + self.time_b
                vio_pos = self.interpolate_odometry(t_vio_corrected)
                
                if vio_pos is not None:
                    # Check motion requirement
                    if len(self.sync_pairs) == 0:
                        # First pair
                        self.sync_pairs.append((vio_pos, gps_ecef, t_gps, t_vio_corrected))
                        synchronized_count += 1
                    else:
                        motion = np.linalg.norm(vio_pos - self.sync_pairs[-1][0])
                        if motion > min_motion:
                            self.sync_pairs.append((vio_pos, gps_ecef, t_gps, t_vio_corrected))
                            synchronized_count += 1
            
            # Progress reporting
            if processed_count % 100 == 0:
                progress = 100 * processed_count / len(self.gps_data)
                self.logger.info(f"  Progress: {progress:.1f}% ({processed_count}/{len(self.gps_data)}) - Synchronized: {synchronized_count}")
        
        self.logger.info(f"Synchronization complete: {synchronized_count}/{processed_count} pairs collected")
        
        # Store processed pairs for analysis
        self.processed_pairs = [(vio, gps, t_gps, t_vio) for vio, gps, t_gps, t_vio in self.sync_pairs]
    
    def _perform_spatial_calibration(self):
        """Perform iterative spatial calibration."""
        if len(self.sync_pairs) < self.config['calibration']['min_pairs']:
            raise RuntimeError(f"Insufficient pairs for calibration: {len(self.sync_pairs)}")
        
        # Extract positions
        vio_positions = np.array([pair[0] for pair in self.sync_pairs])
        gps_positions = np.array([pair[1] for pair in self.sync_pairs])
        
        # Perform rigid alignment
        try:
            R_matrix, t_vector, rms_error = self.rigid_alignment(vio_positions, gps_positions)
            
            # The rigid_alignment computes transform from odom to earth
            # But we store earth -> odom transform for consistency with ROS convention
            R_earth_odom = R_matrix.T
            t_earth_odom = -R_earth_odom @ t_vector
            
            self.current_transform[:3, :3] = R_earth_odom
            self.current_transform[:3, 3] = t_earth_odom
            self.calibration_quality = rms_error
            
            # Check convergence
            if rms_error < self.config['calibration']['convergence_threshold']:
                self.calibration_converged = True
            
            # Store final results
            self.final_transform = self.current_transform.copy()
            self.final_quality = rms_error
            
            self.logger.info(f"Spatial calibration complete:")
            self.logger.info(f"  RMS error: {rms_error:.3f}m")
            self.logger.info(f"  Translation norm: {np.linalg.norm(t_vector):.3f}m")
            
            # Extract rotation angle
            trace_R = np.trace(R_matrix)
            angle_rad = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
            angle_deg = np.degrees(angle_rad)
            self.logger.info(f"  Rotation angle: {angle_deg:.2f}°")
            self.logger.info(f"  Converged: {self.calibration_converged}")
            
        except Exception as e:
            self.logger.error(f"Spatial calibration failed: {e}")
            raise
    
    def _compile_results(self):
        """Compile final calibration results."""
        self.calibration_results = {
            'timestamp': datetime.now().isoformat(),
            'success': True,
            'transform': {
                'matrix': self.final_transform.tolist(),
                'translation': self.final_transform[:3, 3].tolist(),
                'rotation_matrix': self.final_transform[:3, :3].tolist(),
                'translation_norm_m': float(np.linalg.norm(self.final_transform[:3, 3])),
            },
            'time_synchronization': {
                'drift_parameter_a': float(self.time_a),
                'offset_parameter_b_ms': float(self.time_b),
                'drift_rate_ppm': float(1e6 * (self.time_a - 1.0)),
                'iterations': int(self.time_sync_iterations)
            },
            'quality_metrics': {
                'rms_error_m': float(self.final_quality),
                'converged': bool(self.calibration_converged),
                'convergence_threshold_m': self.config['calibration']['convergence_threshold'],
                'synchronized_pairs': len(self.sync_pairs)
            },
            'statistics': {
                'gps_points_processed': len(self.gps_data),
                'odom_points_available': len(self.odom_data),
                'interpolation_hits': self.interpolation_hits,
                'interpolation_misses': self.interpolation_misses,
                'interpolation_success_rate': 100.0 * self.interpolation_hits / max(1, self.interpolation_hits + self.interpolation_misses)
            },
            'data_info': {
                'gps_time_range_ms': [float(self.gps_data['timestamp_ms'].min()), float(self.gps_data['timestamp_ms'].max())],
                'odom_time_range_ms': [float(self.odom_data['timestamp_ms'].min()), float(self.odom_data['timestamp_ms'].max())],
            },
            'configuration': self.config
        }
        
        # Add rotation analysis
        if SCIPY_AVAILABLE:
            try:
                rot_obj = R.from_matrix(self.final_transform[:3, :3])
                euler_angles = rot_obj.as_euler('xyz', degrees=True)
                self.calibration_results['transform']['rotation_euler_deg'] = euler_angles.tolist()
                
                quat = rot_obj.as_quat()  # [x, y, z, w]
                self.calibration_results['transform']['rotation_quaternion'] = quat.tolist()
            except Exception as e:
                self.logger.warning(f"Failed to compute rotation representations: {e}")
    
    def save_results(self, output_dir, prefix="gps_vio_calibration"):
        """
        Save calibration results to files.
        
        Args:
            output_dir (str): Output directory path
            prefix (str): Filename prefix for output files
        
        Returns:
            dict: Paths to created files
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        created_files = {}
        
        # Save JSON results
        json_path = output_dir / f"{prefix}_{timestamp}.json"
        with open(json_path, 'w') as f:
            json.dump(self.calibration_results, f, indent=2)
        created_files['results_json'] = str(json_path)
        
        # Save transform matrix
        transform_path = output_dir / f"{prefix}_transform_{timestamp}.txt"
        with open(transform_path, 'w') as f:
            f.write(f"GPS-VIO Calibration Transform Matrix (Earth -> Odom)\n")
            f.write(f"Generated: {datetime.now().isoformat()}\n")
            f.write(f"RMS Error: {self.final_quality:.6f} meters\n")
            f.write(f"Converged: {self.calibration_converged}\n\n")
            f.write("4x4 Homogeneous Transform Matrix:\n")
            np.savetxt(f, self.final_transform, fmt='%12.8f')
            f.write(f"\nTranslation (meters): [{self.final_transform[0,3]:.6f}, {self.final_transform[1,3]:.6f}, {self.final_transform[2,3]:.6f}]\n")
            f.write(f"Translation Norm: {np.linalg.norm(self.final_transform[:3, 3]):.6f} meters\n")
        created_files['transform_matrix'] = str(transform_path)
        
        # Save synchronized pairs data
        pairs_path = output_dir / f"{prefix}_pairs_{timestamp}.csv"
        pairs_df = pd.DataFrame([
            {
                'vio_x': pair[0][0], 'vio_y': pair[0][1], 'vio_z': pair[0][2],
                'gps_x': pair[1][0], 'gps_y': pair[1][1], 'gps_z': pair[1][2],
                'gps_time_ms': pair[2], 'vio_time_ms': pair[3]
            }
            for pair in self.processed_pairs
        ])
        pairs_df.to_csv(pairs_path, index=False)
        created_files['synchronized_pairs'] = str(pairs_path)
        
        # Save visualization plots
        if MATPLOTLIB_AVAILABLE and self.config['output']['save_plots']:
            plots_path = self._generate_plots(output_dir, prefix, timestamp)
            created_files.update(plots_path)
        
        self.logger.info(f"Results saved to {output_dir}:")
        for file_type, file_path in created_files.items():
            self.logger.info(f"  {file_type}: {file_path}")
        
        return created_files
    
    def _generate_plots(self, output_dir, prefix, timestamp):
        """Generate visualization plots for calibration analysis."""
        if not MATPLOTLIB_AVAILABLE:
            return {}
        
        created_plots = {}
        
        try:
            # Plot 1: Time synchronization analysis
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
            fig.suptitle('GPS-VIO Calibration Analysis', fontsize=16)
            
            # Time differences
            if len(self.processed_pairs) > 0:
                time_diffs = [(pair[3] - pair[2]) / 1000.0 for pair in self.processed_pairs]  # Convert to seconds
                ax1.plot(time_diffs, 'b-', alpha=0.7)
                ax1.set_title('VIO-GPS Time Differences')
                ax1.set_xlabel('Sample Index')
                ax1.set_ylabel('Time Difference (seconds)')
                ax1.grid(True, alpha=0.3)
            
            # Spatial alignment errors
            if len(self.sync_pairs) > 0:
                vio_positions = np.array([pair[0] for pair in self.sync_pairs])
                gps_positions = np.array([pair[1] for pair in self.sync_pairs])
                
                # Transform VIO positions to ECEF for comparison
                R_eo = self.final_transform[:3, :3]
                t_eo = self.final_transform[:3, 3]
                R_oe = R_eo.T
                t_oe = -R_oe @ t_eo
                
                vio_transformed = (R_oe @ vio_positions.T).T + t_oe
                errors = np.linalg.norm(gps_positions - vio_transformed, axis=1)
                
                ax2.plot(errors, 'r-', alpha=0.7)
                ax2.axhline(y=self.final_quality, color='k', linestyle='--', label=f'RMS: {self.final_quality:.3f}m')
                ax2.set_title('Spatial Alignment Errors')
                ax2.set_xlabel('Pair Index')  
                ax2.set_ylabel('Error (meters)')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
            
            # Trajectory comparison (2D projection)
            if len(self.sync_pairs) > 0:
                vio_proj = vio_transformed[:, :2]  # X-Y projection
                gps_proj = gps_positions[:, :2]
                
                ax3.plot(vio_proj[:, 0], vio_proj[:, 1], 'b-', label='VIO (transformed)', alpha=0.7)
                ax3.plot(gps_proj[:, 0], gps_proj[:, 1], 'r-', label='GPS (ECEF)', alpha=0.7)
                ax3.set_title('Trajectory Comparison (X-Y Projection)')
                ax3.set_xlabel('ECEF X (meters)')
                ax3.set_ylabel('ECEF Y (meters)')
                ax3.legend()
                ax3.grid(True, alpha=0.3)
                ax3.axis('equal')
            
            # Statistics summary
            ax4.axis('off')
            stats_text = f"""
Calibration Statistics:
• RMS Error: {self.final_quality:.3f} m
• Synchronized Pairs: {len(self.sync_pairs)}
• Converged: {self.calibration_converged}
• Time Sync Iterations: {self.time_sync_iterations}
• Drift Rate: {1e6 * (self.time_a - 1.0):.1f} ppm
• Interpolation Success: {100.0 * self.interpolation_hits / max(1, self.interpolation_hits + self.interpolation_misses):.1f}%

Transform Summary:
• Translation Norm: {np.linalg.norm(self.final_transform[:3, 3]):.3f} m
• Translation: [{self.final_transform[0,3]:.2f}, {self.final_transform[1,3]:.2f}, {self.final_transform[2,3]:.2f}]
            """
            ax4.text(0.05, 0.95, stats_text.strip(), transform=ax4.transAxes, 
                    fontsize=11, verticalalignment='top', fontfamily='monospace')
            
            plt.tight_layout()
            
            plots_path = output_dir / f"{prefix}_analysis_{timestamp}.png"
            plt.savefig(plots_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            created_plots['analysis_plot'] = str(plots_path)
            
        except Exception as e:
            self.logger.warning(f"Failed to generate plots: {e}")
        
        return created_plots


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(
        description="GPS-VIO Offline Calibration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Using individual files
  python3 gps_vio_offline_calibration.py --gps_data gps_data.csv --odom_data odom_data.csv

  # Using data directory (looks for gps_data_extracted.csv and odom_data_extracted.csv)
  python3 gps_vio_offline_calibration.py --data_dir /path/to/results/odom_output

  # With custom output directory and configuration
  python3 gps_vio_offline_calibration.py --data_dir ./results/odom_output --output_dir ./calibration_results --config config.json

Output Files:
  - gps_vio_calibration_TIMESTAMP.json: Complete calibration results
  - gps_vio_calibration_transform_TIMESTAMP.txt: Transform matrix
  - gps_vio_calibration_pairs_TIMESTAMP.csv: Synchronized data pairs
  - gps_vio_calibration_analysis_TIMESTAMP.png: Visualization plots (if matplotlib available)
        """
    )
    
    # Input data arguments
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--data_dir', type=str,
                           help='Directory containing gps_data_extracted.csv and odom_data_extracted.csv')
    input_group.add_argument('--individual_files', action='store_true',
                           help='Use individual GPS and odometry files (requires --gps_data and --odom_data)')
    
    parser.add_argument('--gps_data', type=str,
                       help='Path to GPS data CSV file (required with --individual_files)')
    parser.add_argument('--odom_data', type=str,
                       help='Path to odometry data CSV file (required with --individual_files)')
    
    # Output arguments
    parser.add_argument('--output_dir', type=str, default='./calibration_results',
                       help='Output directory for results (default: ./calibration_results)')
    
    # Configuration arguments
    parser.add_argument('--config', type=str,
                       help='Path to JSON configuration file')
    parser.add_argument('--no_plots', action='store_true',
                       help='Disable plot generation')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.individual_files:
        if not args.gps_data or not args.odom_data:
            parser.error("--individual_files requires both --gps_data and --odom_data")
    
    # Setup logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Load configuration
    config = {}
    if args.config:
        try:
            with open(args.config, 'r') as f:
                config = json.load(f)
            print(f"📝 Configuration loaded from: {args.config}")
        except Exception as e:
            print(f"❌ Failed to load configuration: {e}")
            return 1
    
    # Apply command line overrides
    if args.no_plots:
        if 'output' not in config:
            config['output'] = {}
        config['output']['save_plots'] = False
    
    try:
        # Initialize calibrator
        print("🚀 GPS-VIO Offline Calibration")
        print("=" * 60)
        
        calibrator = GPSVIOOfflineCalibrator(config=config)
        
        # Load data
        print("📂 Loading data...")
        if args.data_dir:
            success = calibrator.load_data(data_dir=args.data_dir)
        else:
            success = calibrator.load_data(gps_file=args.gps_data, odom_file=args.odom_data)
        
        if not success:
            print("❌ Failed to load data")
            return 1
        
        # Perform calibration
        print("\n🔧 Performing calibration...")
        results = calibrator.calibrate()
        
        # Save results
        print(f"\n💾 Saving results to: {args.output_dir}")
        saved_files = calibrator.save_results(args.output_dir)
        
        # Print summary
        print("\n✅ Calibration completed successfully!")
        print(f"🎯 RMS Error: {results['quality_metrics']['rms_error_m']:.3f} meters")
        print(f"📊 Synchronized Pairs: {results['quality_metrics']['synchronized_pairs']}")
        print(f"🎚️  Converged: {results['quality_metrics']['converged']}")
        print(f"⏱️  Time Drift: {results['time_synchronization']['drift_rate_ppm']:.1f} ppm")
        
        return 0
        
    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())