#!/usr/bin/env python3
"""
Unified ROS2 Bag Data Extractor

This script combines GPS and odometry data extraction from ROS2 bag files.
It handles both .db3 files and bag directories using metadata-first approach.

Features:
- Extracts GPS data (sensor_msgs/NavSatFix) and odometry data (nav_msgs/Odometry)
- Automatic topic detection from metadata.yaml
- Support for both .db3 files and bag folders  
- Outputs data in CSV and JSON formats using pandas
- Efficient memory management for large bag files
- Comprehensive data validation and statistics

Dependencies:
- pandas: For efficient data manipulation and export
- numpy: For numerical operations  
- pyyaml: For metadata parsing
- rosbag2_py: For reading ROS2 bag files
- rclpy: For ROS2 message deserialization

Usage:
    python3 extract_bag_data.py --bag_path /path/to/bag --output_dir ./output
    python3 extract_bag_data.py --bag_path bag.db3 --gps_topic /cbs_gnss --odom_topic /odom

Author: Unified Data Extraction System
Created: September 2025
Version: 1.0
"""

import argparse
import os
import sys
import json
import pandas as pd
import numpy as np
import yaml
import logging
from datetime import datetime
from pathlib import Path
import signal
import time

# Progress bar and color imports
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False
    print("⚠️  tqdm not available, progress bars will be disabled. Install with: pip install tqdm")

try:
    from colorama import init, Fore, Back, Style
    init()  # Initialize colorama
    COLORAMA_AVAILABLE = True
except ImportError:
    COLORAMA_AVAILABLE = False
    # Fallback color codes for basic terminals
    class Fore:
        RED = '\033[31m'
        GREEN = '\033[32m'
        YELLOW = '\033[33m'
        BLUE = '\033[34m'
        MAGENTA = '\033[35m'
        CYAN = '\033[36m'
        WHITE = '\033[37m'
        RESET = '\033[0m'
    class Style:
        BRIGHT = '\033[1m'
        DIM = '\033[2m'
        RESET_ALL = '\033[0m'

# ROS2 imports
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class UnifiedBagDataExtractor:
    """Unified extractor for GPS and odometry data from ROS2 bag files."""
    
    def __init__(self, bag_path, output_dir="results", enable_logging=False, log_dir=None, enable_progress=True):
        """
        Initialize unified data extractor.
        
        Args:
            bag_path (str): Path to ROS2 bag file or directory
            output_dir (str): Output directory for extracted data
            enable_logging (bool): Enable logging to file
            log_dir (str): Directory for log files
        """
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.gps_data = []
        self.odom_data = []
        self.bag_metadata = {}
        
        # Statistics tracking
        self.extraction_stats = {
            'gps_topics': {},
            'odom_topics': {},
            'extraction_start_time': None,
            'extraction_end_time': None,
            'total_messages_processed': 0,
            'parsing_errors': 0
        }
        
        # Progress bar settings
        self.enable_progress = enable_progress and TQDM_AVAILABLE
        self.progress_bar = None
        self.cancel_requested = False
        
        # Memory monitoring threshold (in bytes)
        self.memory_warning_threshold = 1024 * 1024 * 1024  # 1GB default
        
        # Setup signal handler for graceful cancellation
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Ensure output directory exists
        os.makedirs(output_dir, exist_ok=True)
        
        # Set up logging if enabled
        self.logger = None
        self.log_file_path = None
        if enable_logging:
            self._setup_logging(log_dir)
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        self.cancel_requested = True
        self._log("\n⚠️  Cancellation requested. Finishing current operations...", 'WARNING')
        if self.progress_bar:
            self.progress_bar.close()
        print(f"\n{Fore.YELLOW}🛑 Operation cancelled by user{Style.RESET_ALL}")
    
    def _colorize(self, text, color=None, style=None):
        """Add color and style to text if colorama is available."""
        if not COLORAMA_AVAILABLE:
            return text
        
        result = ''
        if style:
            result += style
        if color:
            result += color
        result += text
        if color or style:
            result += Style.RESET_ALL
        return result
    
    def _check_memory_usage(self):
        """Check current memory usage and warn if high."""
        try:
            import psutil
            process = psutil.Process(os.getpid())
            memory_usage = process.memory_info().rss
            
            if memory_usage > self.memory_warning_threshold:
                memory_mb = memory_usage / (1024 * 1024)
                self._log(f"⚠️  High memory usage detected: {memory_mb:.0f} MB", 'WARNING')
                return True
            return False
        except ImportError:
            # psutil not available, skip memory monitoring
            return False
    
    def _validate_gps_data(self, latitude, longitude, altitude):
        """Validate GPS coordinates are within reasonable bounds."""
        if latitude is None or longitude is None:
            return False, "Missing coordinates"
        
        # Check basic coordinate bounds
        if not (-90 <= latitude <= 90):
            return False, f"Invalid latitude: {latitude}"
        if not (-180 <= longitude <= 180):
            return False, f"Invalid longitude: {longitude}"
        
        # Check for obviously invalid values
        if latitude == 0 and longitude == 0:
            return False, "Null island coordinates (0,0)"
        
        if altitude is not None and (altitude < -1000 or altitude > 50000):
            return False, f"Unrealistic altitude: {altitude}m"
        
        return True, "Valid"
    
    def _validate_odom_data(self, pose_data):
        """Validate odometry pose data."""
        if 'position' not in pose_data:
            return False, "Missing position data"
        
        pos = pose_data['position']
        if any(abs(pos[axis]) > 100000 for axis in ['x', 'y', 'z']):  # 100km limit
            return False, "Position values too large"
        
        if 'orientation' in pose_data:
            quat = pose_data['orientation']
            # Check quaternion normalization (should be close to 1)
            norm = (quat['x']**2 + quat['y']**2 + quat['z']**2 + quat['w']**2)**0.5
            if abs(norm - 1.0) > 0.1:
                return False, f"Quaternion not normalized: {norm}"
        
        return True, "Valid"
    
    def _setup_logging(self, log_dir=None):
        """
        Set up logging to file with timestamp.
        
        Args:
            log_dir (str): Directory for log files, defaults to geodetic-points/log
        """
        if log_dir is None:
            # Default to geodetic-points/log directory
            script_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            log_dir = os.path.join(script_dir, 'log')
        
        # Ensure log directory exists
        os.makedirs(log_dir, exist_ok=True)
        
        # Create timestamped log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = os.path.basename(self.bag_path).replace('.db3', '').replace('/', '_')
        log_filename = f'extract_bag_data_{bag_name}_{timestamp}.log'
        self.log_file_path = os.path.join(log_dir, log_filename)
        
        # Set up logger
        self.logger = logging.getLogger('ExtractBagData')
        self.logger.setLevel(logging.INFO)
        
        # Remove existing handlers
        for handler in self.logger.handlers[:]:
            self.logger.removeHandler(handler)
        
        # File handler
        file_handler = logging.FileHandler(self.log_file_path, encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        
        self.logger.addHandler(file_handler)
        
        # Initial log entry
        self.logger.info(f"Extraction session started for bag: {self.bag_path}")
        self.logger.info(f"Output directory: {self.output_dir}")
        
        print(f"📝 Logging enabled: {self.log_file_path}")
    
    def _log(self, message, level='INFO'):
        """Log message to file if logging is enabled."""
        if self.logger:
            if level == 'ERROR':
                self.logger.error(message)
            elif level == 'WARNING':
                self.logger.warning(message)
            else:
                self.logger.info(message)
        print(message)
    
    def read_bag_metadata(self):
        """
        Read bag metadata to identify available topics.
        Based on bag2kml.py metadata reading logic.
        
        Returns:
            dict: Metadata containing topic information
        """
        # Handle both .db3 files and folder paths
        if self.bag_path.endswith('.db3'):
            metadata_path = os.path.join(os.path.dirname(self.bag_path), 'metadata.yaml')
        else:
            metadata_path = os.path.join(self.bag_path, 'metadata.yaml')
        
        gps_topics = {}
        odom_topics = {}
        
        if os.path.exists(metadata_path):
            try:
                with open(metadata_path, 'r', encoding='utf-8') as f:
                    metadata = yaml.safe_load(f)
                
                self._log(f"✅ Successfully read metadata: {metadata_path}")
                
                if 'rosbag2_bagfile_information' in metadata:
                    bag_info = metadata['rosbag2_bagfile_information']
                    
                    if 'topics_with_message_count' in bag_info:
                        self._log(f"Available topics in bag:")
                        
                        for topic_info in bag_info['topics_with_message_count']:
                            topic_meta = topic_info['topic_metadata']
                            topic_name = topic_meta['name']
                            topic_type = topic_meta['type']
                            message_count = topic_info['message_count']
                            
                            if topic_type == 'sensor_msgs/msg/NavSatFix':
                                gps_topics[topic_name] = {
                                    'type': topic_type,
                                    'expected_count': message_count,
                                    'serialization_format': topic_meta.get('serialization_format', 'cdr')
                                }
                                self._log(f"  📍 GPS: {topic_name} ({message_count} messages)")
                                # Initialize stats tracking
                                self.extraction_stats['gps_topics'][topic_name] = {
                                    'expected_count': message_count,
                                    'extracted_count': 0,
                                    'valid_count': 0
                                }
                                
                            elif topic_type == 'nav_msgs/msg/Odometry':
                                odom_topics[topic_name] = {
                                    'type': topic_type,
                                    'expected_count': message_count,
                                    'serialization_format': topic_meta.get('serialization_format', 'cdr')
                                }
                                self._log(f"  🗺️  ODOM: {topic_name} ({message_count} messages)")
                                # Initialize stats tracking
                                self.extraction_stats['odom_topics'][topic_name] = {
                                    'expected_count': message_count,
                                    'extracted_count': 0,
                                    'valid_count': 0
                                }
                            else:
                                self._log(f"  ➖ OTHER: {topic_name} ({topic_type}, {message_count} messages)")
                
                if not gps_topics and not odom_topics:
                    self._log("⚠️  No GPS or odometry topics found in metadata", 'WARNING')
                else:
                    self._log(f"✅ Found {len(gps_topics)} GPS topics and {len(odom_topics)} odometry topics")
                    
            except Exception as e:
                self._log(f"❌ Failed to read metadata: {e}", 'ERROR')
        else:
            self._log(f"⚠️  Metadata file not found: {metadata_path}", 'WARNING')
        
        self.bag_metadata = {
            'gps_topics': gps_topics,
            'odom_topics': odom_topics,
            'metadata_path': metadata_path
        }
        
        return self.bag_metadata
    
    def read_ros2_bag(self, target_gps_topics=None, target_odom_topics=None, storage_id='sqlite3'):
        """
        Read ROS2 bag file and extract GPS and odometry messages.
        Based on bag2kml.py bag reading logic.
        
        Args:
            target_gps_topics (list): List of GPS topic names to extract
            target_odom_topics (list): List of odometry topic names to extract
            storage_id (str): Storage format of the bag file
        
        Returns:
            dict: Dictionary with GPS and odometry messages
        """
        # Handle both .db3 files and folder paths
        bag_path = self.bag_path
        if bag_path.endswith('.db3'):
            bag_path = os.path.dirname(bag_path)
        
        self._log(f"Opening bag: {bag_path}")
        self.extraction_stats['extraction_start_time'] = datetime.now()
        
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            ),
        )
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        
        # Build topic type mapping
        topic_type_map = {}
        for topic_info in topic_types:
            topic_type_map[topic_info.name] = topic_info.type
        
        # Determine which topics to extract
        if target_gps_topics is None:
            target_gps_topics = list(self.bag_metadata.get('gps_topics', {}).keys())
        
        if target_odom_topics is None:
            target_odom_topics = list(self.bag_metadata.get('odom_topics', {}).keys())
        
        if not target_gps_topics and not target_odom_topics:
            self._log("❌ No target topics specified", 'ERROR')
            return {'gps': {}, 'odom': {}}
        
        self._log(f"Target GPS topics: {target_gps_topics}")
        self._log(f"Target odometry topics: {target_odom_topics}")
        
        # Validate topics exist
        all_target_topics = target_gps_topics + target_odom_topics
        for topic in all_target_topics:
            if topic not in topic_type_map:
                self._log(f"⚠️  Warning: Topic {topic} not found in bag", 'WARNING')
        
        # Collect messages
        gps_results = {topic: [] for topic in target_gps_topics}
        odom_results = {topic: [] for topic in target_odom_topics}
        
        gps_counts = {topic: 0 for topic in target_gps_topics}
        odom_counts = {topic: 0 for topic in target_odom_topics}
        
        # Get message types for topics
        msg_types = {}
        for topic in all_target_topics:
            if topic in topic_type_map:
                try:
                    msg_types[topic] = get_message(topic_type_map[topic])
                except Exception as e:
                    self._log(f"❌ Failed to get message type for {topic}: {e}", 'ERROR')
        
        # Calculate total expected messages for progress bar
        total_expected_messages = 0
        for topic in target_gps_topics:
            if topic in self.extraction_stats['gps_topics']:
                total_expected_messages += self.extraction_stats['gps_topics'][topic]['expected_count']
        for topic in target_odom_topics:
            if topic in self.extraction_stats['odom_topics']:
                total_expected_messages += self.extraction_stats['odom_topics'][topic]['expected_count']
        
        self._log(f"Reading messages from bag... (Expected: {total_expected_messages} messages)")
        
        # Initialize progress bar if enabled
        if self.enable_progress and total_expected_messages > 0:
            self.progress_bar = tqdm(
                total=total_expected_messages,
                desc=f"{self._colorize('📦 Reading bag', Fore.CYAN, Style.BRIGHT)}",
                unit="msg",
                unit_scale=True,
                colour='cyan',
                bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}, {rate_fmt}]',
                ncols=100
            )
        elif self.enable_progress:
            # If no expected count available, use indeterminate progress
            self.progress_bar = tqdm(
                desc=f"{self._colorize('📦 Reading bag', Fore.CYAN, Style.BRIGHT)}",
                unit="msg",
                unit_scale=True,
                colour='cyan',
                bar_format='{l_bar}| {n_fmt} [{elapsed}, {rate_fmt}]',
                ncols=100
            )
        
        messages_processed = 0
        last_update_time = time.time()
        
        while reader.has_next() and not self.cancel_requested:
            topic, data, timestamp = reader.read_next()
            
            if topic in target_gps_topics and topic in msg_types:
                try:
                    msg = deserialize_message(data, msg_types[topic])
                    gps_results[topic].append({
                        'bag_timestamp': timestamp,
                        'message': msg
                    })
                    gps_counts[topic] += 1
                    messages_processed += 1
                    
                    # Update progress bar
                    if self.progress_bar:
                        self.progress_bar.update(1)
                        # Update description with current topic info
                        current_time = time.time()
                        if current_time - last_update_time > 0.5:  # Update every 0.5 seconds
                            self.progress_bar.set_description(
                                f"{self._colorize('📍 GPS', Fore.GREEN)} {topic.split('/')[-1]}"
                            )
                            last_update_time = current_time
                    
                    # Log less frequently to reduce noise
                    if gps_counts[topic] % 500 == 0:
                        self._log(f"  GPS {topic}: {gps_counts[topic]} messages...")
                        
                except Exception as e:
                    self.extraction_stats['parsing_errors'] += 1
                    self._log(f"Failed to parse GPS message from {topic}: {e}", 'ERROR')
                    if self.extraction_stats['parsing_errors'] > 100:
                        self._log("Too many parsing errors, stopping extraction", 'ERROR')
                        break
                    continue
            
            elif topic in target_odom_topics and topic in msg_types:
                try:
                    msg = deserialize_message(data, msg_types[topic])
                    odom_results[topic].append({
                        'bag_timestamp': timestamp,
                        'message': msg
                    })
                    odom_counts[topic] += 1
                    messages_processed += 1
                    
                    # Update progress bar
                    if self.progress_bar:
                        self.progress_bar.update(1)
                        # Update description with current topic info
                        current_time = time.time()
                        if current_time - last_update_time > 0.5:  # Update every 0.5 seconds
                            self.progress_bar.set_description(
                                f"{self._colorize('🗺️  ODOM', Fore.BLUE)} {topic.split('/')[-1]}"
                            )
                            last_update_time = current_time
                    
                    # Log less frequently to reduce noise
                    if odom_counts[topic] % 2000 == 0:
                        self._log(f"  ODOM {topic}: {odom_counts[topic]} messages...")
                        
                except Exception as e:
                    self.extraction_stats['parsing_errors'] += 1
                    self._log(f"Failed to parse odometry message from {topic}: {e}", 'ERROR')
                    if self.extraction_stats['parsing_errors'] > 100:
                        self._log("Too many parsing errors, stopping extraction", 'ERROR')
                        break
                    continue
        
        # Close progress bar
        if self.progress_bar:
            self.progress_bar.close()
            
        del reader
        
        self.extraction_stats['extraction_end_time'] = datetime.now()
        self.extraction_stats['total_messages_processed'] = messages_processed
        
        if self.cancel_requested:
            self._log("Extraction cancelled by user", 'WARNING')
            return {'gps': gps_results, 'odom': odom_results}
        
        # Print completion summary with colors
        if messages_processed > 0:
            duration = (self.extraction_stats['extraction_end_time'] - self.extraction_stats['extraction_start_time']).total_seconds()
            rate = messages_processed / duration if duration > 0 else 0
            
            print(f"\n{self._colorize('🎉 Bag reading completed!', Fore.GREEN, Style.BRIGHT)}")
            print(f"📊 Processed {self._colorize(str(messages_processed), Fore.CYAN, Style.BRIGHT)} messages in {self._colorize(f'{duration:.1f}s', Fore.YELLOW)} ({self._colorize(f'{rate:.0f} msg/s', Fore.MAGENTA)})")
            if self.extraction_stats['parsing_errors'] > 0:
                print(f"⚠️  {self._colorize(str(self.extraction_stats['parsing_errors']), Fore.RED)} parsing errors encountered")
        
        # Update extraction statistics and print summary
        for topic in target_gps_topics:
            extracted_count = gps_counts[topic]
            if topic in self.extraction_stats['gps_topics']:
                self.extraction_stats['gps_topics'][topic]['extracted_count'] = extracted_count
                expected = self.extraction_stats['gps_topics'][topic]['expected_count']
                percentage = extracted_count/expected*100 if expected > 0 else 0
                color = Fore.GREEN if percentage >= 95 else Fore.YELLOW if percentage >= 80 else Fore.RED
                self._log(f"✅ GPS {topic}: Extracted {self._colorize(f'{extracted_count}/{expected}', color)} messages ({self._colorize(f'{percentage:.1f}%', color)})")
            else:
                self._log(f"✅ GPS {topic}: Extracted {extracted_count} messages (metadata not available)")
        
        for topic in target_odom_topics:
            extracted_count = odom_counts[topic]
            if topic in self.extraction_stats['odom_topics']:
                self.extraction_stats['odom_topics'][topic]['extracted_count'] = extracted_count
                expected = self.extraction_stats['odom_topics'][topic]['expected_count']
                percentage = extracted_count/expected*100 if expected > 0 else 0
                color = Fore.GREEN if percentage >= 95 else Fore.YELLOW if percentage >= 80 else Fore.RED
                self._log(f"✅ ODOM {topic}: Extracted {self._colorize(f'{extracted_count}/{expected}', color)} messages ({self._colorize(f'{percentage:.1f}%', color)})")
            else:
                self._log(f"✅ ODOM {topic}: Extracted {extracted_count} messages (metadata not available)")
        
        return {
            'gps': gps_results,
            'odom': odom_results
        }
    
    def extract_gps_data(self, gps_messages):
        """
        Extract GPS data from messages.
        Based on extract_gps_data.py logic.
        
        Args:
            gps_messages (dict): Dictionary of GPS messages by topic
        
        Returns:
            list: List of extracted GPS data points
        """
        extracted_data = []
        
        for topic_name, messages in gps_messages.items():
            self._log(f"Extracting GPS data from {topic_name}...")
            
            for msg_data in messages:
                bag_timestamp = msg_data['bag_timestamp']
                msg = msg_data['message']
                
                # Convert bag timestamp to milliseconds with nanosecond precision
                timestamp_ms = bag_timestamp / 1e6  # Convert nanoseconds to milliseconds
                
                # Extract ROS header timestamp if available
                ros_timestamp_ms = None
                if hasattr(msg, 'header'):
                    # Convert ROS timestamp to milliseconds
                    ros_timestamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                    ros_timestamp_ms = ros_timestamp_ns / 1e6
                
                # Extract LLA coordinates
                latitude = msg.latitude if hasattr(msg, 'latitude') else None
                longitude = msg.longitude if hasattr(msg, 'longitude') else None
                altitude = msg.altitude if hasattr(msg, 'altitude') else None
                
                # Extract position covariance if available
                position_covariance = None
                if hasattr(msg, 'position_covariance'):
                    position_covariance = list(msg.position_covariance)
                
                # Extract status if available
                status = None
                if hasattr(msg, 'status'):
                    status = {
                        'status': msg.status.status if hasattr(msg.status, 'status') else None,
                        'service': msg.status.service if hasattr(msg.status, 'service') else None
                    }
                
                # Extract position covariance type
                position_covariance_type = None
                if hasattr(msg, 'position_covariance_type'):
                    position_covariance_type = msg.position_covariance_type
                
                data_point = {
                    'topic_name': topic_name,
                    'timestamp_ms': timestamp_ms,
                    'ros_timestamp_ms': ros_timestamp_ms,
                    'latitude': latitude,
                    'longitude': longitude,
                    'altitude': altitude,
                    'position_covariance': position_covariance,
                    'position_covariance_type': position_covariance_type,
                    'status': status
                }
                
                extracted_data.append(data_point)
        
        # Update valid data counts
        for topic_name in gps_messages.keys():
            if topic_name in self.extraction_stats['gps_topics']:
                valid_count = sum(1 for d in extracted_data 
                                if d['topic_name'] == topic_name and d['latitude'] and d['longitude'])
                self.extraction_stats['gps_topics'][topic_name]['valid_count'] = valid_count
        
        self.gps_data = extracted_data
        return extracted_data
    
    def extract_odom_data(self, odom_messages):
        """
        Extract odometry data from messages.
        Based on extract_odom_data.py logic.
        
        Args:
            odom_messages (dict): Dictionary of odometry messages by topic
        
        Returns:
            list: List of extracted odometry data points
        """
        extracted_data = []
        
        for topic_name, messages in odom_messages.items():
            self._log(f"Extracting odometry data from {topic_name}...")
            
            for msg_data in messages:
                bag_timestamp = msg_data['bag_timestamp']
                msg = msg_data['message']
                
                # Convert bag timestamp to milliseconds with nanosecond precision
                timestamp_ms = bag_timestamp / 1e6  # Convert nanoseconds to milliseconds
                
                # Extract ROS header timestamp if available
                ros_timestamp_ms = None
                frame_id = None
                if hasattr(msg, 'header'):
                    # Convert ROS timestamp to milliseconds
                    ros_timestamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                    ros_timestamp_ms = ros_timestamp_ns / 1e6
                    frame_id = msg.header.frame_id if hasattr(msg.header, 'frame_id') else None
                
                # Extract pose data
                pose_data = {}
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    pose = msg.pose.pose
                    
                    # Position
                    if hasattr(pose, 'position'):
                        pose_data['position'] = {
                            'x': pose.position.x,
                            'y': pose.position.y,
                            'z': pose.position.z
                        }
                    
                    # Orientation (quaternion)
                    if hasattr(pose, 'orientation'):
                        quat = pose.orientation
                        pose_data['orientation'] = {
                            'x': quat.x,
                            'y': quat.y,
                            'z': quat.z,
                            'w': quat.w
                        }
                    
                    # Pose covariance (6x6 matrix as flat array)
                    if hasattr(msg.pose, 'covariance'):
                        pose_data['covariance'] = list(msg.pose.covariance)
                
                # Extract twist data (velocity)
                twist_data = {}
                if hasattr(msg, 'twist') and hasattr(msg.twist, 'twist'):
                    twist = msg.twist.twist
                    
                    # Linear velocity
                    if hasattr(twist, 'linear'):
                        twist_data['linear'] = {
                            'x': twist.linear.x,
                            'y': twist.linear.y,
                            'z': twist.linear.z
                        }
                    
                    # Angular velocity
                    if hasattr(twist, 'angular'):
                        twist_data['angular'] = {
                            'x': twist.angular.x,
                            'y': twist.angular.y,
                            'z': twist.angular.z
                        }
                    
                    # Twist covariance
                    if hasattr(msg.twist, 'covariance'):
                        twist_data['covariance'] = list(msg.twist.covariance)
                
                # Extract child frame id if available
                child_frame_id = None
                if hasattr(msg, 'child_frame_id'):
                    child_frame_id = msg.child_frame_id
                
                data_point = {
                    'topic_name': topic_name,
                    'timestamp_ms': timestamp_ms,
                    'ros_timestamp_ms': ros_timestamp_ms,
                    'frame_id': frame_id,
                    'child_frame_id': child_frame_id,
                    'pose': pose_data,
                    'twist': twist_data
                }
                
                extracted_data.append(data_point)
        
        # Update valid data counts  
        for topic_name in odom_messages.keys():
            if topic_name in self.extraction_stats['odom_topics']:
                valid_count = sum(1 for d in extracted_data 
                                if d['topic_name'] == topic_name and 'position' in d['pose'])
                self.extraction_stats['odom_topics'][topic_name]['valid_count'] = valid_count
        
        self.odom_data = extracted_data
        return extracted_data
    
    def save_gps_to_csv(self, filename=None):
        """Save extracted GPS data to CSV file using pandas."""
        if not self.gps_data:
            self._log("❌ No GPS data to save", 'WARNING')
            return None
        
        if filename is None:
            filename = "gps_data_extracted.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Prepare data for DataFrame
        df_data = []
        for data_point in self.gps_data:
            row = {
                'topic_name': data_point['topic_name'],
                'timestamp_ms': data_point['timestamp_ms'],
                'ros_timestamp_ms': data_point['ros_timestamp_ms'] if data_point['ros_timestamp_ms'] else np.nan,
                'latitude': data_point['latitude'] if data_point['latitude'] else np.nan,
                'longitude': data_point['longitude'] if data_point['longitude'] else np.nan,
                'altitude': data_point['altitude'] if data_point['altitude'] else np.nan,
                'position_covariance_type': data_point['position_covariance_type'] if data_point['position_covariance_type'] else np.nan
            }
            df_data.append(row)
        
        # Create DataFrame and save to CSV
        df = pd.DataFrame(df_data)
        df.to_csv(filepath, index=False, float_format='%.10f')
        
        self._log(f"✅ Saved GPS data to CSV: {filepath}")
        return filepath
    
    def save_gps_to_json(self, filename=None):
        """Save extracted GPS data to JSON file using pandas."""
        if not self.gps_data:
            self._log("❌ No GPS data to save", 'WARNING')
            return None
        
        if filename is None:
            filename = "gps_data_extracted.json"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Convert to DataFrame for easy JSON export
        df = pd.DataFrame(self.gps_data)
        
        # Save to JSON with proper formatting
        df.to_json(filepath, orient='records', indent=2)
        
        self._log(f"✅ Saved GPS data to JSON: {filepath}")
        return filepath
    
    def save_odom_to_csv(self, filename=None):
        """Save extracted odometry data to CSV file using pandas."""
        if not self.odom_data:
            self._log("❌ No odometry data to save", 'WARNING')
            return None
        
        if filename is None:
            filename = "odom_data_extracted.csv"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Prepare data for DataFrame
        df_data = []
        for data_point in self.odom_data:
            row = {
                'topic_name': data_point['topic_name'],
                'timestamp_ms': data_point['timestamp_ms'],
                'ros_timestamp_ms': data_point['ros_timestamp_ms'] if data_point['ros_timestamp_ms'] else np.nan,
                'frame_id': data_point['frame_id'] or '',
                'child_frame_id': data_point['child_frame_id'] or ''
            }
            
            # Add pose data
            if 'position' in data_point['pose']:
                row['position_x'] = data_point['pose']['position']['x']
                row['position_y'] = data_point['pose']['position']['y']
                row['position_z'] = data_point['pose']['position']['z']
            else:
                row['position_x'] = np.nan
                row['position_y'] = np.nan
                row['position_z'] = np.nan
            
            if 'orientation' in data_point['pose']:
                row['orientation_x'] = data_point['pose']['orientation']['x']
                row['orientation_y'] = data_point['pose']['orientation']['y']
                row['orientation_z'] = data_point['pose']['orientation']['z']
                row['orientation_w'] = data_point['pose']['orientation']['w']
            else:
                row['orientation_x'] = np.nan
                row['orientation_y'] = np.nan
                row['orientation_z'] = np.nan
                row['orientation_w'] = np.nan
            
            # Add twist data
            if 'linear' in data_point['twist']:
                row['linear_velocity_x'] = data_point['twist']['linear']['x']
                row['linear_velocity_y'] = data_point['twist']['linear']['y']
                row['linear_velocity_z'] = data_point['twist']['linear']['z']
            else:
                row['linear_velocity_x'] = np.nan
                row['linear_velocity_y'] = np.nan
                row['linear_velocity_z'] = np.nan
            
            if 'angular' in data_point['twist']:
                row['angular_velocity_x'] = data_point['twist']['angular']['x']
                row['angular_velocity_y'] = data_point['twist']['angular']['y']
                row['angular_velocity_z'] = data_point['twist']['angular']['z']
            else:
                row['angular_velocity_x'] = np.nan
                row['angular_velocity_y'] = np.nan
                row['angular_velocity_z'] = np.nan
            
            df_data.append(row)
        
        # Create DataFrame and save to CSV
        df = pd.DataFrame(df_data)
        df.to_csv(filepath, index=False, float_format='%.10f')
        
        self._log(f"✅ Saved odometry data to CSV: {filepath}")
        return filepath
    
    def save_odom_to_json(self, filename=None):
        """Save extracted odometry data to JSON file using pandas."""
        if not self.odom_data:
            self._log("❌ No odometry data to save", 'WARNING')
            return None
        
        if filename is None:
            filename = "odom_data_extracted.json"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Convert to DataFrame for easy JSON export
        df = pd.DataFrame(self.odom_data)
        
        # Save to JSON with proper formatting
        df.to_json(filepath, orient='records', indent=2)
        
        self._log(f"✅ Saved odometry data to JSON: {filepath}")
        return filepath
    
    def print_extraction_statistics(self):
        """Print detailed extraction statistics comparing expected vs extracted vs valid counts."""
        if not self.extraction_stats['extraction_start_time']:
            self._log("No extraction statistics available", 'WARNING')
            return
        
        start_time = self.extraction_stats['extraction_start_time']
        end_time = self.extraction_stats['extraction_end_time']
        duration = (end_time - start_time).total_seconds() if end_time else 0
        
        self._log(f"\n{'='*80}")
        self._log(f"EXTRACTION STATISTICS COMPARISON")
        self._log(f"{'='*80}")
        self._log(f"📁 Bag Path: {self.bag_path}")
        self._log(f"⏱️  Extraction Duration: {duration:.2f} seconds")
        
        # GPS Topics Statistics
        if self.extraction_stats['gps_topics']:
            self._log(f"\n📍 GPS TOPICS ANALYSIS:")
            self._log(f"{'Topic Name':<30} {'Expected':<10} {'Extracted':<12} {'Valid':<10} {'Extraction %':<15} {'Valid %':<10}")
            self._log(f"{'-'*85}")
            
            for topic_name, stats in self.extraction_stats['gps_topics'].items():
                expected = stats.get('expected_count', 0)
                extracted = stats.get('extracted_count', 0)
                valid = stats.get('valid_count', 0)
                
                extraction_pct = (extracted / expected * 100) if expected > 0 else 0
                valid_pct = (valid / extracted * 100) if extracted > 0 else 0
                
                self._log(f"{topic_name:<30} {expected:<10} {extracted:<12} {valid:<10} {extraction_pct:<15.1f} {valid_pct:<10.1f}")
        
        # Odometry Topics Statistics
        if self.extraction_stats['odom_topics']:
            self._log(f"\n🗺️  ODOMETRY TOPICS ANALYSIS:")
            self._log(f"{'Topic Name':<30} {'Expected':<10} {'Extracted':<12} {'Valid':<10} {'Extraction %':<15} {'Valid %':<10}")
            self._log(f"{'-'*85}")
            
            for topic_name, stats in self.extraction_stats['odom_topics'].items():
                expected = stats.get('expected_count', 0)
                extracted = stats.get('extracted_count', 0)
                valid = stats.get('valid_count', 0)
                
                extraction_pct = (extracted / expected * 100) if expected > 0 else 0
                valid_pct = (valid / extracted * 100) if extracted > 0 else 0
                
                self._log(f"{topic_name:<30} {expected:<10} {extracted:<12} {valid:<10} {extraction_pct:<15.1f} {valid_pct:<10.1f}")
        
        # Overall summary
        total_expected = sum(stats.get('expected_count', 0) for stats in self.extraction_stats['gps_topics'].values()) + \
                        sum(stats.get('expected_count', 0) for stats in self.extraction_stats['odom_topics'].values())
        total_extracted = sum(stats.get('extracted_count', 0) for stats in self.extraction_stats['gps_topics'].values()) + \
                         sum(stats.get('extracted_count', 0) for stats in self.extraction_stats['odom_topics'].values())
        total_valid = sum(stats.get('valid_count', 0) for stats in self.extraction_stats['gps_topics'].values()) + \
                     sum(stats.get('valid_count', 0) for stats in self.extraction_stats['odom_topics'].values())
        
        self._log(f"\n📊 OVERALL SUMMARY:")
        self._log(f"Total Expected Messages: {total_expected}")
        self._log(f"Total Extracted Messages: {total_extracted} ({total_extracted/total_expected*100:.1f}%)" if total_expected > 0 else f"Total Extracted Messages: {total_extracted}")
        self._log(f"Total Valid Messages: {total_valid} ({total_valid/total_extracted*100:.1f}%)" if total_extracted > 0 else f"Total Valid Messages: {total_valid}")
        
        if duration > 0:
            self._log(f"Average Processing Rate: {total_extracted/duration:.1f} messages/second")
        
        self._log(f"{'='*80}\n")
    
    def print_gps_summary(self):
        """Print summary of extracted GPS data."""
        if not self.gps_data:
            self._log("❌ No GPS data extracted")
            return
        
        self._log(f"\n{'='*60}")
        self._log(f"GPS DATA EXTRACTION SUMMARY")
        self._log(f"{'='*60}")
        
        self._log(f"📁 Source Bag: {os.path.basename(self.bag_path)}")
        self._log(f"📁 Output Directory: {self.output_dir}")
        self._log(f"📍 Total GPS Points: {len(self.gps_data)}")
        
        # Group by topic
        topic_counts = {}
        for point in self.gps_data:
            topic = point['topic_name']
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
        
        self._log(f"\n📊 GPS Points by Topic:")
        for topic, count in topic_counts.items():
            self._log(f"  {topic}: {count} points")
        
        # Calculate statistics
        valid_points = [p for p in self.gps_data if p['latitude'] and p['longitude']]
        
        if valid_points:
            lat_min = min(p['latitude'] for p in valid_points)
            lat_max = max(p['latitude'] for p in valid_points)
            lon_min = min(p['longitude'] for p in valid_points)
            lon_max = max(p['longitude'] for p in valid_points)
            
            alt_points = [p for p in valid_points if p['altitude']]
            if alt_points:
                alt_min = min(p['altitude'] for p in alt_points)
                alt_max = max(p['altitude'] for p in alt_points)
            else:
                alt_min = alt_max = 0
            
            # Time range
            time_min = min(p['timestamp_ms'] for p in self.gps_data)
            time_max = max(p['timestamp_ms'] for p in self.gps_data)
            duration_s = (time_max - time_min) / 1000.0
            
            self._log(f"\n📊 Valid GPS Points: {len(valid_points)}")
            self._log(f"⏱️  Time Range: {time_min:.3f} to {time_max:.3f} ms")
            self._log(f"⏱️  Duration: {duration_s:.1f} seconds")
            self._log(f"\n🌍 Coordinate Bounds:")
            self._log(f"   Latitude:  {lat_min:.8f}° to {lat_max:.8f}°")
            self._log(f"   Longitude: {lon_min:.8f}° to {lon_max:.8f}°")
            self._log(f"   Altitude:  {alt_min:.2f} to {alt_max:.2f} m")
        else:
            self._log(f"⚠️  No valid GPS coordinates found", 'WARNING')
        
        self._log(f"\n{'='*60}")
    
    def print_odom_summary(self):
        """Print summary of extracted odometry data."""
        if not self.odom_data:
            self._log("❌ No odometry data extracted")
            return
        
        self._log(f"\n{'='*60}")
        self._log(f"ODOMETRY DATA EXTRACTION SUMMARY")
        self._log(f"{'='*60}")
        
        self._log(f"📁 Source Bag: {os.path.basename(self.bag_path)}")
        self._log(f"📁 Output Directory: {self.output_dir}")
        self._log(f"📍 Total Odometry Points: {len(self.odom_data)}")
        
        # Group by topic
        topic_counts = {}
        for point in self.odom_data:
            topic = point['topic_name']
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
        
        self._log(f"\n📊 Odometry Points by Topic:")
        for topic, count in topic_counts.items():
            self._log(f"  {topic}: {count} points")
        
        # Calculate statistics from positions
        positions = []
        for point in self.odom_data:
            if 'position' in point['pose']:
                positions.append([
                    point['pose']['position']['x'],
                    point['pose']['position']['y'],
                    point['pose']['position']['z']
                ])
        
        if positions:
            positions = np.array(positions)
            
            # Calculate distances
            if len(positions) > 1:
                diffs = np.diff(positions, axis=0)
                distances = np.linalg.norm(diffs, axis=1)
                total_distance = np.sum(distances)
            else:
                total_distance = 0
            
            # Time range
            time_min = min(p['timestamp_ms'] for p in self.odom_data)
            time_max = max(p['timestamp_ms'] for p in self.odom_data)
            duration_s = (time_max - time_min) / 1000.0
            
            self._log(f"\n📊 Trajectory Statistics:")
            self._log(f"⏱️  Time Range: {time_min:.3f} to {time_max:.3f} ms")
            self._log(f"⏱️  Duration: {duration_s:.1f} seconds")
            self._log(f"📏 Total Distance: {total_distance:.2f} m")
            
            self._log(f"\n🗺️  Position Range:")
            self._log(f"   X: [{np.min(positions[:, 0]):.3f}, {np.max(positions[:, 0]):.3f}] m")
            self._log(f"   Y: [{np.min(positions[:, 1]):.3f}, {np.max(positions[:, 1]):.3f}] m")
            self._log(f"   Z: [{np.min(positions[:, 2]):.3f}, {np.max(positions[:, 2]):.3f}] m")
        
        self._log(f"\n{'='*60}")


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(
        description="Unified GPS and odometry data extractor from ROS2 bag files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract all GPS and odometry data from bag file
  python3 extract_bag_data.py --bag_path /path/to/bag_file

  # Extract from specific topics
  python3 extract_bag_data.py --bag_path bag.db3 --gps_topic /cbs_gnss --odom_topic /odom

  # Specify output directory and format
  python3 extract_bag_data.py --bag_path /path/to/bag --output_dir ./unified_output --format csv

  # Enable logging with detailed statistics
  python3 extract_bag_data.py --bag_path /path/to/bag --enable_logging --log_dir ./logs

Output Files:
  - gps_data_extracted.csv/json: GPS coordinates and timestamps
  - odom_data_extracted.csv/json: Pose and velocity data
  - extract_bag_data_[bag_name]_[timestamp].log: Detailed extraction log (if --enable_logging)
        """
    )
    
    parser.add_argument('--bag_path', type=str, required=True,
                       help='Path to ROS2 bag file or directory')
    parser.add_argument('--output_dir', type=str, default='results',
                       help='Output directory for extracted data (default: results)')
    parser.add_argument('--gps_topic', type=str, nargs='*',
                       help='GPS topic name(s) to extract (auto-detect if not specified)')
    parser.add_argument('--odom_topic', type=str, nargs='*', 
                       help='Odometry topic name(s) to extract (auto-detect if not specified)')
    parser.add_argument('--format', type=str, choices=['csv', 'json', 'both'], default='both',
                       help='Output format (default: both)')
    parser.add_argument('--gps_only', action='store_true',
                       help='Extract only GPS data')
    parser.add_argument('--odom_only', action='store_true',
                       help='Extract only odometry data')
    parser.add_argument('--enable_logging', action='store_true',
                       help='Enable logging to file with timestamp')
    parser.add_argument('--log_dir', type=str,
                       help='Directory for log files (default: geodetic-points/log)')
    
    args = parser.parse_args()
    
    # Validate bag path
    if not os.path.exists(args.bag_path):
        print(f"❌ Error: Bag path does not exist: {args.bag_path}")
        sys.exit(1)
    
    # Create extractor with logging if enabled
    extractor = UnifiedBagDataExtractor(
        args.bag_path, 
        args.output_dir, 
        enable_logging=args.enable_logging,
        log_dir=args.log_dir
    )
    
    print("="*80)
    print("UNIFIED ROS2 BAG DATA EXTRACTOR")
    print("="*80)
    print(f"📁 Input Bag: {args.bag_path}")
    print(f"📁 Output Dir: {args.output_dir}")
    print(f"📄 Output Format: {args.format}")
    print("="*80)
    
    # Step 1: Read bag metadata
    print(f"\n📖 Step 1: Reading bag metadata...")
    metadata = extractor.read_bag_metadata()
    
    # Step 2: Read bag file
    print(f"\n📦 Step 2: Reading ROS2 bag file...")
    
    target_gps_topics = args.gps_topic if args.gps_topic else None
    target_odom_topics = args.odom_topic if args.odom_topic else None
    
    # Skip topics based on flags
    if args.gps_only:
        target_odom_topics = []
    elif args.odom_only:
        target_gps_topics = []
    
    bag_results = extractor.read_ros2_bag(target_gps_topics, target_odom_topics)
    
    # Step 3: Extract data
    success = False
    
    if not args.odom_only and bag_results['gps']:
        print(f"\n📍 Step 3a: Extracting GPS data...")
        extractor.extract_gps_data(bag_results['gps'])
        success = True
    
    if not args.gps_only and bag_results['odom']:
        print(f"\n🗺️  Step 3b: Extracting odometry data...")
        extractor.extract_odom_data(bag_results['odom'])
        success = True
    
    if not success:
        print("❌ No data was extracted")
        sys.exit(1)
    
    # Step 4: Save to files
    print(f"\n💾 Step 4: Saving extracted data...")
    
    if extractor.gps_data:
        if args.format in ['csv', 'both']:
            extractor.save_gps_to_csv()
        if args.format in ['json', 'both']:
            extractor.save_gps_to_json()
    
    if extractor.odom_data:
        if args.format in ['csv', 'both']:
            extractor.save_odom_to_csv()
        if args.format in ['json', 'both']:
            extractor.save_odom_to_json()
    
    # Step 5: Print extraction statistics
    if args.enable_logging or extractor.extraction_stats['extraction_start_time']:
        extractor.print_extraction_statistics()
    
    # Step 6: Print summaries
    if extractor.gps_data:
        extractor.print_gps_summary()
    
    if extractor.odom_data:
        extractor.print_odom_summary()
    
    if extractor.logger:
        extractor._log(f"\n🎉 Success! Data extracted to: {args.output_dir}")
        if extractor.log_file_path:
            extractor._log(f"📝 Full extraction log saved to: {extractor.log_file_path}")
    else:
        print(f"\n🎉 Success! Data extracted to: {args.output_dir}")


if __name__ == "__main__":
    main()