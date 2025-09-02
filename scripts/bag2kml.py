#!/usr/bin/env python3
"""
ROS2 Bag to KML Converter for GPS Trajectories

This script extracts all GPS trajectories from ROS2 bag files and converts them 
to KML format compatible with Google Earth and other mapping applications.

Features:
- Automatic GPS topic detection (sensor_msgs/msg/NavSatFix)
- Support for multiple GPS topics in single bag file
- Color-coded trajectories by GPS source
- Time-stamped waypoints with detailed information
- Trajectory statistics and metadata
- Efficient bag reading with validation
- Support for both .db3 files and bag folders

Usage:
    python3 bag2kml.py --bag_path /path/to/bag --output_dir results/
    python3 bag2kml.py --bag_path bag.db3 --topics /cbs_gnss /iphone_gnss
    
Output:
    - Individual KML files for each GPS topic
    - Combined KML with all trajectories
    - Detailed trajectory statistics

Author: GPS Trajectory Extraction System
Created: September 2025
Compatible: Google Earth, QGIS, ArcGIS, and other KML-supporting applications
"""

import argparse
import os
import sys
import math
import numpy as np
from datetime import datetime, timezone
from collections import defaultdict
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

# ROS2 imports
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class GPSTrajectoryExtractor:
    """Extract GPS trajectories from ROS2 bag files and convert to KML format."""
    
    def __init__(self, bag_path, output_dir="results"):
        """
        Initialize GPS trajectory extractor.
        
        Args:
            bag_path (str): Path to ROS2 bag file or directory
            output_dir (str): Output directory for KML files
        """
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.trajectories = {}
        self.trajectory_stats = {}
        
        # Color mapping for different GPS sources
        self.colors = {
            '/cbs_gnss': 'ff0000ff',        # Red
            '/cbs_gnss_p7pro': 'ff00ff00',  # Green  
            '/gnss': 'ffff0000',            # Blue
            '/iphone_gnss': 'ff00ffff',     # Yellow
            '/cbgps': 'ffff00ff',           # Magenta
            '/ublox_gps': 'ff80ff80',       # Light Green
            '/other_gps': 'ff8080ff',       # Light Red
        }
        self.default_colors = [
            'ff0000ff', 'ff00ff00', 'ffff0000', 'ff00ffff', 'ffff00ff',
            'ff80ff80', 'ff8080ff', 'fff0f0f0', 'ff404040', 'ff808080'
        ]
        
        # Ensure output directory exists
        os.makedirs(output_dir, exist_ok=True)
    
    def read_bag_metadata(self):
        """
        Read bag metadata to identify GPS topics.
        Based on test_gps_trajectory_visualization.py:read_bag_metadata
        """
        import yaml
        
        # Handle both .db3 files and folder paths
        if self.bag_path.endswith('.db3'):
            metadata_path = os.path.join(os.path.dirname(self.bag_path), 'metadata.yaml')
        else:
            metadata_path = os.path.join(self.bag_path, 'metadata.yaml')
        
        navsat_topics = {}
        
        if os.path.exists(metadata_path):
            try:
                with open(metadata_path, 'r', encoding='utf-8') as f:
                    metadata = yaml.safe_load(f)
                
                print(f"✅ Successfully read metadata: {metadata_path}")
                
                if 'rosbag2_bagfile_information' in metadata:
                    bag_info = metadata['rosbag2_bagfile_information']
                    
                    if 'topics_with_message_count' in bag_info:
                        print(f"GPS topics found in bag:")
                        
                        for topic_info in bag_info['topics_with_message_count']:
                            topic_meta = topic_info['topic_metadata']
                            topic_name = topic_meta['name']
                            topic_type = topic_meta['type']
                            message_count = topic_info['message_count']
                            
                            if topic_type == 'sensor_msgs/msg/NavSatFix':
                                navsat_topics[topic_name] = {
                                    'type': topic_type,
                                    'expected_count': message_count,
                                    'serialization_format': topic_meta.get('serialization_format', 'cdr')
                                }
                                print(f"  {topic_name}: {message_count} messages")
                
                if not navsat_topics:
                    print("⚠️  No NavSatFix topics found in metadata")
                else:
                    print(f"✅ Found {len(navsat_topics)} GPS topics")
                    
            except Exception as e:
                print(f"❌ Failed to read metadata: {e}")
        else:
            print(f"⚠️  Metadata file not found: {metadata_path}")
        
        return navsat_topics
    
    def read_ros2_bag(self, target_topics=None, storage_id='sqlite3'):
        """
        Read ROS2 bag file and extract GPS messages.
        Based on test_gps_trajectory_visualization.py:read_ros2_bag
        """
        # Handle both .db3 files and folder paths
        bag_path = self.bag_path
        if bag_path.endswith('.db3'):
            bag_path = os.path.dirname(bag_path)
        
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            ),
        )

        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"Topic {topic_name} not in bag")

        # If no target topics specified, find all NavSatFix topics
        if target_topics is None:
            target_topics = []
            for topic_type in topic_types:
                if topic_type.type == 'sensor_msgs/msg/NavSatFix':
                    target_topics.append(topic_type.name)
            print(f"Auto-detected GPS topics: {target_topics}")
        
        # Validate topics exist and are NavSatFix type
        valid_topics = []
        for topic_name in target_topics:
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    if topic_type.type == 'sensor_msgs/msg/NavSatFix':
                        valid_topics.append(topic_name)
                        print(f"✅ Validated topic: {topic_name}")
                    else:
                        print(f"❌ Wrong type for {topic_name}: {topic_type.type} (expected NavSatFix)")
                    break
            else:
                print(f"❌ Topic not found: {topic_name}")
        
        if not valid_topics:
            print("❌ No valid NavSatFix topics found")
            return {}

        results = {topic: [] for topic in valid_topics}
        message_counts = {topic: 0 for topic in valid_topics}
        
        print(f"Reading GPS messages from {len(valid_topics)} topics...")
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic in valid_topics:
                try:
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)
                    results[topic].append((topic, msg, timestamp))
                    message_counts[topic] += 1
                except Exception as e:
                    print(f"Failed to parse message from {topic}: {e}")
                    continue
        
        del reader
        
        # Report message counts
        print(f"\n=== Message Count Summary ===")
        for topic in valid_topics:
            count = message_counts[topic]
            print(f"{topic}: {count} messages")
        
        return results
    
    def clean_and_process_gps_data(self, bag_results):
        """
        Clean GPS data and organize by topic.
        Based on test_gps_trajectory_visualization.py:clean_gps_data and process_gps_trajectory
        """
        for topic_name, topic_result in bag_results.items():
            if not topic_result:
                continue
                
            print(f"Processing GPS trajectory: {topic_name}")
            
            # Clean invalid GPS data
            valid_data = []
            for topic, msg, timestamp in topic_result:
                if (not math.isnan(msg.latitude) and not math.isnan(msg.longitude) and
                    not math.isnan(msg.altitude) and
                    abs(msg.latitude) > 0.001 and abs(msg.longitude) > 0.001):
                    valid_data.append((topic, msg, timestamp))
            
            if not valid_data:
                print(f"  ⚠️  No valid GPS data for {topic_name}")
                continue
            
            # Sort by timestamp and convert to trajectory format
            gps_trajectory = []
            for topic, msg, timestamp in sorted(valid_data, key=lambda x: x[2]):
                timestamp_sec = timestamp / 1e9
                ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                
                gps_point = {
                    'timestamp': timestamp,
                    'timestamp_sec': timestamp_sec,
                    'ros_timestamp_sec': ros_time_sec,
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'topic_name': topic_name
                }
                gps_trajectory.append(gps_point)
            
            self.trajectories[topic_name] = gps_trajectory
            
            # Calculate trajectory statistics
            self.calculate_trajectory_stats(topic_name, gps_trajectory)
            
            print(f"  ✅ Processed {len(gps_trajectory)} valid GPS points")
            print(f"  📍 Coordinate range: {min(p['latitude'] for p in gps_trajectory):.6f}° to {max(p['latitude'] for p in gps_trajectory):.6f}° N")
            print(f"                       {min(p['longitude'] for p in gps_trajectory):.6f}° to {max(p['longitude'] for p in gps_trajectory):.6f}° E")
            print(f"  ⏱️  Time span: {(gps_trajectory[-1]['timestamp_sec'] - gps_trajectory[0]['timestamp_sec']):.1f} seconds")
    
    def calculate_trajectory_stats(self, topic_name, trajectory):
        """Calculate comprehensive trajectory statistics."""
        if len(trajectory) < 2:
            return
        
        # Calculate distances and speeds
        from geopy.distance import geodesic
        distances = []
        time_diffs = []
        speeds = []
        
        total_distance = 0
        for i in range(1, len(trajectory)):
            prev_point = trajectory[i-1]
            curr_point = trajectory[i]
            
            # Calculate distance
            coord1 = (prev_point['latitude'], prev_point['longitude'])
            coord2 = (curr_point['latitude'], curr_point['longitude'])
            distance = geodesic(coord1, coord2).meters
            distances.append(distance)
            total_distance += distance
            
            # Calculate time difference and speed
            time_diff = curr_point['timestamp_sec'] - prev_point['timestamp_sec']
            time_diffs.append(time_diff)
            
            if time_diff > 0:
                speed = distance / time_diff  # m/s
                speeds.append(speed)
        
        # Calculate statistics
        self.trajectory_stats[topic_name] = {
            'point_count': len(trajectory),
            'total_distance_m': total_distance,
            'duration_sec': trajectory[-1]['timestamp_sec'] - trajectory[0]['timestamp_sec'],
            'avg_speed_ms': np.mean(speeds) if speeds else 0,
            'max_speed_ms': np.max(speeds) if speeds else 0,
            'avg_distance_m': np.mean(distances) if distances else 0,
            'max_distance_m': np.max(distances) if distances else 0,
            'avg_time_interval_s': np.mean(time_diffs) if time_diffs else 0,
            'coordinate_bounds': {
                'lat_min': min(p['latitude'] for p in trajectory),
                'lat_max': max(p['latitude'] for p in trajectory),
                'lon_min': min(p['longitude'] for p in trajectory),
                'lon_max': max(p['longitude'] for p in trajectory),
                'alt_min': min(p['altitude'] for p in trajectory),
                'alt_max': max(p['altitude'] for p in trajectory)
            }
        }
    
    def get_color_for_topic(self, topic_name, topic_index):
        """Get appropriate KML color for GPS topic."""
        if topic_name in self.colors:
            return self.colors[topic_name]
        else:
            return self.default_colors[topic_index % len(self.default_colors)]
    
    def create_kml_document(self, topic_name, trajectory, is_combined=False):
        """Create KML document for GPS trajectory."""
        # Create KML root
        kml = ET.Element('kml', xmlns="http://www.opengis.net/kml/2.2")
        document = ET.SubElement(kml, 'Document')
        
        # Document metadata
        name = ET.SubElement(document, 'name')
        name.text = f"GPS Trajectory: {topic_name}" if not is_combined else "Combined GPS Trajectories"
        
        description = ET.SubElement(document, 'description')
        if not is_combined:
            stats = self.trajectory_stats.get(topic_name, {})
            description.text = f"""
GPS Trajectory Analysis
Topic: {topic_name}
Points: {stats.get('point_count', 0)}
Total Distance: {stats.get('total_distance_m', 0):.1f} m
Duration: {stats.get('duration_sec', 0):.1f} s
Average Speed: {stats.get('avg_speed_ms', 0)*3.6:.1f} km/h
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
            """.strip()
        else:
            total_points = sum(len(traj) for traj in self.trajectories.values())
            total_distance = sum(stats.get('total_distance_m', 0) for stats in self.trajectory_stats.values())
            description.text = f"""
Combined GPS Trajectories Analysis
Topics: {len(self.trajectories)}
Total Points: {total_points}
Total Distance: {total_distance:.1f} m
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
            """.strip()
        
        return kml, document
    
    def add_trajectory_to_kml(self, document, topic_name, trajectory, color, topic_index=0):
        """Add GPS trajectory to KML document."""
        # Create placemark for trajectory
        placemark = ET.SubElement(document, 'Placemark')
        name = ET.SubElement(placemark, 'name')
        name.text = f"{topic_name} Trajectory"
        
        description = ET.SubElement(placemark, 'description')
        stats = self.trajectory_stats.get(topic_name, {})
        description.text = f"""
GPS Topic: {topic_name}
Points: {stats.get('point_count', 0)}
Distance: {stats.get('total_distance_m', 0):.1f} m
Duration: {stats.get('duration_sec', 0):.1f} s
Avg Speed: {stats.get('avg_speed_ms', 0)*3.6:.1f} km/h
Max Speed: {stats.get('max_speed_ms', 0)*3.6:.1f} km/h
        """.strip()
        
        # Create style for trajectory line
        style = ET.SubElement(placemark, 'Style')
        line_style = ET.SubElement(style, 'LineStyle')
        line_color = ET.SubElement(line_style, 'color')
        line_color.text = color
        line_width = ET.SubElement(line_style, 'width')
        line_width.text = "3"
        
        # Create LineString for trajectory
        linestring = ET.SubElement(placemark, 'LineString')
        tessellate = ET.SubElement(linestring, 'tessellate')
        tessellate.text = "1"
        altitude_mode = ET.SubElement(linestring, 'altitudeMode')
        altitude_mode.text = "absolute"
        
        coordinates = ET.SubElement(linestring, 'coordinates')
        coord_text = []
        for point in trajectory:
            coord_text.append(f"{point['longitude']:.8f},{point['latitude']:.8f},{point['altitude']:.2f}")
        coordinates.text = ' '.join(coord_text)
        
        # Add start and end placemarks
        self.add_waypoint_placemarks(document, topic_name, trajectory, color)
    
    def add_waypoint_placemarks(self, document, topic_name, trajectory, color):
        """Add start and end point placemarks."""
        if not trajectory:
            return
        
        # Start point
        start_point = trajectory[0]
        start_placemark = ET.SubElement(document, 'Placemark')
        start_name = ET.SubElement(start_placemark, 'name')
        start_name.text = f"{topic_name} Start"
        
        start_desc = ET.SubElement(start_placemark, 'description')
        start_time = datetime.fromtimestamp(start_point['timestamp_sec'], tz=timezone.utc)
        start_desc.text = f"""
Start Point - {topic_name}
Time: {start_time.strftime('%Y-%m-%d %H:%M:%S UTC')}
Coordinates: {start_point['latitude']:.8f}°, {start_point['longitude']:.8f}°
Altitude: {start_point['altitude']:.2f} m
ROS Timestamp: {start_point['ros_timestamp_sec']:.3f} s
        """.strip()
        
        start_style = ET.SubElement(start_placemark, 'Style')
        start_icon_style = ET.SubElement(start_style, 'IconStyle')
        start_icon_color = ET.SubElement(start_icon_style, 'color')
        start_icon_color.text = color
        start_icon_scale = ET.SubElement(start_icon_style, 'scale')
        start_icon_scale.text = "1.2"
        start_icon = ET.SubElement(start_icon_style, 'Icon')
        start_href = ET.SubElement(start_icon, 'href')
        start_href.text = "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png"
        
        start_point_elem = ET.SubElement(start_placemark, 'Point')
        start_coord = ET.SubElement(start_point_elem, 'coordinates')
        start_coord.text = f"{start_point['longitude']:.8f},{start_point['latitude']:.8f},{start_point['altitude']:.2f}"
        
        # End point
        end_point = trajectory[-1]
        end_placemark = ET.SubElement(document, 'Placemark')
        end_name = ET.SubElement(end_placemark, 'name')
        end_name.text = f"{topic_name} End"
        
        end_desc = ET.SubElement(end_placemark, 'description')
        end_time = datetime.fromtimestamp(end_point['timestamp_sec'], tz=timezone.utc)
        end_desc.text = f"""
End Point - {topic_name}
Time: {end_time.strftime('%Y-%m-%d %H:%M:%S UTC')}
Coordinates: {end_point['latitude']:.8f}°, {end_point['longitude']:.8f}°
Altitude: {end_point['altitude']:.2f} m
ROS Timestamp: {end_point['ros_timestamp_sec']:.3f} s
        """.strip()
        
        end_style = ET.SubElement(end_placemark, 'Style')
        end_icon_style = ET.SubElement(end_style, 'IconStyle')
        end_icon_color = ET.SubElement(end_icon_style, 'color')
        end_icon_color.text = color
        end_icon_scale = ET.SubElement(end_icon_style, 'scale')
        end_icon_scale.text = "1.2"
        end_icon = ET.SubElement(end_icon_style, 'Icon')
        end_href = ET.SubElement(end_icon, 'href')
        end_href.text = "http://maps.google.com/mapfiles/kml/shapes/flag.png"
        
        end_point_elem = ET.SubElement(end_placemark, 'Point')
        end_coord = ET.SubElement(end_point_elem, 'coordinates')
        end_coord.text = f"{end_point['longitude']:.8f},{end_point['latitude']:.8f},{end_point['altitude']:.2f}"
    
    def save_individual_kml_files(self):
        """Save individual KML files for each GPS topic."""
        individual_files = []
        
        for topic_index, (topic_name, trajectory) in enumerate(self.trajectories.items()):
            if not trajectory:
                continue
            
            # Generate safe filename
            safe_topic_name = topic_name.replace('/', '_').replace(' ', '_')
            filename = f"gps_trajectory_{safe_topic_name}.kml"
            filepath = os.path.join(self.output_dir, filename)
            
            # Create KML document
            kml, document = self.create_kml_document(topic_name, trajectory)
            color = self.get_color_for_topic(topic_name, topic_index)
            
            # Add trajectory to KML
            self.add_trajectory_to_kml(document, topic_name, trajectory, color, topic_index)
            
            # Save KML file
            self.save_kml_file(kml, filepath)
            individual_files.append(filepath)
            
            print(f"✅ Saved individual KML: {filename}")
        
        return individual_files
    
    def save_combined_kml_file(self):
        """Save combined KML file with all GPS trajectories."""
        if not self.trajectories:
            return None
        
        filename = "combined_gps_trajectories.kml"
        filepath = os.path.join(self.output_dir, filename)
        
        # Create combined KML document
        kml, document = self.create_kml_document("Combined", None, is_combined=True)
        
        # Add each trajectory with different colors
        for topic_index, (topic_name, trajectory) in enumerate(self.trajectories.items()):
            if trajectory:
                color = self.get_color_for_topic(topic_name, topic_index)
                self.add_trajectory_to_kml(document, topic_name, trajectory, color, topic_index)
        
        # Save combined KML file
        self.save_kml_file(kml, filepath)
        print(f"✅ Saved combined KML: {filename}")
        
        return filepath
    
    def save_kml_file(self, kml_element, filepath):
        """Save KML element to file with proper formatting."""
        # Convert to string with pretty formatting
        rough_string = ET.tostring(kml_element, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        # Remove empty lines and save
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
    
    def print_trajectory_summary(self):
        """Print comprehensive trajectory summary."""
        if not self.trajectories:
            print("❌ No GPS trajectories found")
            return
        
        print(f"\n{'='*60}")
        print(f"GPS TRAJECTORY EXTRACTION SUMMARY")
        print(f"{'='*60}")
        
        print(f"📁 Source Bag: {os.path.basename(self.bag_path)}")
        print(f"📁 Output Directory: {self.output_dir}")
        print(f"📊 Total GPS Topics: {len(self.trajectories)}")
        
        total_points = sum(len(traj) for traj in self.trajectories.values())
        total_distance = sum(stats.get('total_distance_m', 0) for stats in self.trajectory_stats.values())
        
        print(f"📍 Total GPS Points: {total_points}")
        print(f"📏 Total Distance: {total_distance:.1f} m ({total_distance/1000:.2f} km)")
        
        print(f"\n📋 Individual Trajectory Details:")
        for topic_name, trajectory in self.trajectories.items():
            stats = self.trajectory_stats.get(topic_name, {})
            bounds = stats.get('coordinate_bounds', {})
            
            print(f"\n🗂️  Topic: {topic_name}")
            print(f"   📍 Points: {stats.get('point_count', 0)}")
            print(f"   📏 Distance: {stats.get('total_distance_m', 0):.1f} m")
            print(f"   ⏱️  Duration: {stats.get('duration_sec', 0):.1f} s")
            print(f"   🏃 Avg Speed: {stats.get('avg_speed_ms', 0)*3.6:.1f} km/h")
            print(f"   🏃 Max Speed: {stats.get('max_speed_ms', 0)*3.6:.1f} km/h")
            print(f"   🌍 Lat Range: {bounds.get('lat_min', 0):.6f}° to {bounds.get('lat_max', 0):.6f}°")
            print(f"   🌍 Lon Range: {bounds.get('lon_min', 0):.6f}° to {bounds.get('lon_max', 0):.6f}°")
            print(f"   ⛰️  Alt Range: {bounds.get('alt_min', 0):.1f} to {bounds.get('alt_max', 0):.1f} m")
        
        print(f"\n{'='*60}")
    
    def extract_and_convert(self, target_topics=None):
        """Main extraction and conversion workflow."""
        print(f"🚀 Starting GPS trajectory extraction from: {self.bag_path}")
        
        try:
            # Step 1: Read bag metadata
            print(f"\n📖 Step 1: Reading bag metadata...")
            metadata = self.read_bag_metadata()
            
            # Step 2: Read bag file
            print(f"\n📦 Step 2: Reading ROS2 bag file...")
            bag_results = self.read_ros2_bag(target_topics)
            
            if not bag_results:
                print("❌ No GPS data found in bag file")
                return False
            
            # Step 3: Clean and process GPS data
            print(f"\n🧹 Step 3: Cleaning and processing GPS data...")
            self.clean_and_process_gps_data(bag_results)
            
            if not self.trajectories:
                print("❌ No valid GPS trajectories after cleaning")
                return False
            
            # Step 4: Generate KML files
            print(f"\n🗺️  Step 4: Generating KML files...")
            individual_files = self.save_individual_kml_files()
            combined_file = self.save_combined_kml_file()
            
            # Step 5: Print summary
            self.print_trajectory_summary()
            
            print(f"\n✅ GPS trajectory extraction completed successfully!")
            print(f"📁 Individual KML files: {len(individual_files)}")
            if combined_file:
                print(f"📁 Combined KML file: {os.path.basename(combined_file)}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error during GPS trajectory extraction: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(
        description="Extract GPS trajectories from ROS2 bag files and convert to KML format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract all GPS topics from bag file
  python3 bag2kml.py --bag_path /path/to/bag_file

  # Extract specific GPS topics
  python3 bag2kml.py --bag_path bag.db3 --topics /cbs_gnss /iphone_gnss

  # Specify custom output directory
  python3 bag2kml.py --bag_path /path/to/bag --output_dir ./kml_output

Output Files:
  - Individual KML files for each GPS topic (gps_trajectory_[topic_name].kml)
  - Combined KML file with all trajectories (combined_gps_trajectories.kml)
  
Compatible with: Google Earth, QGIS, ArcGIS, and other KML-supporting applications
        """
    )
    
    parser.add_argument('--bag_path', type=str, required=True,
                       help='Path to ROS2 bag file or directory')
    parser.add_argument('--output_dir', type=str, default='results',
                       help='Output directory for KML files (default: results)')
    parser.add_argument('--topics', type=str, nargs='*', default=None,
                       help='Specific GPS topics to extract (default: auto-detect all NavSatFix topics)')
    
    args = parser.parse_args()
    
    # Validate bag path
    if not os.path.exists(args.bag_path):
        print(f"❌ Error: Bag path does not exist: {args.bag_path}")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print("="*80)
    print("ROS2 BAG TO KML CONVERTER")
    print("="*80)
    print(f"📁 Input Bag: {args.bag_path}")
    print(f"📁 Output Dir: {args.output_dir}")
    if args.topics:
        print(f"🎯 Target Topics: {args.topics}")
    else:
        print(f"🔍 Auto-detecting GPS topics...")
    print("="*80)
    
    # Create extractor and run conversion
    extractor = GPSTrajectoryExtractor(args.bag_path, args.output_dir)
    success = extractor.extract_and_convert(args.topics)
    
    if success:
        print(f"\n🎉 Success! KML files saved to: {args.output_dir}")
        sys.exit(0)
    else:
        print(f"\n💥 Failed to extract GPS trajectories")
        sys.exit(1)

if __name__ == "__main__":
    main()