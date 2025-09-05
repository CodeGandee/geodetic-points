#!/usr/bin/env python3
"""
Topic Monitor Node for GPS-VIO Calibration Testing

This node monitors specified ROS2 topics and logs them in two formats:
1. Pandas DataFrame format (CSV files)
2. ROS2 log info messages

Each topic is logged to a separate file based on the topic name.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import os
import json
import csv
from datetime import datetime
from threading import Lock
from collections import defaultdict

# Message imports
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage


class TopicMonitorNode(Node):
    """
    ROS2 node for monitoring and logging multiple topics.
    """
    
    def __init__(self):
        super().__init__('topic_monitor_node')
        
        # Parameters
        self.declare_parameter('log_dir', './log')
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('buffer_size', 100)
        self.declare_parameter('auto_save_interval', 10.0)
        
        # Topics to monitor (can be extended via parameters)
        self.declare_parameter('topics_to_monitor', [
            '/calibration/transform_earth_odom',
            '/tf_static',
            '/vio/pose_earth'
        ])
        
        # Get parameters
        self.log_dir = self.get_parameter('log_dir').value
        self.monitor_rate = self.get_parameter('monitor_rate_hz').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        self.topics_to_monitor = self.get_parameter('topics_to_monitor').value
        
        # Create log directory if it doesn't exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        # Generate timestamp for this session
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Data storage
        self.data_buffers = defaultdict(list)
        self.data_lock = Lock()
        self.message_counts = defaultdict(int)
        
        # Create CSV files for each topic
        self.csv_files = {}
        self.init_csv_files()
        
        # Subscribe to topics
        self._subscriptions = []
        self.create_subscriptions()
        
        # Create timer for auto-saving
        self.save_timer = self.create_timer(
            self.auto_save_interval,
            self.save_all_data
        )
        
        # Create timer for status monitoring
        self.monitor_timer = self.create_timer(
            1.0 / self.monitor_rate,  # Convert Hz to seconds
            self.print_status
        )
        
        # Log initialization
        self.get_logger().info(f'Topic Monitor Node initialized')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'Session timestamp: {self.session_timestamp}')
        self.get_logger().info(f'Monitoring topics: {self.topics_to_monitor}')
        
    def init_csv_files(self):
        """Initialize CSV files for each topic."""
        for topic in self.topics_to_monitor:
            # Clean topic name for filename
            clean_topic = topic.replace('/', '_').strip('_')
            
            # Create CSV filename
            csv_filename = os.path.join(
                self.log_dir,
                f'topic_monitor_{clean_topic}_{self.session_timestamp}.csv'
            )
            self.csv_files[topic] = csv_filename
            
            self.get_logger().info(f'CSV file for {topic}: {csv_filename}')
    
    def create_subscriptions(self):
        """Create subscriptions for all monitored topics."""
        
        # QoS profile for reliable communication (using VOLATILE for regular topics)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for tf_static (latched topic)
        qos_tf_static = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        for topic in self.topics_to_monitor:
            if topic == '/calibration/transform_earth_odom':
                sub = self.create_subscription(
                    TransformStamped,
                    topic,
                    lambda msg, t=topic: self.transform_callback(msg, t),
                    qos_profile
                )
                self._subscriptions.append(sub)
                
            elif topic == '/tf_static':
                sub = self.create_subscription(
                    TFMessage,
                    topic,
                    lambda msg, t=topic: self.tf_callback(msg, t),
                    qos_tf_static
                )
                self._subscriptions.append(sub)
                
            elif topic == '/vio/pose_earth':
                sub = self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, t=topic: self.pose_callback(msg, t),
                    qos_profile
                )
                self._subscriptions.append(sub)
                
            self.get_logger().info(f'Subscribed to {topic}')
    
    def transform_callback(self, msg: TransformStamped, topic: str):
        """Handle TransformStamped messages."""
        with self.data_lock:
            # Extract data
            data = {
                'timestamp': datetime.now().isoformat(),
                'ros_time_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                'wall_time_iso': datetime.now().isoformat(),
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'translation_x': msg.transform.translation.x,
                'translation_y': msg.transform.translation.y,
                'translation_z': msg.transform.translation.z,
                'rotation_x': msg.transform.rotation.x,
                'rotation_y': msg.transform.rotation.y,
                'rotation_z': msg.transform.rotation.z,
                'rotation_w': msg.transform.rotation.w
            }
            
            # Add to buffer
            self.data_buffers[topic].append(data)
            self.message_counts[topic] += 1
            
            # Log info
            self.get_logger().info(
                f'[{topic}] Transform: frame={msg.header.frame_id}->{msg.child_frame_id}, '
                f'translation=({msg.transform.translation.x:.3f}, '
                f'{msg.transform.translation.y:.3f}, {msg.transform.translation.z:.3f})'
                f'rotation=({msg.transform.rotation.x:.3f}, '
                f'{msg.transform.rotation.y:.3f}, {msg.transform.rotation.z:.3f}, {msg.transform.rotation.w:.3f})'
            )
            
            # Check buffer size and save if needed
            if len(self.data_buffers[topic]) >= self.buffer_size:
                self.save_topic_data(topic)
    
    def tf_callback(self, msg: TFMessage, topic: str):
        """Handle TFMessage messages, filtering for earth->odom transforms."""
        with self.data_lock:
            for transform in msg.transforms:
                # Only process earth->odom transforms
                if transform.header.frame_id.lstrip('/') == 'earth' and transform.child_frame_id.lstrip('/') == 'odom':
                    data = {
                        'timestamp': datetime.now().isoformat(),
                        'ros_time_ns': transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                        'wall_time_iso': datetime.now().isoformat(),
                        'frame_id': transform.header.frame_id,
                        'child_frame_id': transform.child_frame_id,
                        'translation_x': transform.transform.translation.x,
                        'translation_y': transform.transform.translation.y,
                        'translation_z': transform.transform.translation.z,
                        'rotation_x': transform.transform.rotation.x,
                        'rotation_y': transform.transform.rotation.y,
                        'rotation_z': transform.transform.rotation.z,
                        'rotation_w': transform.transform.rotation.w
                    }
                    
                    # Add to buffer
                    self.data_buffers[topic].append(data)
                    self.message_counts[topic] += 1
                    
                    # Log info
                    self.get_logger().info(
                        f'[{topic}] TF Static: earth->odom, '
                        f'translation=({transform.transform.translation.x:.3f}, '
                        f'{transform.transform.translation.y:.3f}, '
                        f'{transform.transform.translation.z:.3f})'
                        f'rotation=({transform.transform.rotation.x:.3f}, '
                        f'{transform.transform.rotation.y:.3f}, {transform.transform.rotation.z:.3f}, {transform.transform.rotation.w:.3f})'
                    )

                    # Check buffer size and save if needed
                    if len(self.data_buffers[topic]) >= self.buffer_size:
                        self.save_topic_data(topic)
    
    def pose_callback(self, msg: PoseStamped, topic: str):
        """Handle PoseStamped messages."""
        with self.data_lock:
            # Extract data
            data = {
                'timestamp': datetime.now().isoformat(),
                'ros_time_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                'wall_time_iso': datetime.now().isoformat(),
                'frame_id': msg.header.frame_id,
                'position_x': msg.pose.position.x,
                'position_y': msg.pose.position.y,
                'position_z': msg.pose.position.z,
                'orientation_x': msg.pose.orientation.x,
                'orientation_y': msg.pose.orientation.y,
                'orientation_z': msg.pose.orientation.z,
                'orientation_w': msg.pose.orientation.w
            }
            
            # Add to buffer
            self.data_buffers[topic].append(data)
            self.message_counts[topic] += 1
            
            # Log info
            self.get_logger().info(
                f'[{topic}] Pose: frame={msg.header.frame_id}, '
                f'position=({msg.pose.position.x:.3f}, '
                f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), '
                f'orientation=({msg.pose.orientation.x:.3f}, '
                f'{msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})'
            )
            
            # Check buffer size and save if needed
            if len(self.data_buffers[topic]) >= self.buffer_size:
                self.save_topic_data(topic)
    
    def save_topic_data(self, topic: str):
        """Save buffered data for a specific topic to CSV."""
        if topic not in self.data_buffers or not self.data_buffers[topic]:
            return
            
        # Extract buffer data inside lock, then write outside
        rows_to_write = None
        with self.data_lock:
            if self.data_buffers[topic]:
                rows_to_write = self.data_buffers[topic].copy()
                self.data_buffers[topic].clear()
        
        if rows_to_write:
            self._flush_topic_buffer_to_csv(topic, rows_to_write)
    
    def _flush_topic_buffer_to_csv(self, topic: str, rows: list):
        """Write rows to CSV file in append mode."""
        try:
            csv_file = self.csv_files[topic]
            file_exists = os.path.exists(csv_file) and os.path.getsize(csv_file) > 0
            
            # Write to CSV file
            with open(csv_file, 'a', newline='') as f:
                if rows:
                    # Get field names from first row
                    fieldnames = rows[0].keys()
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    
                    # Write header only if file is new/empty
                    if not file_exists:
                        writer.writeheader()
                    
                    # Write all rows
                    writer.writerows(rows)
            
            self.get_logger().info(
                f'Saved {len(rows)} records for {topic} to {csv_file}'
            )
            
        except Exception as e:
            self.get_logger().error(
                f'Error saving data for {topic}: {str(e)}'
            )
    
    def save_all_data(self):
        """Timer callback to save all buffered data."""
        # Save data for each topic
        for topic in self.topics_to_monitor:
            self.save_topic_data(topic)
        
        # Log status
        with self.data_lock:
            self.get_logger().info(
                f'Status: {", ".join([f"{t}={self.message_counts[t]} msgs" for t in self.topics_to_monitor])}'
            )
    
    def print_status(self):
        """Timer callback to print status information."""
        with self.data_lock:
            buffer_sizes = {topic: len(self.data_buffers[topic]) for topic in self.topics_to_monitor}
        
        self.get_logger().info(
            f'Monitor Status - Messages: {", ".join([f"{t}={self.message_counts[t]}" for t in self.topics_to_monitor])}, '
            f'Buffered: {", ".join([f"{t}={buffer_sizes[t]}" for t in self.topics_to_monitor])}'
        )
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        # Save any remaining data
        self.save_all_data()
        
        # Log final statistics
        self.get_logger().info('Topic Monitor Node shutting down')
        self.get_logger().info('Final message counts:')
        for topic, count in self.message_counts.items():
            self.get_logger().info(f'  {topic}: {count} messages')
            csv_file = self.csv_files[topic]
            if os.path.exists(csv_file):
                file_size = os.path.getsize(csv_file) / 1024  # KB
                self.get_logger().info(f'    CSV file: {csv_file} ({file_size:.2f} KB)')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TopicMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()