#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import rclpy.time
import rclpy.duration

class GlobeMarkerNode(Node):
    def __init__(self):
        super().__init__('globe_marker')
        
        # Declare parameters with defaults
        self.declare_parameter('mesh_resource', 'package://geodetic_points/meshes/earth.dae')
        self.declare_parameter('scale', 10.0)  # Default 10 meters radius for testing
        self.declare_parameter('frame_id', 'earth')
        self.declare_parameter('publish_once', True)  # Only publish once by default
        self.declare_parameter('publish_frequency', 0.5)  # Publish frequency in Hz (if not publish_once)
        # Remove follow_gps_center parameter - earth should stay fixed
        
        # Create publisher
        self.pub = self.create_publisher(Marker, 'earth_globe', 1)
        
        # Track publishing state
        self.has_published = False
        
        
        # Get parameters
        self.mesh_resource = self.get_parameter('mesh_resource').value
        scale = self.get_parameter('scale').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_once = self.get_parameter('publish_once').value
        publish_frequency = self.get_parameter('publish_frequency').value
        # Earth should stay fixed at origin - no GPS following
        
        # Earth stays fixed at origin position
        self.globe_position = [0.0, 0.0, 0.0]
        
        # TF broadcaster for earth frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Type checking and validation for scale parameter
        if not isinstance(scale, (int, float)) or scale <= 0:
            self.get_logger().error(f"Invalid scale parameter: {scale}. Must be a positive number.")
            raise ValueError(f"Scale parameter must be positive, got: {scale}")
        
        # Apply scale to all affected variables once during initialization
        self.earth_radius = 6371000.0 
        self.earth_scale_x = self.earth_radius*scale
        self.earth_scale_y = self.earth_radius*scale
        self.earth_scale_z = self.earth_radius*scale

        # Calculate timer period
        if self.publish_once:
            timer_period = 1.0  # Publish immediately then stop
        else:
            timer_period = 1.0 / publish_frequency if publish_frequency > 0 else 2.0
        
        # Log all initialization parameters
        self._log_initialization_parameters(scale, publish_frequency, timer_period)
        
        # Start timer after initialization
        self.timer = self.create_timer(timer_period, self.publish_globe)

    def _log_initialization_parameters(self, scale, publish_frequency, timer_period):
        """Centralized logging function for initialization parameters"""
        self.get_logger().info("=== GlobeMarkerNode Initialization ===")
        self.get_logger().info(f"Mesh Resource: {self.mesh_resource}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info(f"Scale Factor (received): {scale}")
        
        if scale == 1.0:
            self.get_logger().info("Scale = 1.0: Using real Earth size (6,371 km radius)")
            self.get_logger().info("Earth display radius: 6,371,000 m")
        else:
            self.get_logger().info(f"Scale = {scale}: Earth scaled by factor {scale}")
            earth_radius_scaled = 6371000.0 * scale
            if scale < 1.0:
                self.get_logger().info(f"Earth will be {1/scale:.0f}x smaller than real size {self.earth_radius} m")
                self.get_logger().info(f"Earth display radius: {earth_radius_scaled:.3f} m")
            else:
                self.get_logger().info(f"Earth will be {scale:.0f}x larger than real size {self.earth_radius} m")
                self.get_logger().info(f"Earth display radius: {earth_radius_scaled:.0f} m")
        
        self.get_logger().info(f"Applied Scale - X: {self.earth_scale_x}, Y: {self.earth_scale_y}, Z: {self.earth_scale_z}")
        self.get_logger().info(f"Publish Once: {self.publish_once}")
        self.get_logger().info(f"Publish Frequency: {publish_frequency} Hz")
        self.get_logger().info(f"Timer Period: {timer_period:.2f} seconds")
        self.get_logger().info(f"Publisher Topic: earth_globe")
        self.get_logger().info("=======================================")
    
    # Remove GPS trajectory following - earth stays fixed

    def publish_base_earth_frame(self):
        """发布基础earth坐标系"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # 父坐标系
        t.child_frame_id = 'earth'  # 子坐标系
        
        # earth坐标系相对于map坐标系的位置和姿态
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def publish_globe(self):
        # 首先发布基础earth坐标系
        self.publish_base_earth_frame()
        
        # Earth stays fixed at origin - no position updates
        if not self.has_published:
        
            # Create marker using pre-calculated scaled values
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'globe'
            m.id = 0
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            
            # Set lifetime to 0 (forever) - this prevents RViz from removing the marker
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            
            # Set mesh resource
            m.mesh_resource = self.mesh_resource
            m.mesh_use_embedded_materials = True
            
            # Earth fixed at origin position
            m.pose.position.x = 0.0
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0

            # Set orientation (identity quaternion)
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            
            # Use pre-calculated scaled values
            m.scale.x = self.earth_scale_x
            m.scale.y = self.earth_scale_y
            m.scale.z = self.earth_scale_z
            
            # Publish marker
            self.pub.publish(m)
            
            # Mark as published and log
            self.has_published = True
            self._log_publish_info(m)
            
            self.get_logger().info("Earth globe published at fixed origin position (0, 0, 0)")
            
            # If publish_once is enabled, destroy the timer to stop further publishing
            if self.publish_once:
                self.get_logger().info("publish_once enabled - stopping timer")
                self.timer.destroy()
                self.timer = None

    def _log_publish_info(self, marker):
        """Centralized logging function for publish information"""
        self.get_logger().info("=== Published Globe Marker ===")
        self.get_logger().info(f"Frame ID: {marker.header.frame_id}")
        self.get_logger().info(f"Mesh Resource: {marker.mesh_resource}")
        self.get_logger().info(f"Applied Scale XYZ: ({marker.scale.x}, {marker.scale.y}, {marker.scale.z})")
        self.get_logger().info(f"Position: ({marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z})")
        self.get_logger().info(f"Namespace: {marker.ns}, ID: {marker.id}")
        self.get_logger().info(f"Lifetime: {marker.lifetime.sec}s (0=forever)")
        self.get_logger().info("==============================")

def main():
    rclpy.init()
    node = GlobeMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()