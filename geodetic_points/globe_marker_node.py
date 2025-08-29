#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class GlobeMarkerNode(Node):
    def __init__(self):
        super().__init__('globe_marker')
        
        # Declare parameters with defaults
        self.declare_parameter('mesh_resource', 'package://geodetic_points/meshes/earth.dae')
        self.declare_parameter('scale', 10.0)  # Default 10 meters radius for testing
        self.declare_parameter('frame_id', 'earth')
        self.declare_parameter('publish_once', True)  # Only publish once by default
        self.declare_parameter('publish_frequency', 0.5)  # Publish frequency in Hz (if not publish_once)
        
        # Create publisher
        self.pub = self.create_publisher(Marker, 'earth_globe', 1)
        
        # Track publishing state
        self.has_published = False
        
        # Get parameters
        mesh_resource = self.get_parameter('mesh_resource').value
        scale = self.get_parameter('scale').value
        frame_id = self.get_parameter('frame_id').value
        publish_once = self.get_parameter('publish_once').value
        publish_frequency = self.get_parameter('publish_frequency').value
        
        # Calculate timer period
        if publish_once:
            timer_period = 1.0  # Publish immediately then stop
        else:
            timer_period = 1.0 / publish_frequency if publish_frequency > 0 else 2.0
        
        # Log initialization parameters
        self.get_logger().info("=== GlobeMarkerNode Initialization ===")
        self.get_logger().info(f"mesh_resource: {mesh_resource}")
        self.get_logger().info(f"scale: {scale}")
        self.get_logger().info(f"frame_id: {frame_id}")
        self.get_logger().info(f"publish_once: {publish_once}")
        self.get_logger().info(f"publish_frequency: {publish_frequency} Hz")
        self.get_logger().info(f"timer_period: {timer_period:.2f} seconds")
        self.get_logger().info(f"publisher topic: earth_globe")
        self.get_logger().info("=====================================")
        
        # Start timer after initialization
        self.timer = self.create_timer(timer_period, self.publish_globe)

    def publish_globe(self):
        # Check if we should publish
        publish_once = self.get_parameter('publish_once').value
        
        # If publish_once is enabled and we've already published, skip
        if publish_once and self.has_published:
            self.get_logger().debug("Skipping publish - already published once")
            return
        
        # Get current parameters
        frame_id = self.get_parameter('frame_id').value
        mesh_resource = self.get_parameter('mesh_resource').value
        scale = self.get_parameter('scale').value
        
        # Create marker
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'globe'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        
        # Set lifetime to 0 (forever) - this prevents RViz from removing the marker
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        
        # Set mesh resource
        m.mesh_resource = mesh_resource
        m.mesh_use_embedded_materials = True
        
        # Set position (center of earth frame)
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        
        # Set orientation (identity quaternion)
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        
        # Set scale
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        
        # Set color (blue-green for Earth) - enable this for better visibility
        # m.color.a = 1.0
        # m.color.r = 0.2
        # m.color.g = 0.6
        # m.color.b = 0.8
        
        # Publish marker
        self.pub.publish(m)
        
        # Mark as published
        self.has_published = True
        
        # Log detailed publish information
        self.get_logger().info("=== Published Globe Marker ===")
        self.get_logger().info(f"Frame ID: {frame_id}")
        self.get_logger().info(f"Mesh Resource: {mesh_resource}")
        self.get_logger().info(f"Scale: {scale}")
        self.get_logger().info(f"Position: ({m.pose.position.x}, {m.pose.position.y}, {m.pose.position.z})")
        self.get_logger().info(f"Color RGBA: ({m.color.r}, {m.color.g}, {m.color.b}, {m.color.a})")
        self.get_logger().info(f"Namespace: {m.ns}, ID: {m.id}")
        self.get_logger().info(f"Lifetime: {m.lifetime.sec}s (0=forever)")
        self.get_logger().info("==============================")
        
        # If publish_once is enabled, destroy the timer to stop further publishing
        if publish_once:
            self.get_logger().info("publish_once enabled - stopping timer")
            self.timer.destroy()
            self.timer = None

def main():
    rclpy.init()
    node = GlobeMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()