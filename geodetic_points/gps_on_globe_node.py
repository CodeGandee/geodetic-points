#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque
try:
    from pyproj import Transformer
except ImportError:
    # Fallback for basic lat/lon to approximate ECEF conversion
    import math
    
    class SimpleTransformer:
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
    
    Transformer = None

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

        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.point_radius_m = float(self.get_parameter('point_radius_m').value)
        self.trail_width_m = float(self.get_parameter('trail_width_m').value)
        self.publish_every_n = int(self.get_parameter('publish_every_n').value)

        # LLA -> ECEF transformer
        if Transformer is not None:
            # Using pyproj (EPSG:4979 to EPSG:4978), always_xy = lon,lat,h order
            self.transformer = Transformer.from_crs('EPSG:4979', 'EPSG:4978', always_xy=True)
            self.get_logger().info("Using pyproj for coordinate transformation")
        else:
            # Using fallback simple transformer
            self.transformer = SimpleTransformer()
            self.get_logger().warn("pyproj not available, using simple coordinate transformation")

        self.points_ecef = deque(maxlen=self.max_points)
        self.fix_count = 0

        gps_topic = self.get_parameter('gps_topic').value
        self.sub = self.create_subscription(NavSatFix, gps_topic, self.gps_cb, 10)
        self.pub_points = self.create_publisher(Marker, 'gps_globe_points', 1)
        self.pub_trail = self.create_publisher(Marker, 'gps_globe_trail', 1)

        self.get_logger().info(f"GpsOnGlobeNode started: topic={gps_topic} frame={self.frame_id}")

    def gps_cb(self, msg: NavSatFix):
        self.fix_count += 1
        if self.publish_every_n > 1 and (self.fix_count % self.publish_every_n) != 0:
            return

        # Convert LLA -> ECEF (meters)
        lon = msg.longitude
        lat = msg.latitude
        alt = msg.altitude
        x, y, z = self.transformer.transform(lon, lat, alt)
        self.points_ecef.append((x, y, z))

        # Publish markers
        self.publish_points_marker()
        self.publish_trail_marker()

    def publish_points_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'gps'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = self.point_radius_m
        m.scale.y = self.point_radius_m
        m.scale.z = self.point_radius_m
        m.color.a = 0.9
        m.color.r = 1.0
        m.color.g = 0.1
        m.color.b = 0.1
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.points_ecef]
        self.pub_points.publish(m)

    def publish_trail_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'gps'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = self.trail_width_m  # line width
        m.color.a = 0.8
        m.color.r = 1.0
        m.color.g = 0.85
        m.color.b = 0.1
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.points_ecef]
        self.pub_trail.publish(m)

def main():
    rclpy.init()
    node = GpsOnGlobeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()