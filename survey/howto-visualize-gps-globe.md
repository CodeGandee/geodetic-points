# How to Visualize GPS and VIO Data on a Textured Globe in RViz2

This guide shows how to render:
1. A textured Earth sphere.
2. GPS track (`gps/fix` and transformed VIO points) in global coordinates.
3. Axes frames (ECEF, ENU, VIO) for spatial context.

Target: ROS 2 Jazzy+ with RViz2.

## Assets (Public Domain / Permissive)

Use NASA Blue Marble textures (public domain):
* NASA Visible Earth Collection (Blue Marble): https://visibleearth.nasa.gov/collection/1484/blue-marble
* Example single image (Wikimedia, 2002): https://commons.wikimedia.org/wiki/File:Blue_Marble_2002.png

Download a medium resolution (e.g., 8192x4096) equirectangular JPEG/PNG to `share/textures/earth.jpg` inside your ROS package.

## Coordinate Frames

* `earth` (ECEF origin, axes per [ECEF](https://en.wikipedia.org/wiki/ECEF)).
* `enu_anchor` (local tangent plane, see [Local tangent plane](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates)).
* `vio_base` (local VIO frame).

Static transforms published via `static_transform_publisher` or a Python node.

## Strategy

RViz2 cannot apply image textures directly to a procedural sphere out of the box; we have two practical options:

1. Marker Mesh Resource: Create a unit UV sphere mesh (e.g., in Blender) exported as `earth.dae` or `earth.glb` with material referencing `earth.jpg`. Then publish it using a `visualization_msgs/Marker` of type `MESH_RESOURCE`.
2. Marker Triangles: Procedurally generate a low‑poly sphere with vertices + triangles and publish as `Marker` type `MESH_RESOURCE` pointing to a locally installed mesh, or type `TRIANGLE_LIST` if embedding geometry (but no texture support for TRIPLE_LIST with image in RViz2).

Approach 1 (mesh resource + texture) yields texture; Approach 2 is simpler but untextured.

## Package File Layout Example

```
my_globe_viz/
  package.xml
  setup.py
  resource/
  my_globe_viz/
    __init__.py
    globe_marker_node.py
  meshes/
    earth.dae
  share/textures/
    earth.jpg
  launch/
    globe_viz.launch.py
```

Ensure `package.xml` exports `<exec_depend>rviz2</exec_depend>` and `<exec_depend>visualization_msgs</exec_depend>`.

## Generating the Mesh (External Step)

In Blender: Add UV sphere → Shade Smooth → UV unwrap (should already be proper for default sphere) → Assign material with image texture `earth.jpg` (equirectangular). Export as Collada (`earth.dae`). Place under `meshes/`.

## Globe Marker Node (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class GlobeMarkerNode(Node):
    def __init__(self):
        super().__init__('globe_marker')
        self.pub = self.create_publisher(Marker, 'earth_globe', 1)
        self.timer = self.create_timer(2.0, self.publish_globe)
        self.declare_parameter('mesh_resource', 'package://my_globe_viz/meshes/earth.dae')
        self.declare_parameter('scale', 6371000.0)  # mean Earth radius (m)
        self.declare_parameter('frame_id', 'earth')

    def publish_globe(self):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'globe'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = self.get_parameter('mesh_resource').value
        m.mesh_use_embedded_materials = True
        R = self.get_parameter('scale').value
        s = R * 1.0  # sphere radius
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        self.pub.publish(m)

def main():
    rclpy.init()
    node = GlobeMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Live GPS Overlay Node (Python)

This node subscribes to `gps/fix` (`sensor_msgs/NavSatFix`), converts lat/lon/alt (WGS84) to ECEF, and publishes:
- A red point set (Marker SPHERE_LIST) for recent GPS fixes
- A yellow trail (Marker LINE_STRIP) showing the path
All geometry is published in the `earth` frame so it overlays the textured globe.

Dependencies: `pyproj` (see [about-geodetic-compute-library.md](gps-points/about-geodetic-compute-library.md))

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque
from pyproj import Transformer

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

        # LLA -> ECEF transformer (EPSG:4979 to EPSG:4978), always_xy = lon,lat,h order
        self.transformer = Transformer.from_crs('EPSG:4979', 'EPSG:4978', always_xy=True)

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
```

Scaling: Earth radius (mean) ≈ 6,371,000 m ([Wikipedia: Earth radius](https://en.wikipedia.org/wiki/Earth_radius)). If your ECEF data is also in meters, the sphere will match scale (may appear enormous in RViz—use camera controls, or downscale both sphere and data consistently by factor, e.g. 1e-3, and note this in your frame naming).

## Displaying GPS and VIO Points

1. Publish `vio/points3d_geodetic` in ECEF frame (`earth`).
2. Add RViz2 displays:
   * Marker (topic `earth_globe`).
   * PointCloud2 for `vio/points3d_geodetic`.
   * PointCloud2 for raw/local cloud (optionally) with TF.
   * NavSatFix points (`vio/points3d_pts`) via Point display (set style = Spheres).
3. Add `TF` display to see axes.

## Optional: Local ENU Overlay

Publish a small axis marker (Marker type `AXES`) at the ENU origin to clarify orientation. Use a transform chain `earth`→`enu_anchor` (static) and `enu_anchor`→`vio_base` (dynamic if fused pose changes).

## Handling Large Point Clouds

* Use RViz2 point decimation.
* Downsample on the publisher side (voxel grid filter) for interactive rates.
* Consider `Intensity` or `RGB` fields for coloring by altitude or time.

## Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Textured globe mesh
        Node(
            package='my_globe_viz',
            executable='globe_marker_node',
            parameters=[{
                'mesh_resource': 'package://my_globe_viz/meshes/earth.dae',
                'scale': 6371000.0,   # meters
                'frame_id': 'earth'
            }],
            output='screen'
        ),
        # Live GPS overlay (points + trail) in ECEF/earth frame
        Node(
            package='my_globe_viz',
            executable='gps_on_globe_node',
            parameters=[{
                'frame_id': 'earth',
                'gps_topic': 'gps/fix',
                'max_points': 4000,
                'point_radius_m': 50000.0,
                'trail_width_m': 80000.0,
                'publish_every_n': 1
            }],
            output='screen'
        )
    ])
```

Run RViz2 with an RViz config that subscribes to the marker and point topics.

## Texture Attribution

NASA Blue Marble imagery is public domain per NASA policy (https://visibleearth.nasa.gov). When redistributing, include: "Image courtesy of NASA Visible Earth / Blue Marble collection." (Attribution helpful though not legally required in U.S.).

## References

* ECEF: https://en.wikipedia.org/wiki/ECEF
* Local tangent plane (ENU): https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
* Earth radius: https://en.wikipedia.org/wiki/Earth_radius
* NASA Blue Marble: https://visibleearth.nasa.gov/collection/1484/blue-marble
* Example texture (Wikimedia): https://commons.wikimedia.org/wiki/File:Blue_Marble_2002.png
* RViz2 Markers: https://docs.ros.org/

---
You now have a globe + global data visualization pipeline in RViz2.