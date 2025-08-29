# GPS Globe Visualization Usage Guide

## Basic Usage

### 1. Default Launch
Launch with default parameters (scale=10.0 meters):
```bash
source install/setup.bash
ros2 launch geodetic_points globe_viz.launch.py
```

### 2. Custom Scale Parameter
Launch with custom scale (e.g., 20 meters radius):
```bash
ros2 launch geodetic_points globe_viz.launch.py scale:=20.0
```

### 3. Multiple Parameters
Launch with custom parameters:
```bash
ros2 launch geodetic_points globe_viz.launch.py \
  scale:=100.0 \
  frame_id:=world \
  gps_topic:=my_gps/fix
```

### 4. Real Earth Scale
Launch with actual Earth radius (6.37 million meters):
```bash
ros2 launch geodetic_points globe_viz.launch.py scale:=6371000.0
```

### 5. Optimize Rendering Performance
#### Publish Once (Default - Recommended)
Only publish globe marker once to reduce rendering load:
```bash
ros2 launch geodetic_points globe_viz.launch.py publish_once:=true
```

#### Continuous Publishing with Custom Frequency  
For dynamic updates, publish at lower frequency:
```bash
ros2 launch geodetic_points globe_viz.launch.py \
  publish_once:=false \
  publish_frequency:=0.1
```

## Available Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `scale` | `10.0` | Earth globe radius in meters |
| `mesh_resource` | `package://geodetic_points/meshes/earth.dae` | Path to earth mesh file |
| `frame_id` | `earth` | Frame ID for the earth globe |
| `gps_topic` | `gps/fix` | GPS topic name |
| `publish_once` | `true` | Publish globe marker only once (true/false) |
| `publish_frequency` | `0.5` | Publish frequency in Hz (only if publish_once=false) |

## View Launch Arguments
To see all available parameters and their descriptions:
```bash
ros2 launch geodetic_points globe_viz.launch.py --show-args
```

## Individual Node Testing

### Test Globe Marker Node
```bash
ros2 run geodetic_points globe_marker_node --ros-args -p scale:=5.0
```

### Test GPS Overlay Node
```bash
ros2 run geodetic_points gps_on_globe_node --ros-args -p gps_topic:=my_gps/fix
```

## Troubleshooting

1. **Scale Issues**: Start with small scale (10-100 meters) for testing
2. **Display Issues**: Ensure RViz2 can connect to display (X11 forwarding for remote systems)
3. **GPS Data**: Publish test GPS data to `/gps/fix` topic using `sensor_msgs/NavSatFix`