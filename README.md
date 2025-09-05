# Geodetic Points - VIO+GPS Navigation System

A comprehensive **ROS2-based Visual-Inertial Odometry (VIO) + GPS navigation and geodetic point localization system** that provides real-time sensor fusion, 3D Earth visualization, and precise coordinate transformations between local VIO frames and global geodetic coordinates.

## 🎯 Project Overview

This system enables precise integration of VIO (Visual-Inertial Odometry) and GPS data for applications requiring accurate global positioning and trajectory visualization. It transforms local VIO coordinates to Earth-Centered Earth-Fixed (ECEF) coordinates and provides rich 3D visualization capabilities.

### Key Features
- 🌍 **Real-time VIO-GPS Calibration**: Time synchronization and spatial alignment between VIO and GPS data
- 🗺️ **3D Earth Visualization**: Interactive globe with GPS trajectories and VIO point clouds  
- 📡 **Multi-Topic GPS Support**: Automatic detection and processing of multiple GPS sources
- 🎥 **Bag Data Processing**: Extract and convert ROS2 bag data to various formats (CSV, JSON, KML)
- 📊 **Trajectory Analysis**: Comprehensive statistics and quality metrics
- 🔧 **Modular Architecture**: Configurable nodes for different use cases

## 📁 Project Structure

### 🔧 Core ROS2 Nodes (`geodetic_points/`)
- **`gps_vio_calibration_node.py`** - GPS-VIO time and spatial calibration engine
- **`globe_marker_node.py`** - 3D Earth globe visualization with textured mesh
- **`gps_on_globe_node.py`** - Real-time GPS trajectory visualization on Earth
- **`vio_earth_visualization_node.py`** - VIO data transformation and Earth coordinate visualization
- **`topic_monitor_node.py`** - ROS topic monitoring and diagnostics

### 🚀 Launch Files (`launch/`)
- **`vio_gps_integration.launch.py`** - Complete VIO-GPS system with visualization
- **`globe_viz.launch.py`** - Earth globe and GPS trajectory visualization
- **`multi_gps_viz.launch.py`** - Multi-GPS source visualization
- **`sigle_calibration_node.launch.py`** - Standalone calibration system
- **`test_calibration_globe.launch.py`** - Testing and validation setup

### 🛠️ Data Processing Tools (`scripts/`)
- **`bag2kml.py`** - Convert ROS2 bag GPS data to Google Earth KML format
- **`extract_bag_data.py`** - Unified GPS and odometry data extraction with progress bars
- **`offline_calibration.py`** - Batch processing and offline calibration
- **`experiment/extract_bag_data.py`** - Enhanced extraction tool with colorized progress and validation

### 📚 Documentation (`doc/`)
Technical documentation covering system issues, solutions, and usage guides:
- System troubleshooting and error analysis
- Coordinate transformation explanations  
- Integration guides and best practices
- Performance optimization techniques

### 📖 Research & Planning (`survey/`, `plan/`)
- **`survey/`** - Technical research on geodetic libraries and algorithms
- **`plan/`** - Project roadmap and offline calibration strategies

### 🎨 Visualization Assets (`meshes/`, `textures/`, `rviz/`)
- **`meshes/earth.dae`** - High-quality 3D Earth model
- **`textures/`** - Earth surface textures
- **`rviz/*.rviz`** - Pre-configured RViz visualization setups

### 🔬 Testing & Results (`tests/`, `results/`)
- **`tests/`** - Comprehensive test suites with performance benchmarks
- **`results/`** - Sample outputs, screenshots, and trajectory data

### 📝 Logs & Build (`log/`, `build/`, `install/`)
- **`log/`** - System logs, calibration results, and extraction reports
- **`build/`** - Colcon build artifacts
- **`install/`** - ROS2 package installation files

## 🚀 Quick Start

### Prerequisites
```bash
# Install ROS2 dependencies
sudo apt install ros-humble-desktop python3-pip

# Install Python dependencies  
pip install -r requirements.txt
```

### Build & Install
```bash
# Build the ROS2 package
./build.sh

# Source the setup
source install/setup.bash
```

### Launch Complete System
```bash
# Launch VIO-GPS integration with 3D visualization
ros2 launch geodetic_points vio_gps_integration.launch.py

# Or launch individual components
ros2 launch geodetic_points globe_viz.launch.py scale:=10
```

### Extract Data from ROS2 Bags
```bash
# Extract GPS and odometry data with enhanced progress bars
python3 scripts/experiment/extract_bag_data.py \
  --bag_path /path/to/bag \
  --output_dir results/ \
  --enable_logging \
  --format both

# Convert GPS trajectories to KML for Google Earth  
python3 scripts/bag2kml.py \
  --bag_path /path/to/bag \
  --output_dir kml_output/
```

## 🔧 System Architecture

### Coordinate Frames
- **`earth`** - ECEF (Earth-Centered Earth-Fixed) global reference
- **`odom`** - VIO local odometry frame  
- **`base_link`** - Vehicle/sensor platform frame
- **`camera_*`** - Individual camera frames

### Data Flow
```
GPS Sensors ──┐
              ├── Time Sync ──── Spatial Calibration ──── Earth Transform
VIO System ───┘                                            │
                                                           ├── Visualization
Point Clouds ──── Transform ─────────────────────────────┘
```

### Key Transformations
1. **Time Synchronization**: `t_corrected = a * t_gps + b`
2. **Spatial Calibration**: Rigid transform via Kabsch algorithm
3. **Coordinate Conversion**: `odom → earth` via TF2 transforms
4. **Geodetic Projection**: ECEF ↔ WGS84 (lat/lon/alt)

## 📊 Features & Capabilities

### GPS-VIO Calibration
- Automatic time offset and drift compensation
- Spatial alignment using point cloud registration  
- Real-time calibration quality metrics
- Robust outlier detection and filtering

### Visualization
- Interactive 3D Earth globe with realistic textures
- Multi-colored GPS trajectory overlays
- VIO trajectory and point cloud rendering
- Adaptive camera control and trajectory following
- RViz integration with custom marker types

### Data Processing
- Multi-format bag data extraction (CSV, JSON, KML)
- Colorized progress bars with real-time statistics
- Data validation and quality assessment
- Memory-efficient processing of large datasets
- Comprehensive logging and error reporting

## 🔬 Testing & Validation

Run the comprehensive test suite:
```bash
# Enhanced functionality tests
python3 tests/test_extract_bag_data_enhanced.py --verbose

# Performance benchmarks
python3 tests/test_extract_bag_data_enhanced.py --benchmark
```

## 📈 Project Status

**Status**: ✅ **Production Ready** - Core functionality implemented and tested

### Completed Features
- ✅ GPS-VIO time and spatial calibration
- ✅ 3D Earth visualization with trajectory overlay
- ✅ ROS2 bag data extraction and conversion  
- ✅ Multi-format export (CSV, JSON, KML)
- ✅ Comprehensive testing and validation
- ✅ Enhanced progress tracking and logging

### Future Enhancements  
- 🔄 Real-time RTK GPS integration
- 🔄 Advanced sensor fusion algorithms
- 🔄 Web-based visualization interface
- 🔄 Multi-robot coordinate system support

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support & Documentation

- **Technical Issues**: Check `doc/` for troubleshooting guides
- **Usage Examples**: See `launch/` files and script documentation  
- **Research Background**: Review `survey/` for algorithm references
- **System Logs**: Monitor `log/` directory for detailed execution logs

---

**Built with ROS2 Humble • Python 3.10+ • OpenCV • SciPy • PyProj**
