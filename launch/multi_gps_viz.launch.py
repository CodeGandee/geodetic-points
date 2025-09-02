#!/usr/bin/env python3
"""
Multi-GPS Globe Visualization Launch File

This launch file provides comprehensive 3D visualization of multiple GPS sources simultaneously:
- Fixed Earth globe visualization with textured mesh
- Multiple GPS trajectory rendering with different colors and styles
- Centralized camera control following the primary GPS trajectory  
- Optional ROS bag replay for comparative data analysis
- Debug output capabilities for all GPS sources

Key Features:
- Support for up to 4 different GPS sources with distinct visual styling
- Primary GPS source controls camera positioning to avoid conflicts
- Each GPS source has configurable colors, sizes, and update rates
- Thread-safe adaptive marker sizing with shared computation

Usage:
  ros2 launch geodetic_points multi_gps_viz.launch.py [arguments]

Key arguments:
  scale:=10                    # Earth scale divisor (1/10 = 0.1x real size)
  primary_gps_topic:=cbs_gnss  # Primary GPS for camera control
  enable_camera_control:=true  # Enable camera following (only for primary GPS)
  bag_file:=/path/to/bag      # Optional bag file for replay
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ============================================================================
    # CONSTANTS AND FILE PATHS
    # ============================================================================
    pkg_share = get_package_share_directory('geodetic_points')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'globe_viz.rviz')
    default_mesh_resource = 'package://geodetic_points/meshes/earth.dae'
    
    # GPS Source Configurations
    # Each GPS source has: topic, name, color (RGB), marker_sizes (point_radius, trail_width)
    gps_sources = {
        'cbs_gnss': {
            'topic': '/cbs_gnss',
            'name': 'cbs',
            'display_name': 'CBS GNSS (Primary)',
            'color_point': (1.0, 0.1, 0.1),    # Red
            'color_trail': (1.0, 0.85, 0.1),   # Orange-red
            'point_radius': 10.0,
            'trail_width': 5.0,
            'max_points': 4000,
            'is_primary': True
        },
        'iphone_gnss': {
            'topic': '/iphone_gnss', 
            'name': 'iphone',
            'display_name': 'iPhone GNSS',
            'color_point': (0.1, 1.0, 0.1),    # Green
            'color_trail': (0.1, 1.0, 0.3),    # Light green
            'point_radius': 10.0,
            'trail_width': 5.0,
            'max_points': 4000,
            'is_primary': False
        },
        'p7pro_gnss': {
            'topic': '/cbs_gnss_p7pro',
            'name': 'p7pro', 
            'display_name': 'P7Pro GNSS',
            'color_point': (0.1, 0.1, 1.0),    # Blue
            'color_trail': (0.3, 0.3, 1.0),    # Light blue
            'point_radius':10.0,
            'trail_width': 5.0,
            'max_points': 4000,
            'is_primary': False
        },
        'standard_gnss': {
            'topic': '/gnss',
            'name': 'gnss',
            'display_name': 'Standard GNSS',
            'color_point': (1.0, 0.1, 1.0),    # Purple
            'color_trail': (1.0, 0.3, 1.0),    # Light purple  
            'point_radius': 10.0,
            'trail_width': 5.0,
            'max_points': 4000,
            'is_primary': False
        }
    }
    
    # ============================================================================
    # GLOBAL/COMMON PARAMETERS
    # ============================================================================
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='10',
        description='Scale divisor for earth globe and GPS coordinates (1/scale applied to nodes). '
                   'Example: scale=10 means Earth will be 1/10th real size (637km radius)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='earth',
        description='Base coordinate frame ID for all visualizations'
    )
    
    # ============================================================================
    # EARTH GLOBE VISUALIZATION PARAMETERS  
    # ============================================================================
    mesh_resource_arg = DeclareLaunchArgument(
        'mesh_resource',
        default_value=default_mesh_resource,
        description='Path to the earth mesh resource file (DAE format with textures)'
    )
    
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='true',
        description='Publish globe marker only once (true) or continuously (false)'
    )
    
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='0.5',
        description='Globe marker publish frequency in Hz (only used if publish_once=false)'
    )
    
    # ============================================================================
    # GPS TRAJECTORY PARAMETERS
    # ============================================================================
    primary_gps_topic_arg = DeclareLaunchArgument(
        'primary_gps_topic',
        default_value='cbs_gnss',
        description='Primary GPS source key for camera control (cbs_gnss, iphone_gnss, p7pro_gnss, standard_gnss)'
    )
    
    publish_every_n_arg = DeclareLaunchArgument(
        'publish_every_n',
        default_value='1',
        description='Publish every Nth GPS point for all sources (1=all points, 10=every 10th point)'
    )
    
    marker_calc_frequency_arg = DeclareLaunchArgument(
        'marker_calc_frequency',
        default_value='1.0',
        description='Shared marker size calculation frequency in Hz (lower = less CPU usage)'
    )
    
    # ============================================================================
    # CAMERA CONTROL PARAMETERS
    # ============================================================================
    enable_camera_control_arg = DeclareLaunchArgument(
        'enable_camera_control',
        default_value='true',
        description='Enable camera control following primary GPS trajectory'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='2620.0',
        description='Camera height above primary GPS trajectory center in meters'
    )
    
    camera_window_size_arg = DeclareLaunchArgument(
        'camera_window_size',
        default_value='15',
        description='Number of recent GPS points used to calculate camera center (larger = smoother)'
    )
    
    camera_min_update_distance_arg = DeclareLaunchArgument(
        'camera_min_update_distance',
        default_value='10.0',
        description='Minimum distance change (meters) required to update camera position'
    )
    
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='2.0454',
        description='Camera yaw angle in radians for primary GPS (horizontal rotation, default: 117.2°)'
    )
    
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch',
        default_value='1.4304', 
        description='Camera pitch angle in radians for primary GPS (vertical tilt, default: 82.0°)'
    )
    
    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Camera roll angle in radians for primary GPS (rotation around viewing axis)'
    )
    
    # ============================================================================
    # DATA REPLAY PARAMETERS
    # ============================================================================
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to ROS bag file for replay (leave empty to disable replay)'
    )
    
    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='10.0',
        description='Bag replay rate multiplier (e.g., 10.0 = 10x speed for multi-GPS comparison)'
    )
    
    # ============================================================================
    # DEBUG/DEVELOPMENT PARAMETERS
    # ============================================================================
    enable_debug_output_arg = DeclareLaunchArgument(
        'enable_debug_output',
        default_value='false',
        description='Enable debug coordinate transformation output (can be verbose with multiple GPS)'
    )
    
    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================
    launch_entities = [
        # ------------------------------------------------------------------------
        # PARAMETER DECLARATIONS
        # ------------------------------------------------------------------------
        # Global parameters
        scale_arg,
        frame_id_arg,
        
        # Earth globe parameters
        mesh_resource_arg,
        publish_once_arg,
        publish_frequency_arg,
        
        # GPS trajectory parameters
        primary_gps_topic_arg,
        publish_every_n_arg,
        marker_calc_frequency_arg,
        
        # Camera control parameters
        enable_camera_control_arg,
        camera_height_arg,
        camera_window_size_arg,
        camera_min_update_distance_arg,
        camera_yaw_arg,
        camera_pitch_arg,
        camera_roll_arg,
        
        # Data replay parameters
        bag_file_arg,
        bag_rate_arg,
        
        # Debug parameters
        enable_debug_output_arg,
        
        # ------------------------------------------------------------------------
        # CORE VISUALIZATION NODES
        # ------------------------------------------------------------------------
        
        # Earth Globe Visualization Node
        # Renders a fixed 3D textured Earth model at the origin
        Node(
            package='geodetic_points',
            executable='globe_marker_node',
            name='earth_globe_visualizer',
            parameters=[{
                'mesh_resource': LaunchConfiguration('mesh_resource'),
                'scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_once': LaunchConfiguration('publish_once'),
                'publish_frequency': LaunchConfiguration('publish_frequency')
            }],
            output='screen'
        ),
    ]
    
    # ------------------------------------------------------------------------
    # MULTI-GPS TRAJECTORY NODES
    # ------------------------------------------------------------------------
    # Generate GPS trajectory nodes dynamically with conflict prevention
    for gps_key, config in gps_sources.items():
        # Determine if this GPS source should have camera control
        is_camera_controller = config['is_primary']
        
        # Create GPS trajectory node with unique naming and configurations
        gps_node = Node(
            package='geodetic_points',
            executable='gps_on_globe_node',
            name=f'gps_{config["name"]}_trajectory',
            namespace=f'multi_gps/{config["name"]}',  # Prevent topic conflicts
            parameters=[{
                # Core parameters
                'frame_id': LaunchConfiguration('frame_id'),
                'gps_topic': config['topic'],
                'scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                
                # Trajectory visualization parameters
                'max_points': config['max_points'],
                'point_radius_m': config['point_radius'],
                'trail_width_m': config['trail_width'],
                'publish_every_n': LaunchConfiguration('publish_every_n'),
                'marker_calc_frequency': LaunchConfiguration('marker_calc_frequency'),
                
                # Color configuration
                'point_color_r': config['color_point'][0],
                'point_color_g': config['color_point'][1], 
                'point_color_b': config['color_point'][2],
                'trail_color_r': config['color_trail'][0],
                'trail_color_g': config['color_trail'][1],
                'trail_color_b': config['color_trail'][2],
                
                # Camera control parameters (only for primary GPS)
                'camera_height': LaunchConfiguration('camera_height') if is_camera_controller else 0.0,
                'camera_window_size': LaunchConfiguration('camera_window_size') if is_camera_controller else 1,
                'camera_min_update_distance': LaunchConfiguration('camera_min_update_distance') if is_camera_controller else 1000.0,
                'camera_yaw': LaunchConfiguration('camera_yaw') if is_camera_controller else 0.0,
                'camera_pitch': LaunchConfiguration('camera_pitch') if is_camera_controller else 0.0,
                'camera_roll': LaunchConfiguration('camera_roll') if is_camera_controller else 0.0,
                
                # Debug parameters (reduced for non-primary to avoid spam)
                'enable_debug_output': LaunchConfiguration('enable_debug_output') if is_camera_controller else False
            }],
            remappings=[
                # Unique topic names to prevent conflicts
                ('gps_globe_points', f'gps_{config["name"]}_points'),
                ('gps_globe_trail', f'gps_{config["name"]}_trail'),
                # Only primary GPS publishes camera TF
                ('camera_view', 'camera_view' if is_camera_controller else f'camera_view_{config["name"]}_disabled')
            ],
            condition=IfCondition('true'),  # Always launch all GPS nodes
            output='screen'
        )
        launch_entities.append(gps_node)
    
    # ------------------------------------------------------------------------
    # DATA REPLAY SYSTEM
    # ------------------------------------------------------------------------
    
    # ROS Bag Replay (Conditional)
    # Only launches if bag_file parameter is provided
    bag_replay = ExecuteProcess(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('bag_file'), "' != ''"
        ])),
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag_file'),
            '--rate', LaunchConfiguration('bag_rate'),
            '--clock'  # Publish simulation time
        ],
        name='multi_gps_bag_player',
        output='screen'
    )
    launch_entities.append(bag_replay)
    
    # ------------------------------------------------------------------------
    # VISUALIZATION FRONTEND
    # ------------------------------------------------------------------------
    
    # RViz2 3D Visualization
    # Provides interactive 3D visualization interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='multi_gps_visualizer',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    launch_entities.append(rviz_node)
    
    return LaunchDescription(launch_entities)