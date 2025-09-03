#!/usr/bin/env python3
"""
Globe Visualization Launch File

This launch file provides a comprehensive 3D GPS visualization system featuring:
- Fixed Earth globe visualization with textured mesh
- Real-time GPS trajectory rendering with adaptive marker sizing  
- Camera control that follows GPS trajectory with windowed positioning
- Optional ROS bag replay for data analysis
- Debug output capabilities

Usage:
  ros2 launch geodetic_points globe_viz.launch.py [arguments]

Key arguments:
  scale:=10                    # Earth scale divisor (1/10 = 0.1x real size)
  gps_topic:=gps/fix          # GPS data topic
  camera_height:=100.0        # Camera height above trajectory center
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
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='gps/fix',
        description='ROS topic name for GPS NavSatFix messages'
    )
    
    max_points_arg = DeclareLaunchArgument(
        'max_points',
        default_value='4000',
        description='Maximum number of GPS points to display in trajectory'
    )
    
    point_radius_arg = DeclareLaunchArgument(
        'point_radius_m',
        default_value='10.0',
        description='Base radius of GPS point markers in meters (before adaptive scaling)'
    )
    
    trail_width_arg = DeclareLaunchArgument(
        'trail_width_m',
        default_value='5.0',
        description='Base width of GPS trail line in meters (before adaptive scaling)'
    )
    
    publish_every_n_arg = DeclareLaunchArgument(
        'publish_every_n',
        default_value='1',
        description='Publish every Nth GPS point (1=all points, 10=every 10th point)'
    )
    
    # ============================================================================
    # CAMERA CONTROL PARAMETERS
    # ============================================================================
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='2620.0',
        description='Camera height above GPS trajectory center in meters'
    )
    
    camera_window_size_arg = DeclareLaunchArgument(
        'camera_window_size',
        default_value='10',
        description='Number of recent GPS points used to calculate camera center position'
    )
    
    camera_min_update_distance_arg = DeclareLaunchArgument(
        'camera_min_update_distance',
        default_value='5.0',
        description='Minimum distance change (meters) required to update camera position'
    )
    
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='2.0454',
        description='Camera yaw angle in radians (horizontal rotation, default from screenshot: 117.2°)'
    )
    
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', 
        default_value='1.4304',
        description='Camera pitch angle in radians (vertical tilt, default from screenshot: 82.0°)'
    )
    
    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll',
        default_value='0.0',
        description='Camera roll angle in radians (rotation around viewing axis)'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_view',
        description='TF frame ID for camera (empty string disables camera TF publishing)'
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
        default_value='20.0',
        description='Bag replay rate multiplier (e.g., 20.0 = 20x speed)'
    )
    
    # ============================================================================
    # DEBUG/DEVELOPMENT PARAMETERS
    # ============================================================================
    enable_debug_output_arg = DeclareLaunchArgument(
        'enable_debug_output',
        default_value='true',
        description='Enable debug coordinate transformation output topics and logging'
    )
    
    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================
    return LaunchDescription([
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
        gps_topic_arg,
        max_points_arg,
        point_radius_arg,
        trail_width_arg,
        publish_every_n_arg,
        
        # Camera control parameters
        camera_height_arg,
        camera_window_size_arg,
        camera_min_update_distance_arg,
        camera_yaw_arg,
        camera_pitch_arg,
        camera_roll_arg,
        camera_frame_id_arg,
        
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
        
        # GPS Trajectory & Camera Control Node
        # Handles GPS trajectory visualization and camera positioning
        Node(
            package='geodetic_points',
            executable='gps_on_globe_node',
            name='gps_trajectory_manager',
            parameters=[{
                # Core parameters
                'frame_id': LaunchConfiguration('frame_id'),
                'gps_topic': LaunchConfiguration('gps_topic'),
                'scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                
                # Trajectory visualization parameters
                'max_points': LaunchConfiguration('max_points'),
                'point_radius_m': LaunchConfiguration('point_radius_m'),
                'trail_width_m': LaunchConfiguration('trail_width_m'),
                'publish_every_n': LaunchConfiguration('publish_every_n'),
                
                # Camera control parameters
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_window_size': LaunchConfiguration('camera_window_size'),
                'camera_min_update_distance': LaunchConfiguration('camera_min_update_distance'),
                'camera_yaw': LaunchConfiguration('camera_yaw'),
                'camera_pitch': LaunchConfiguration('camera_pitch'),
                'camera_roll': LaunchConfiguration('camera_roll'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                
                # Debug parameters
                'enable_debug_output': LaunchConfiguration('enable_debug_output')
            }],
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # DATA REPLAY SYSTEM
        # ------------------------------------------------------------------------
        
        # ROS Bag Replay (Conditional)
        # Only launches if bag_file parameter is provided
        ExecuteProcess(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('bag_file'), "' != ''"
            ])),
            cmd=[
                'ros2', 'bag', 'play', LaunchConfiguration('bag_file'), 
                '--rate', LaunchConfiguration('bag_rate'),
                '--clock'  # Publish simulation time
            ],
            name='bag_player',
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # VISUALIZATION FRONTEND
        # ------------------------------------------------------------------------
        
        # RViz2 3D Visualization
        # Provides interactive 3D visualization interface
        Node(
            package='rviz2',
            executable='rviz2',
            name='globe_visualizer',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])