#!/usr/bin/env python3
"""
Unified VIO-GPS Integration Launch File

This comprehensive launch file provides a complete VIO-GPS calibration and visualization system:
- Earth globe visualization with textured mesh
- GPS-VIO time synchronization and spatial calibration  
- Real-time VIO trajectory and point cloud visualization in Earth coordinates
- Camera control that follows GPS/VIO trajectory
- Optional ROS bag replay for offline analysis
- Flexible configuration for different operational modes

Usage:
  ros2 launch geodetic_points vio_gps_integration.launch.py [arguments]

Key arguments:
  scale:=10                           # Earth scale divisor (1/10 = 0.1x real size)
  enable_calibration:=true            # Enable GPS-VIO calibration
  enable_visualization:=true          # Enable real-time visualization  
  enable_rviz:=true                  # Launch RViz interface
  bag_file:=/path/to/bag.db3        # Optional bag file for replay
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ============================================================================
    # CONSTANTS AND FILE PATHS
    # ============================================================================
    pkg_share = get_package_share_directory('geodetic_points')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'globe_vio_gps.rviz')
    default_mesh_resource = 'package://geodetic_points/meshes/earth.dae'
    
    # ============================================================================
    # GLOBAL/COMMON PARAMETERS
    # ============================================================================
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='10',
        description='Scale divisor for earth globe and coordinates (1/scale applied). '
                    'Example: scale=10 means Earth will be 1/10th real size'
    )
    
    frame_id_earth_arg = DeclareLaunchArgument(
        'frame_id_earth',
        default_value='earth',
        description='Earth/ECEF coordinate frame ID'
    )
    
    frame_id_odom_arg = DeclareLaunchArgument(
        'frame_id_odom',
        default_value='odom',
        description='VIO odometry coordinate frame ID'
    )
    
    # ============================================================================
    # OPERATIONAL MODE PARAMETERS
    # ============================================================================
    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration',
        default_value='true',
        description='Enable GPS-VIO calibration (time sync and spatial alignment)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable real-time visualization of GPS and VIO data'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz for 3D visualization'
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
    
    # ============================================================================
    # GPS-VIO CALIBRATION PARAMETERS
    # ============================================================================
    # Time synchronization parameters
    time_sync_buffer_arg = DeclareLaunchArgument(
        'time_sync.buffer_seconds',
        default_value='15.0',
        description='VIO data buffer duration for time synchronization (seconds)'
    )
    
    time_sync_max_offset_arg = DeclareLaunchArgument(
        'time_sync.max_offset_search',
        default_value='2.0',
        description='Maximum time offset to search during synchronization (seconds)'
    )
    
    time_sync_smoothing_arg = DeclareLaunchArgument(
        'time_sync.smoothing_alpha',
        default_value='0.05',
        description='Time model update smoothing factor (0-1)'
    )
    
    # Spatial calibration parameters
    calibration_min_pairs_arg = DeclareLaunchArgument(
        'calibration.min_pairs',
        default_value='10',
        description='Minimum GPS-VIO pairs required for calibration'
    )
    
    calibration_max_error_arg = DeclareLaunchArgument(
        'calibration.max_error',
        default_value='10.0',
        description='Maximum acceptable calibration RMS error (meters)'
    )
    
    calibration_convergence_arg = DeclareLaunchArgument(
        'calibration.convergence_threshold',
        default_value='0.01',
        description='Calibration convergence threshold (meters)'
    )
    
    calibration_auto_publish_arg = DeclareLaunchArgument(
        'calibration.auto_publish_transform',
        default_value='true',
        description='Automatically publish earth->odom transform when converged'
    )
    
    # ============================================================================
    # GPS VISUALIZATION PARAMETERS
    # ============================================================================
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='/gps/fix',
        description='ROS topic name for GPS NavSatFix messages'
    )
    
    max_gps_points_arg = DeclareLaunchArgument(
        'max_gps_points',
        default_value='4000',
        description='Maximum number of GPS points to display in trajectory'
    )
    
    gps_point_radius_arg = DeclareLaunchArgument(
        'gps_point_radius_m',
        default_value='10.0',
        description='Base radius of GPS point markers in meters'
    )
    
    gps_trail_width_arg = DeclareLaunchArgument(
        'gps_trail_width_m',
        default_value='5.0',
        description='Base width of GPS trail line in meters'
    )
    
    # ============================================================================
    # VIO VISUALIZATION PARAMETERS
    # ============================================================================
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='ROS topic name for VIO odometry messages'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/vio/points3d',
        description='ROS topic name for VIO point cloud messages'
    )
    
    max_vio_points_arg = DeclareLaunchArgument(
        'max_vio_points',
        default_value='5000',
        description='Maximum number of VIO trajectory points to display'
    )
    
    vio_point_radius_arg = DeclareLaunchArgument(
        'vio_point_radius',
        default_value='3.0',
        description='VIO trajectory point marker radius (meters)'
    )
    
    vio_arrow_length_arg = DeclareLaunchArgument(
        'vio_arrow_length',
        default_value='10.0',
        description='VIO orientation arrow length (meters)'
    )
    
    cloud_downsample_arg = DeclareLaunchArgument(
        'cloud_downsample_factor',
        default_value='10',
        description='Point cloud downsampling factor for visualization'
    )
    
    # ============================================================================
    # CAMERA CONTROL PARAMETERS
    # ============================================================================
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='2620.0',
        description='Camera height above trajectory center in meters'
    )
    
    camera_window_size_arg = DeclareLaunchArgument(
        'camera_window_size',
        default_value='10',
        description='Number of recent points used to calculate camera center position'
    )
    
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw',
        default_value='2.0454',
        description='Camera yaw angle in radians'
    )
    
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch',
        default_value='1.4304',
        description='Camera pitch angle in radians'
    )
    
    # ============================================================================
    # DATA REPLAY PARAMETERS
    # ============================================================================
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837',
        description='Path to ROS bag file for replay (leave empty for live data)'
    )
    
    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='10.0',
        description='Bag replay rate multiplier'
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
        frame_id_earth_arg,
        frame_id_odom_arg,
        
        # Operational modes
        enable_calibration_arg,
        enable_visualization_arg,
        enable_rviz_arg,
        
        # Earth globe parameters  
        mesh_resource_arg,
        publish_once_arg,
        
        # GPS-VIO calibration parameters
        time_sync_buffer_arg,
        time_sync_max_offset_arg,
        time_sync_smoothing_arg,
        calibration_min_pairs_arg,
        calibration_max_error_arg,
        calibration_convergence_arg,
        calibration_auto_publish_arg,
        
        # GPS visualization parameters
        gps_topic_arg,
        max_gps_points_arg,
        gps_point_radius_arg,
        gps_trail_width_arg,
        
        # VIO visualization parameters
        odom_topic_arg,
        pointcloud_topic_arg,
        max_vio_points_arg,
        vio_point_radius_arg,
        vio_arrow_length_arg,
        cloud_downsample_arg,
        
        # Camera control parameters
        camera_height_arg,
        camera_window_size_arg,
        camera_yaw_arg,
        camera_pitch_arg,
        
        # Data replay parameters
        bag_file_arg,
        bag_rate_arg,
        
        # ------------------------------------------------------------------------
        # CORE VISUALIZATION NODES
        # ------------------------------------------------------------------------
        
        # Earth Globe Visualization Node
        # Always launched to provide the Earth reference
        Node(
            package='geodetic_points',
            executable='globe_marker_node',
            name='earth_globe_visualizer',
            parameters=[{
                'use_sim_time': True,
                'mesh_resource': LaunchConfiguration('mesh_resource'),
                'scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                'frame_id': LaunchConfiguration('frame_id_earth'),
                'publish_once': LaunchConfiguration('publish_once'),
                'publish_frequency': 0.5
            }],
            output='screen'
        ),
        
        # GPS Trajectory & Camera Control Node
        # Always launched to visualize GPS data
        Node(
            package='geodetic_points',
            executable='gps_on_globe_node',
            name='gps_trajectory_manager',
            parameters=[{
                'use_sim_time': True,
                # Core parameters
                'frame_id': LaunchConfiguration('frame_id_earth'),
                'gps_topic': LaunchConfiguration('gps_topic'),
                'scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                
                # Trajectory visualization parameters
                'max_points': LaunchConfiguration('max_gps_points'),
                'point_radius_m': LaunchConfiguration('gps_point_radius_m'),
                'trail_width_m': LaunchConfiguration('gps_trail_width_m'),
                'publish_every_n': 1,
                
                # Camera control parameters
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_window_size': LaunchConfiguration('camera_window_size'),
                'camera_min_update_distance': 5.0,
                'camera_yaw': LaunchConfiguration('camera_yaw'),
                'camera_pitch': LaunchConfiguration('camera_pitch'),
                'camera_roll': 0.0,
                'camera_frame_id': 'camera_view',
                
                # Debug parameters
                'enable_debug_output': False
            }],
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # GPS-VIO CALIBRATION SYSTEM
        # ------------------------------------------------------------------------
        
        # GPS-VIO Calibration Node (Conditional)
        # Handles time synchronization and spatial calibration
        Node(
            package='geodetic_points',
            executable='gps_vio_calibration_node',
            name='gps_vio_calibration',
            condition=IfCondition(LaunchConfiguration('enable_calibration')),
            remappings=[
                ('gps/fix', LaunchConfiguration('gps_topic')),
                ('odom', LaunchConfiguration('odom_topic'))
            ],
            parameters=[{
                'use_sim_time': True,
                # Time sync parameters
                'time_sync.buffer_seconds': LaunchConfiguration('time_sync.buffer_seconds'),
                'time_sync.max_offset_search': LaunchConfiguration('time_sync.max_offset_search'),
                'time_sync.smoothing_alpha': LaunchConfiguration('time_sync.smoothing_alpha'),
                'time_sync.drift_estimation_window': 50,
                'time_sync.min_motion_threshold': 0.1,
                
                # Calibration parameters
                'calibration.window_size': 100,
                'calibration.min_motion_threshold': 0.2,
                'calibration.outlier_threshold': 3.0,
                'calibration.auto_publish_transform': LaunchConfiguration('calibration.auto_publish_transform'),
                'calibration.convergence_threshold': LaunchConfiguration('calibration.convergence_threshold'),
                'calibration.max_error': LaunchConfiguration('calibration.max_error'),
                'calibration.min_pairs': LaunchConfiguration('calibration.min_pairs'),
                
                # Frame IDs
                'frame_id_earth': LaunchConfiguration('frame_id_earth'),
                'frame_id_odom': LaunchConfiguration('frame_id_odom')
            }],
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # VIO-EARTH VISUALIZATION SYSTEM
        # ------------------------------------------------------------------------
        
        # VIO-Earth Visualization Node (Conditional)
        # Transforms and visualizes VIO data in Earth coordinates
        Node(
            package='geodetic_points',
            executable='vio_earth_visualization_node',
            name='vio_earth_visualization',
            condition=IfCondition(LaunchConfiguration('enable_visualization')),
            remappings=[
                ('odom', LaunchConfiguration('odom_topic')),
                ('vio/points3d', LaunchConfiguration('pointcloud_topic'))
            ],
            parameters=[{
                'use_sim_time': True,
                # Frame IDs
                'frame_id_earth': LaunchConfiguration('frame_id_earth'),
                'frame_id_odom': LaunchConfiguration('frame_id_odom'),
                'frame_id_base_link': 'base_link',
                
                # Transform parameters
                'tf_timeout_ms': 100,
                'use_tf_for_transform': True,
                
                # Visualization parameters
                'visualization_scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
                'max_vio_points': LaunchConfiguration('max_vio_points'),
                'max_cloud_points': 50000,
                
                # VIO visualization
                'vio.point_radius': LaunchConfiguration('vio_point_radius'),
                'vio.trail_width': 2.0,
                'vio.arrow_length': LaunchConfiguration('vio_arrow_length'),
                'vio.arrow_width': 0.5,
                'vio.color': [0.1, 1.0, 0.1, 0.9],  # Green
                
                # Point cloud visualization
                'cloud.point_size': 1.0,
                'cloud.color': [0.1, 0.1, 1.0, 0.7],  # Blue
                'cloud.downsample_factor': LaunchConfiguration('cloud_downsample_factor'),
                'cloud.adaptive_sizing': True,
                'cloud.max_display_distance': 1000.0,
                
                # Performance parameters
                'visualization_rate_hz': 10.0,
                'point_cloud_rate_hz': 2.0
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
                '--clock',  # Publish simulation time
                # Topic remappings for common bag formats
                '--remap', '/cbs_gnss:=/gps/fix', '/slamware_ros_sdk_server_node/odom:=/odom','/slamware_ros_sdk_server_node/points3d:=/vio/points3d'
            ],
            name='bag_player',
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # VISUALIZATION FRONTEND
        # ------------------------------------------------------------------------
        
        # RViz2 3D Visualization (Conditional)
        # Provides interactive 3D visualization interface
        Node(
            package='rviz2',
            executable='rviz2',
            name='integrated_visualizer',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])