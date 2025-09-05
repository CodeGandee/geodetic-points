#!/usr/bin/env python3
"""
GPS-VIO Calibration Test Launch File

This launch file is specifically designed to test the gps_vio_calibration_node.py
functionality. It launches all the necessary supporting nodes while focusing on
the calibration node's performance, including:

- Earth globe visualization for reference
- GPS trajectory visualization  
- VIO-Earth visualization for transformed data
- ROS bag replay for testing data
- RViz for monitoring

The launch file excludes the gps_vio_calibration_node initially to establish
a baseline, then can be enabled to test calibration performance.

Key Features:
- Focused testing of GPS-VIO calibration node
- Comprehensive monitoring of published topics and transforms
- Time synchronization validation
- Transform quality assessment
- Compatible with existing build.sh and run.sh scripts

Usage:
  ros2 launch geodetic_points gps_vio_calibration_test.launch.py [arguments]

Key arguments:
  enable_calibration:=true           # Enable/disable calibration node for testing
  monitor_topics:=true              # Enable topic monitoring and debugging
  bag_file:=/path/to/test.db3      # Test data bag file
  calibration_timeout:=30.0         # Maximum calibration time for testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def set_log_level(context):
    """Function to set log level based on enable_debug_logging parameter."""
    enable_debug = context.launch_configurations.get('enable_debug_logging', 'true')
    if enable_debug.lower() == 'true':
        # Set to INFO level for detailed logging
        os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time}] [{name}]: {message}'
        # os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'
        return [SetEnvironmentVariable('RCUTILS_LOG_MIN_SEVERITY', 'INFO')]
    else:
        # Set to WARN level for minimal logging
        return [SetEnvironmentVariable('RCUTILS_LOG_MIN_SEVERITY', 'WARN')]

def generate_launch_description():
    # ============================================================================
    # CONSTANTS AND FILE PATHS
    # ============================================================================
    pkg_share = get_package_share_directory('geodetic_points')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'globe_vio_gps.rviz')
    default_mesh_resource = 'package://geodetic_points/meshes/earth.dae'

    # Get ROS_LOG_DIR from environment or use default
    log_dir = os.environ.get('ROS_LOG_DIR', './log')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # ============================================================================
    # TEST CONFIGURATION PARAMETERS
    # ============================================================================
    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration',
        default_value='true',
        description='Enable GPS-VIO calibration node for testing'
    )
    
    enable_debug_logging_arg = DeclareLaunchArgument(
        'enable_debug_logging',
        default_value='true',
        description='Enable debug logging for all nodes (true: all INFO logs, false: only WARNING and above)'
    )
    
    enable_topic_monitoring_arg = DeclareLaunchArgument(
        'enable_topic_monitoring',
        default_value='true',
        description='Enable continuous monitoring of calibration topics to log file'
    )
    
    calibration_timeout_arg = DeclareLaunchArgument(
        'calibration_timeout',
        default_value='30.0',
        description='Maximum time to wait for calibration convergence (seconds)'
    )
    
    # ============================================================================
    # GLOBAL/COMMON PARAMETERS (same as globe_viz.launch.py)
    # ============================================================================
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='10',
        description='Scale divisor for earth globe and GPS coordinates'
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
    # EARTH GLOBE VISUALIZATION PARAMETERS
    # ============================================================================
    mesh_resource_arg = DeclareLaunchArgument(
        'mesh_resource',
        default_value=default_mesh_resource,
        description='Path to the earth mesh resource file'
    )
    
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='true',
        description='Publish globe marker only once'
    )
    
    # ============================================================================
    # GPS-VIO CALIBRATION TEST PARAMETERS
    # ============================================================================
    # Time synchronization parameters for testing
    time_sync_buffer_arg = DeclareLaunchArgument(
        'time_sync.buffer_seconds',
        default_value='15.0',
        description='VIO data buffer duration for time synchronization'
    )
    
    time_sync_max_offset_arg = DeclareLaunchArgument(
        'time_sync.max_offset_search',
        default_value='2.0',
        description='Maximum time offset to search during synchronization'
    )
    
    time_sync_smoothing_arg = DeclareLaunchArgument(
        'time_sync.smoothing_alpha',
        default_value='0.05',
        description='Time model update smoothing factor'
    )
    
    # Spatial calibration parameters for testing
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
        default_value='0.5',
        description='Calibration convergence threshold (meters) - relaxed for testing'
    )
    
    calibration_auto_publish_arg = DeclareLaunchArgument(
        'calibration.auto_publish_transform',
        default_value='true',
        description='Automatically publish earth->odom transform when converged'
    )
    
    # ============================================================================
    # GPS TRAJECTORY PARAMETERS
    # ============================================================================
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='/cbs_gnss',
        description='ROS topic name for GPS NavSatFix messages'
    )
    
    max_gps_points_arg = DeclareLaunchArgument(
        'max_gps_points',
        default_value='4000',
        description='Maximum number of GPS points to display'
    )
    
    gps_point_radius_arg = DeclareLaunchArgument(
        'gps_point_radius_m',
        default_value='10.0',
        description='GPS point marker radius in meters'
    )
    
    gps_trail_width_arg = DeclareLaunchArgument(
        'gps_trail_width_m',
        default_value='5.0',
        description='GPS trail line width in meters'
    )
    
    # ============================================================================
    # VIO VISUALIZATION PARAMETERS
    # ============================================================================
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/slamware_ros_sdk_server_node/odom',
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
        description='VIO trajectory point marker radius'
    )
    
    # ============================================================================
    # CAMERA CONTROL PARAMETERS
    # ============================================================================
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='2620.0',
        description='Camera height above trajectory center'
    )
    
    camera_window_size_arg = DeclareLaunchArgument(
        'camera_window_size',
        default_value='10',
        description='Number of recent points for camera positioning'
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
        description='Path to ROS bag file for testing (leave empty for live data)'
    )
    
    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='10.0',
        description='Bag replay rate multiplier for testing'
    )

    # In ROS2 (Humble) you can set ROS_LOG_DIR to control where rcl/rclcpp write log files
    set_ros_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value=log_dir
    )
    
    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================
    return LaunchDescription([
        # ------------------------------------------------------------------------
        # PARAMETER DECLARATIONS
        # ------------------------------------------------------------------------
        # Test configuration
        enable_calibration_arg,
        enable_debug_logging_arg,
        enable_topic_monitoring_arg,
        calibration_timeout_arg,
        
        # Set log level based on enable_debug_logging parameter
        OpaqueFunction(function=set_log_level),
        
        # Global parameters
        scale_arg,
        frame_id_earth_arg,
        frame_id_odom_arg,
        
        # Earth globe parameters  
        mesh_resource_arg,
        publish_once_arg,
        
        # GPS-VIO calibration test parameters
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
        
        # Camera control parameters
        camera_height_arg,
        camera_window_size_arg,
        camera_yaw_arg,
        camera_pitch_arg,
        
        # Data replay parameters
        bag_file_arg,
        bag_rate_arg,
        
        # ------------------------------------------------------------------------
        # SUPPORTING VISUALIZATION NODES (from globe_viz.launch.py)
        # ------------------------------------------------------------------------
        set_ros_log_dir,
        # Earth Globe Visualization Node
        # Provides the Earth reference for visualization
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
        # Visualizes GPS trajectory and manages camera
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
                
                # Enhanced debug output for testing
                'debug_logging': LaunchConfiguration('enable_debug_logging')
            }],
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # GPS-VIO CALIBRATION NODE (TEST TARGET)
        # ------------------------------------------------------------------------
        
        # GPS-VIO Calibration Node - THE MAIN NODE UNDER TEST
        # This is the primary focus of this launch file
        Node(
            package='geodetic_points',
            executable='gps_vio_calibration_node',
            name='gps_vio_calibration_test',
            condition=IfCondition(LaunchConfiguration('enable_calibration')),
            remappings=[
                ('/gps/fix', LaunchConfiguration('gps_topic')),
                ('/odom', LaunchConfiguration('odom_topic'))
            ],
            parameters=[{
                'use_sim_time': True,
                # Time sync parameters - optimized for testing
                'time_sync.buffer_seconds': LaunchConfiguration('time_sync.buffer_seconds'),
                'time_sync.max_offset_search': LaunchConfiguration('time_sync.max_offset_search'),
                'time_sync.smoothing_alpha': LaunchConfiguration('time_sync.smoothing_alpha'),
                'time_sync.drift_estimation_window': 50,
                'time_sync.min_motion_threshold': 0.1,
                
                # Calibration parameters - optimized for testing
                'calibration.window_size': 100,
                'calibration.min_motion_threshold': 0.2,
                'calibration.outlier_threshold': 3.0,
                'calibration.auto_publish_transform': LaunchConfiguration('calibration.auto_publish_transform'),
                'calibration.convergence_threshold': LaunchConfiguration('calibration.convergence_threshold'),
                'calibration.max_error': LaunchConfiguration('calibration.max_error'),
                'calibration.min_pairs': LaunchConfiguration('calibration.min_pairs'),
                
                # Frame IDs
                'frame_id_earth': LaunchConfiguration('frame_id_earth'),
                'frame_id_odom': LaunchConfiguration('frame_id_odom'),
                
                # Debug logging
                'debug_logging': LaunchConfiguration('enable_debug_logging')
            }],
            output='screen'
        ),
        
        # ------------------------------------------------------------------------
        # VIO-EARTH VISUALIZATION NODE
        # ------------------------------------------------------------------------
        
        # VIO-Earth Visualization Node
        # Shows VIO data transformed to Earth coordinates using calibration
        # Node(
        #     package='geodetic_points',
        #     executable='vio_earth_visualization_node',
        #     name='vio_earth_visualization_test',
        #     remappings=[
        #         ('/odom', LaunchConfiguration('odom_topic')),
        #         ('/vio/points3d', LaunchConfiguration('pointcloud_topic'))
        #     ],
        #     parameters=[{
        #         'use_sim_time': True,
        #         # Frame IDs
        #         'frame_id_earth': LaunchConfiguration('frame_id_earth'),
        #         'frame_id_odom': LaunchConfiguration('frame_id_odom'),
        #         'frame_id_base_link': 'base_link',
                
        #         # Debug logging - enable for detailed diagnostics
        #         'debug_logging': LaunchConfiguration('enable_debug_logging'),
                
        #         # Transform parameters - extended timeout for testing
        #         'tf_timeout_ms': 1000,  # Increased to 1s as per doc recommendations
        #         'use_tf_for_transform': True,
                
        #         # Visualization parameters
        #         'visualization_scale': PythonExpression(['1.0 / ', LaunchConfiguration('scale')]),
        #         'max_vio_points': LaunchConfiguration('max_vio_points'),
        #         'max_cloud_points': 50000,
                
        #         # VIO visualization - more visible for testing
        #         'vio.point_radius': LaunchConfiguration('vio_point_radius'),
        #         'vio.trail_width': 2.0,
        #         'vio.arrow_length': 10.0,
        #         'vio.arrow_width': 0.5,
        #         'vio.color': [0.1, 1.0, 0.1, 0.9],  # Bright green for visibility
                
        #         # Point cloud visualization
        #         'cloud.point_size': 1.0,
        #         'cloud.color': [0.1, 0.1, 1.0, 0.7],  # Blue
        #         'cloud.downsample_factor': 10,
        #         'cloud.adaptive_sizing': True,
        #         'cloud.max_display_distance': 1000.0,
                
        #         # Performance parameters - optimized for testing
        #         'visualization_rate_hz': 5.0,  # Reduced rate for testing stability
        #         'point_cloud_rate_hz': 1.0
        #     }],
        #     output='screen'
        # ),
        
        # ------------------------------------------------------------------------
        # DATA REPLAY SYSTEM
        # ------------------------------------------------------------------------
        
        # ROS Bag Replay (Conditional)
        # Provides test data for calibration validation
        ExecuteProcess(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('bag_file'), "' != ''"
            ])),
            cmd=[
                'ros2', 'bag', 'play', LaunchConfiguration('bag_file'),
                '--rate', LaunchConfiguration('bag_rate'),
                '--clock'
            ],
            name='bag_player_test',
            output='log'
        ),
        
        # ------------------------------------------------------------------------
        # MONITORING AND DEBUGGING TOOLS
        # ------------------------------------------------------------------------
        
        # Topic Monitor Node
        # Monitors key calibration topics and saves to CSV and log files
        # Node(
        #     package='geodetic_points',
        #     executable='topic_monitor_node',
        #     name='topic_monitor',
        #     condition=IfCondition(LaunchConfiguration('enable_topic_monitoring')),
        #     parameters=[{
        #         'use_sim_time': True,
        #         'log_dir': log_dir,
        #         'topics_to_monitor': [
        #             '/calibration/transform_earth_odom',
        #             '/tf_static',
        #             '/vio/pose_earth'
        #         ],
        #         'monitor_rate_hz': 10.0,
        #         'buffer_size': 100,
        #         'auto_save_interval': 10.0
        #     }],
        #     output='screen'
        # ),
        
        # ------------------------------------------------------------------------
        # VISUALIZATION FRONTEND
        # ------------------------------------------------------------------------
        
        # RViz2 3D Visualization
        # Provides visual monitoring of the calibration process
        Node(
            package='rviz2',
            executable='rviz2',
            name='calibration_test_visualizer',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])