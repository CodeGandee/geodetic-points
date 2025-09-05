#!/usr/bin/env python3
"""
Test launch file for GPS-VIO calibration node.
This launch file runs only the gps_vio_calibration_node for testing purposes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.conditions import IfCondition
import os
from datetime import datetime


def generate_launch_description():
    # Get timestamp for log files
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # Get ROS_LOG_DIR from environment or use default
    log_dir = os.environ.get('ROS_LOG_DIR', './log')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # ============================================================================
    # GPS TRAJECTORY PARAMETERS
    # ============================================================================
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='/cbs_gnss',
        description='ROS topic name for GPS NavSatFix messages'
    )

    # ============================================================================
    # VIO VISUALIZATION PARAMETERS
    # ============================================================================
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/slamware_ros_sdk_server_node/odom',
        description='ROS topic name for VIO odometry messages'
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

    # Declare launch arguments
    debug_logging_arg = DeclareLaunchArgument(
        'debug_logging',
        default_value='true',  # Enable debug logging by default for testing
        description='Enable debug logging'
    )
    
    # ============================================================================
    # MONITORING AND LOGGING PARAMETERS
    # ============================================================================
    enable_topic_monitoring_arg = DeclareLaunchArgument(
        'enable_topic_monitoring',
        default_value='true',
        description='Enable monitoring and logging of calibration topics'
    )
    
    topic_log_rate_arg = DeclareLaunchArgument(
        'topic_log_rate',
        default_value='2.0',
        description='Rate (Hz) for logging topic data'
    )
    
    
    # In ROS2 (Humble) you can set ROS_LOG_DIR to control where rcl/rclcpp write log files
    set_ros_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value=log_dir
    )

    
    return LaunchDescription([
        debug_logging_arg,
        bag_file_arg,
        bag_rate_arg,
        gps_topic_arg,
        odom_topic_arg,
        enable_topic_monitoring_arg,
        topic_log_rate_arg,
        set_ros_log_dir,
        
        LogInfo(msg=f'Log directory: {log_dir}'),
        LogInfo(msg=f'Log files will be prefixed with timestamp: {timestamp}'),

        Node(
            package='geodetic_points',
            executable='gps_vio_calibration_node',
            name='gps_vio_calibration',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # Time sync parameters
                'time_sync.buffer_seconds': 60.0,
                'time_sync.max_offset_search': 5.0,
                'time_sync.smoothing_alpha': 0.05,
                'time_sync.drift_estimation_window': 50,
                'time_sync.min_motion_threshold': 0.05,
                
                # Spatial calibration parameters
                'calibration.window_size': 100,
                'calibration.min_motion_threshold': 0.05,
                'calibration.outlier_threshold': 3.0,
                'calibration.auto_publish_transform': True,
                'calibration.convergence_threshold': 0.5,
                'calibration.max_error': 10.0,
                'calibration.min_pairs': 5,
                'calibration.publish_before_convergence': True,
                
                # Frame IDs
                'frame_id_earth': 'earth',
                'frame_id_odom': 'odom',
                
                # Enhanced logging parameters
                'debug_logging': LaunchConfiguration('debug_logging'),
                'enable_file_logging': True,
                'log_interval_seconds': 5.0
            }], 
            remappings=[
                    ('/gps/fix', LaunchConfiguration('gps_topic')),
                    ('/odom', LaunchConfiguration('odom_topic'))
                ],
        ),
        
        # ============================================================================
        # TOPIC MONITORING FOR CALIBRATION QUALITY ANALYSIS
        # These processes record calibration topics to log files
        # ============================================================================
        
        # Combined calibration metrics logger (samples all topics periodically)
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('enable_topic_monitoring')),
            cmd=[
                'sh', '-c',
                f'''echo "Calibration monitoring started at $(date)" > {log_dir}/calibration_metrics_{timestamp}.log
                while true; do
                    echo "========== $(date -Iseconds) =========='" >> {log_dir}/calibration_metrics_{timestamp}.log
                    echo "Quality:" >> {log_dir}/calibration_metrics_{timestamp}.log
                    ros2 topic echo /calibration/quality std_msgs/msg/Float64 --once 2>/dev/null >> {log_dir}/calibration_metrics_{timestamp}.log || echo "No quality data" >> {log_dir}/calibration_metrics_{timestamp}.log
                    echo "Time Difference:" >> {log_dir}/calibration_metrics_{timestamp}.log
                    ros2 topic echo /sync/time_difference std_msgs/msg/Float64 --once 2>/dev/null >> {log_dir}/calibration_metrics_{timestamp}.log || echo "No time diff data" >> {log_dir}/calibration_metrics_{timestamp}.log
                    echo "Clock Drift Rate (ppm):" >> {log_dir}/calibration_metrics_{timestamp}.log
                    ros2 topic echo /sync/clock_drift_rate std_msgs/msg/Float64 --once 2>/dev/null >> {log_dir}/calibration_metrics_{timestamp}.log || echo "No drift data" >> {log_dir}/calibration_metrics_{timestamp}.log
                    echo "Residual Error:" >> {log_dir}/calibration_metrics_{timestamp}.log
                    ros2 topic echo /sync/residual_error std_msgs/msg/Float64 --once 2>/dev/null >> {log_dir}/calibration_metrics_{timestamp}.log || echo "No error data" >> {log_dir}/calibration_metrics_{timestamp}.log
                    sleep 5
                done'''
            ],
            name='metrics_logger',
            output='log'
        ),
        
        # Monitor /tf_static topic (only logs when published)
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('enable_topic_monitoring')),
            cmd=[
                'sh', '-c',
                f'ros2 topic echo /tf_static tf2_msgs/msg/TFMessage --qos-durability transient_local > {log_dir}/tf_static_{timestamp}.log 2>&1'
            ],
            name='tf_static_monitor',
            output='log'
        ),
        
        # Monitor calibration transform
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('enable_topic_monitoring')),
            cmd=[
                'sh', '-c',
                f'ros2 topic echo /calibration/transform_earth_odom geometry_msgs/msg/TransformStamped > {log_dir}/calibration_transform_{timestamp}.log 2>&1'
            ],
            name='calibration_transform_monitor',
            output='log'
        ),
        ExecuteProcess(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('bag_file'), "' != ''"
            ])),
            cmd=[
                'ros2', 'bag', 'play', LaunchConfiguration('bag_file'),
                '--rate', LaunchConfiguration('bag_rate'),
                '--clock',  # Essential for sim_time synchronization
            ],
            name='bag_player_test',
            output='screen'
        ),
    ])