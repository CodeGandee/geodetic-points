#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('geodetic_points')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'globe_viz.rviz')
    
    # Declare launch arguments
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='6371000.0',
        description='Scale of the earth globe (radius in meters)'
    )
    
    mesh_resource_arg = DeclareLaunchArgument(
        'mesh_resource',
        default_value='package://geodetic_points/meshes/earth.dae',
        description='Path to the earth mesh resource file'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='earth',
        description='Frame ID for the earth globe'
    )
    
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='gps/fix',
        description='GPS topic name'
    )
    
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='true',
        description='Publish globe marker only once (true/false)'
    )
    
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='0.5',
        description='Globe marker publish frequency in Hz (only used if publish_once=false)'
    )
    
    return LaunchDescription([
        # Launch arguments
        scale_arg,
        mesh_resource_arg,
        frame_id_arg,
        gps_topic_arg,
        publish_once_arg,
        publish_frequency_arg,
        
        # Textured globe mesh
        Node(
            package='geodetic_points',
            executable='globe_marker_node',
            parameters=[{
                'mesh_resource': LaunchConfiguration('mesh_resource'),
                'scale': LaunchConfiguration('scale'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_once': LaunchConfiguration('publish_once'),
                'publish_frequency': LaunchConfiguration('publish_frequency')
            }],
            output='screen'
        ),
        # Live GPS overlay (points + trail) in ECEF/earth frame
        Node(
            package='geodetic_points',
            executable='gps_on_globe_node',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'gps_topic': LaunchConfiguration('gps_topic'),
                'max_points': 4000,
                'point_radius_m': 50000.0,
                'trail_width_m': 80000.0,
                'publish_every_n': 1
            }],
            output='screen'
        ),
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])