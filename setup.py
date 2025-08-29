from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'geodetic_points'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'textures'), glob(os.path.join('textures', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Visual-Inertial Odometry (VIO) + GPS aided navigation and geodetic point localization system for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'globe_marker_node = geodetic_points.globe_marker_node:main',
            'gps_on_globe_node = geodetic_points.gps_on_globe_node:main',
        ],
    },
)