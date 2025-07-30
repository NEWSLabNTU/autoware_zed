#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    enable_od_service = LaunchConfiguration('enable_od_service')
    
    # Get config override path
    config_override_path = os.path.join(
        get_package_share_directory('zed_launch'),
        'config',
        'od_override.yaml'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zedxm',
            description='Camera model (zed, zedm, zed2, zed2i, zedx, zedxm)'
        ),
        DeclareLaunchArgument(
            'enable_od_service',
            default_value='true',
            description='Enable object detection via service call after launch'
        ),
        
        # Include ZED camera launch with object detection parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_model': camera_model,
                'camera_name': camera_name,
                'ros_params_override_path': config_override_path,
            }.items()
        ),
        
        # Service call to enable object detection after camera starts
        ExecuteProcess(
            cmd=['sleep', '5'],  # Wait for camera to initialize
            condition=IfCondition(enable_od_service)
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/zed/zed_node/enable_obj_det',
                'std_srvs/srv/SetBool',
                '{data: true}'
            ],
            shell=True,
            condition=IfCondition(enable_od_service),
            output='screen'
        ),
    ])