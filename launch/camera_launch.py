# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_config_dynamic_path = [get_package_share_directory('akros2_base'), 
                                      '/config/akros2_', 
                                      LaunchConfiguration('config'), 
                                      '/camera_config.yaml']
    
    camera_info_dynamic_path = [get_package_share_directory('akros2_base'), 
                                      '/config/akros2_', 
                                      LaunchConfiguration('config'), 
                                      '/camera_info.yaml']
    
    return LaunchDescription([   
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled)'),

        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('config', 'mecanum'),
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='v4l2_camera',
                    plugin='v4l2_camera::V4L2Camera',
                    name='camera_node',
                    namespace='',
                    parameters=[{'camera_info_url': camera_info_dynamic_path},
                                camera_config_dynamic_path],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
    ])

