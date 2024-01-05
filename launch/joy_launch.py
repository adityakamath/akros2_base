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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import logging

def generate_launch_description():
    joy_twist_config_dynamic_path = [get_package_share_directory('akros2_base'), 
                                     '/config/joy/', 
                                     LaunchConfiguration('joy_config'), 
                                     '_twist_config.yaml']
    
    joy_mode_config_dynamic_path = [get_package_share_directory('akros2_base'), 
                                    '/config/joy/', 
                                    LaunchConfiguration('joy_config'), 
                                    '_mode_config.yaml']
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Joystick Configuration: ps4, stadia, sn30pro, steamdeck'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
                'coalesce_interval': 0.01,
            }],
            arguments=["--ros-args", "--log-level", "ERROR"]),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_teleop',
            parameters=[joy_twist_config_dynamic_path],
            remappings=[
                ('/cmd_vel', '/joy_vel')
            ]),
        
        Node(
            package='akros2_base',
            executable='joy_mode_handler',
            name='joy_mode_handler',
            output='screen',
            parameters=[joy_mode_config_dynamic_path]),
    ])
