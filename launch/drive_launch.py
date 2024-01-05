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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import logging

def generate_launch_description():
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'joy_launch.py'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy',
            default_value='True',
            description='Enable Joy and Teleop Twist Joy nodes'),
        
        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Joystick Configuration: ps4, stadia, sn30pro, steamdeck'),
        
        Node(
            package='akros2_base',
            executable='twist_mixer',
            name='twist_mixer',
            output='screen',
            parameters=[{'timer_period': 0.02}],
            remappings=[
                ('/teleop_vel', '/joy_vel'),
                ('/auto_vel', '/nav_vel'),
                ('/mix_vel', '/cmd_vel'),
            ]),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            condition=IfCondition(LaunchConfiguration('joy')),
            launch_arguments={'joy_config': LaunchConfiguration('joy_config')}.items()),
    ])
