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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_filter_config_dynamic_path = [get_package_share_directory('akros2_base'),
                                      '/config/akros2_',
                                      LaunchConfiguration('config'),
                                      '/imu_filter_config.yaml']

    ekf_config_dynamic_path = [get_package_share_directory('akros2_base'),
                               '/config/akros2_',
                               LaunchConfiguration('config'),
                               '/ekf_config.yaml']
    
    motion_detector_config_dynamic_path = [get_package_share_directory('akros2_base'),
                                           '/config/akros2_',
                                           LaunchConfiguration('config'),
                                           '/motion_detector_config.yaml']

    return LaunchDescription([
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled), diff (2 wheeled)'),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_filter_config_dynamic_path],
            remappings=[
                ('/imu/data_raw', '/imu'),
                ('/imu/data', '/imu/filtered')]),
        
        Node(
            package='akros2_base',
            executable='motion_detector',
            name='motion_detector',
            output='screen',
            parameters=[motion_detector_config_dynamic_path],
            remappings=[('/imu', '/imu/filtered')]),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_dynamic_path]),
    ])
