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
            name='ns',
            default_value='',
            description='Namespace of the system'),
        
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled)'),

        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('config', 'mecanum'),
            name='camera_container',
            namespace=LaunchConfiguration('ns'),
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='v4l2_camera',
                    plugin='v4l2_camera::V4L2Camera',
                    name='camera_node',
                    namespace=LaunchConfiguration('ns'),
                    parameters=[{'camera_info_url': camera_info_dynamic_path},
                                camera_config_dynamic_path],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
    ])

