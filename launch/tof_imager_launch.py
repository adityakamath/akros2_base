from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tof_imager_launch_path = PathJoinSubstitution(
        [FindPackageShare('tof_imager_ros'), 'launch', 'tof_imager_launch.py'])
    
    tof_imager_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_base"), "config", "tof_imager_config.yaml"])


    return LaunchDescription([       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensehat_launch_path),
            launch_arguments={'frame_id': 'tof_link',
                              'config_path': tof_imager_config_path}.items()),

    ])