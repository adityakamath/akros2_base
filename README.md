# akros2_base
ROS2 package to run AKROS2 base nodes with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled). This package includes packages related to sensor drivers, filtering and fusion for localization.

## sensor_fusion_launch.py
This is the launch file for running sensor fusion related nodes. Currently, this implements the [Madgwick Filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick) for filtering IMU measurements and the [Extended Kalman Filter](https://ahrs.readthedocs.io/en/latest/filters/ekf.html) from [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html) to fuse the [filtered IMU measurements and Wheel Odometry messages](https://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html) from the robot. This launch file subscribes to raw [IMU](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) (```/imu```), [Magnetometer](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) if available (```/mag```) and wheel [Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) (```/odometry```) messages, and publishes filtered IMU (```/imu/filtered```) and Odometry (```/odometry/filtered```) messages, alongside the ```odom->base_footprint``` transform. The ```config``` launch argument can be used for selecting the robot platform: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) with the config files available in the [config](https://github.com/adityakamath/akros2_navigation/tree/humble/config) directory in the corresponding folders (Default: ```mecanum```). 

## laser_launch.py
This launch file runs the [ldlidar](https://github.com/linorobot/ldlidar) and [laser_filters](https://github.com/ros-perception/laser_filters) packages. It has only one launch argument:

* ```laser_filter```: Enable/Disable the LIDAR filter chain (Default: ```True```)

## camera_launch.py
This launch file runs the [v4l2_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera) packages. It has two launch arguments:

* ```ns```: Namespace of the system (Default: ```''```)
* ```config```: Select robot platform. Currently, the camera is only used on the mecanum platform (Default: ```mecanum```)
