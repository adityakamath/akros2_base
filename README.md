# akros2_base
ROS2 package to run AKROS2 base nodes with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled) and Differential Drive (2 wheeled). This package includes packages related to drivers, sensors, filters and fusion.

## twist_mixer_launch.py
This launch file runs the ```twist_mixer``` executable by itself. Parameter and Topic remappings are done in the launch file and there are no launch arguments.

## teleop_launch.py
This launch files runs all teleop related nodes and launch files - the ```joy``` and ```teleop_twist_joy``` nodes using [joy_launch.py](https://github.com/adityakamath/akros2_teleop/blob/humble/launch/joy_launch.py), and the ```joy_mode_handler``` node, all from [akros2_teleop](https://github.com/adityakamath/akros2_teleop). The ```joy_config``` launch argument is used to configure the controller. Parameters are set using config files in akros2_teleop.

## sensor_fusion_launch.py
This is the launch file for running sensor fusion related nodes. Currently, this implements the [Madgwick Filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick) for filtering IMU measurements and the [Extended Kalman Filter](https://ahrs.readthedocs.io/en/latest/filters/ekf.html) from [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html) to fuse the [filtered IMU measurements and Wheel Odometry messages](https://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html) from the robot. This launch file subscribes to raw [IMU](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) (```/imu```), [Magnetometer](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) if available (```/mag```) and wheel [Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) (```/odometry```) messages, and publishes filtered IMU (```/imu/filtered```) and Odometry (```/odometry/filtered```) messages, alongside the ```odom->base_footprint``` transform. Additionally, this launch file also launches the [motion_detector](https://github.com/adityakamath/akros2_teleop/blob/humble/akros2_teleop/motion_detector.py) node which uses the angular velocities from the filtered IMU messages to detect for any motion based on thresholds defined in the ```config``` directory. If motion is detected, the published [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html) message (```/in_motion```) is set to ```True```.

The ```config``` launch argument can be used for selecting the robot platform: ```mecanum``` (4-wheeled), ```omni``` (3-wheeled) or ```diff``` (2-wheeled) with the config files available in the [config](https://github.com/adityakamath/akros2_navigation/tree/humble/config) directory in the corresponding folders (Default: ```mecanum```).

## laser_launch.py
This launch file runs the [ldlidar](https://github.com/linorobot/ldlidar) and [laser_filters](https://github.com/ros-perception/laser_filters) packages. It has only one launch argument:

* ```laser_filter```: Enable/Disable the LIDAR filter chain (Default: ```True```)

## camera_launch.py
This launch file runs the [v4l2_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera) packages. It has two launch arguments:

* ```config```: Used to select cameras from the different configurations: ```mecanum```, ```omni```, ```diff``` (Default: ```mecanum```)
* ```compose```: If True, launches the camera node as a composable container. If False, runs the node normally. The normal method also publishes compressed image data. However, with the composable container, the compressed image plugins are not used by default (WIP) (Default: ```False```)

## control_launch.py
Launches the [micro-ROS agent](https://github.com/micro-ROS/micro-ROS-Agent) with the correct arguments based on the value of ```config```:

* ```config```: Launches specific low-level control nodes based on the configs: ```mecanum```, ```omni```, ```diff``` (Default: ```mecanum```)

## tof_imager_launch.py
Launches the [tof_imager_ros](https://github.com/adityakamath/tof_imager_ros) package using the config file in ```config/akros2_diff/tof_imager_config.yaml``` as this sensor is currently only used on the differential drive robot. There are no arguments in this launch file.
