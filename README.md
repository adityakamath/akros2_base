# akros2_base
ROS2 package to run AKROS2 base nodes with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled). This package includes packages related to drivers, sensors, filters and fusion.

## drive_launch.py
This launch file runs the ```twist_mixer``` executable by itself. Parameter and Topic remappings are done in the launch file and there are no launch arguments.

## teleop_launch.py
This launch files runs the ```joy``` and ```teleop_twist_joy``` nodes using [joy_launch.py](https://github.com/adityakamath/akros2_drive/blob/humble/launch/joy_launch.py), and the ```joy_mode_handler``` node, both from [akros2_drive](https://github.com/adityakamath/akros2_drive), and uses the ```joy_config``` launch argument to configure the controller. Parameters are set using config files in akros2_drive.

### sensor_fusion_launch.py
This is the launch file for running sensor fusion related nodes. Currently, this implements the [Madgwick Filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick) for filtering IMU measurements and the [Extended Kalman Filter](https://ahrs.readthedocs.io/en/latest/filters/ekf.html) from [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html) to fuse the [filtered IMU measurements and Wheel Odometry messages](https://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html) from the robot. This launch file subscribes to raw [IMU](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) (```/imu```), [Magnetometer](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) if available (```/mag```) and wheel [Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) (```/odometry```) messages, and publishes filtered IMU (```/imu/filtered```) and Odometry (```/odometry/filtered```) messages, alongside the ```odom->base_footprint``` transform. The ```config``` launch argument can be used for selecting the robot platform: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) with the config files available in the [config](https://github.com/adityakamath/akros2_navigation/tree/humble/config) directory in the corresponding folders (Default: ```mecanum```).

### laser_launch.py
This launch file runs the [ldlidar](https://github.com/linorobot/ldlidar) and [laser_filters](https://github.com/ros-perception/laser_filters) packages. It has only one launch argument:

* ```laser_filter```: Enable/Disable the LIDAR filter chain (Default: ```True```)

### camera_launch.py
This launch file runs the [v4l2_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera) packages. It has two launch arguments:

* ```config```: Used to select cameras from the different configurations: ```mecanum```, ```omni``` (Default: ```mecanum```)
* ```compose```: If True, launches the camera node as a composable container. If False, runs the node normally. The normal method also publishes compressed image data. However, with the composable container, the compressed image plugins are not used by default (WIP) (Default: ```False```)

### control_launch.py
Launches the [micro-ROS agent](https://github.com/micro-ROS/micro-ROS-Agent) with the correct arguments based on the value of ```config```e:

* ```config```: Launches specific low-level control nodes based on the configs: ```mecanum```, ```omni``` (Default: ```mecanum```)

## Note
Due to issues with launching everything together, it is recommended to launch ```camera_launch.py``` and ```control_launch.py``` on separate terminals and not from the main launch file: ```bringup_launch.py``` in [akros2_bringup](). Run ```bringup_launch.py``` with ```control:=false camera:=false```.
