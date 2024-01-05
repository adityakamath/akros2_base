# akros2_base
ROS2 package to run AKROS2 base nodes with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled). This package includes packages related to drivers, sensor filtering and fusion for localization.

## Nodes
All the nodes from the [akros2_drive](https://github.com/adityakamath/akros2_drive) repo were moved to this repository, as those nodes are also a part of the robot base and there wasn't a need for a separate repository. akros2_drive is now archived. This section explains these nodes: 

* Uses the [joy](https://github.com/adityakamath/joystick_drivers/tree/ros2/joy) node with the [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) package to publish the status of the controller's buttons/joysticks, and Twist messages based on user-defined configuration in the config/joy directory. The configuration currently supports [PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), [Stadia](https://stadia.google.com/controller/), [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) and [Steam Deck](https://store.steampowered.com/steamdeck) controllers. Similarly, mappings and mode/twist config files for other controllers can also be made. Then, the ```joy_config``` launch argument needs to be updated accordingly.
* Implements the ```joy_mode_handler``` node, which subscribes to the joystick status, and publishes [Mode](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Mode.msg) messages to indicate ```stop```, ```auto```, or ```teleop```. Uses parameters `estop_button` and `auto_button`, which represent the mapping for the E-Stop and Auto/Teleop buttons for the joystick that is used. Joystick mappings and reference parameter values can be found in the ```config``` directory (```_mode_config.yaml```)
* Implements the ```twist_mixer``` node that subscribes to the Mode messages from ```ds4_feedback```, the Twist messages from ```ds4_twist``` and Twist messages from an external autonomous node. Based on the mode, it then publishes either the Twist message from the PS4 controller, or the autonomous node, or zero values if the emergency stop is pressed. The parameter `timer_period` can be updated in the launch file (Default: 0.01 seconds).
* Both the ```joy_mode_handler``` and ```twist_mixer``` nodes are composed into a single ```drive_node``` (multi-threaded) executable that is then launched from the launch file. Both nodes can also be run individually.

## Launch Files
The following launch files are provided in this package:

### drive_launch.py
This is the main launch file that launches the ```twist_mixer``` node and ```joy_launch.py``` if the corresponding launch argument is set:

* ```joy```: Enable/Disable Joy related packages in the Joy launch file: [joy_launch.py](https://github.com/adityakamath/akros2_base/blob/humble/launch/joy_launch.py) (Default: ```True```)
* ```joy_config```: Configures joystick mapping: ```ps4``` ([PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/)), ```stadia``` ([Stadia](https://stadia.google.com/controller/)), ```sn30pro``` ([8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/), ```steamdeck``` ([Steam Deck](https://store.steampowered.com/steamdeck): mapping is identical to PS3/Sixaxis controllers) (Default: ```steamdeck```)

### joy_launch.py
This launch files launches the ```joy```, ```teleop_twist_joy``` and ```joy_mode_handler``` nodes and uses the ```joy_config``` launch argument to configure the controller. This node can be launched individually or via ```drive_launch.py``` by setting the ```joy``` launch argument. Parameters for ```teleop_twist_joy``` and ```joy_mode_handler``` are set using the ```_twist_config.yaml``` and ```_mode_config.yaml``` configs in ```config/joy/```.

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
