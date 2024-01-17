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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException

class MotionDetector(Node):
    def __init__(self, node_name='motion_detector'):
        super().__init__(node_name)

        self.declare_parameter('angular_x_threshold', 0.0)
        self.declare_parameter('angular_y_threshold', 0.0)
        self.declare_parameter('angular_z_threshold', 0.0)
        
        self._motion_status = Bool()

        self.create_subscription(Imu, 'imu', self.cb_imu, 1)
        self._pub_status = self.create_publisher(Bool, 'in_motion', 1)

        self.get_logger().info('Initialized')

    def cb_imu(self, msg):
        """
        :type msg: Imu
        """       
        ang_x_thresh = self.get_parameter('angular_x_threshold').value
        ang_y_thresh = self.get_parameter('angular_y_threshold').value
        ang_z_thresh = self.get_parameter('angular_z_threshold').value

        if (abs(msg.angular_velocity.x) <= ang_x_thresh) and (abs(msg.angular_velocity.y) <= ang_y_thresh) and (abs(msg.angular_velocity.z) <= ang_z_thresh):
            self._motion_status.data = False
        else:
            self._motion_status.data = True

        self._pub_status.publish(self._motion_status)

def main(args=None):
    rclpy.init(args=args)
    node = MotionDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()