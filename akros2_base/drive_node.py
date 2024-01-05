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

import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from akros2_drive.joy_mode_handler import JoystickModeHandler
from akros2_drive.twist_mixer_node import TwistMixer

def main(args=None):
    rclpy.init(args=args)
    
    try:
        joy_mode_handler = JoystickModeHandler(node_name='joy_mode_handler')
        twist_mixer = TwistMixer(node_name='twist_mixer')

        executor = MultiThreadedExecutor()
        executor.add_node(twist_mixer)
        executor.add_node(joy_mode_handler)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            twist_mixer.destroy_node()
            joy_mode_handler.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
