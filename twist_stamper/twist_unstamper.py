# Copyright 2021 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class TwistUnstamper(Node):

    def __init__(self):
        super().__init__('twist_unstamper')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_out', 10)

        self.subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel_in',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, inMsg):

        self.publisher_.publish(inMsg.twist)
        # self.get_logger().info('X: "%f"' % outMsg.twist.linear.x)


def main(args=None):
    rclpy.init(args=args)

    twist_unstamper = TwistUnstamper()
    rclpy.spin(twist_unstamper)

    twist_unstamper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
