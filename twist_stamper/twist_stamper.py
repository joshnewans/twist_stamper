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

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class TwistStamper(Node):

    def __init__(self):
        super().__init__('twist_stamper')

        self.declare_parameter("frame_id")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, inMsg):
        outMsg = TwistStamped()
        outMsg.header = Header()
        outMsg.header.stamp = self.get_clock().now().to_msg()
        outMsg.header.frame_id = self.frame_id
        outMsg.twist = inMsg

        self.publisher_.publish(outMsg)
        # self.get_logger().info('X: "%f"' % outMsg.twist.linear.x)


def main(args=None):
    rclpy.init(args=args)

    twist_stamper = TwistStamper()
    rclpy.spin(twist_stamper)

    twist_stamper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
