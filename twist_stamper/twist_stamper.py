# Copyright 2021 Josh Newans
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Copyright holder string nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
