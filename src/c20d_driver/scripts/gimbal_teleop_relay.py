#!/usr/bin/env python3
# Copyright 2026 wee733
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Relay the Foxglove gimbal pad's degree/second Twist convention to C20D."""

import math

from c20d_driver.msg import AngularRateCommand
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile


class GimbalTeleopRelay(Node):
    """Forward one output per input, without refreshing commands during a disconnect."""

    def __init__(self):
        """Create the command output, input subscription and schema/observation echo."""
        super().__init__('gimbal_teleop_relay')
        self.publisher = self.create_publisher(
            AngularRateCommand, '/c20d_driver/command/angular_rate', 1)
        echo_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.echo = self.create_publisher(Twist, '/gimbal_teleop/input_echo', echo_qos)
        self.subscription = self.create_subscription(
            Twist, '/gimbal_teleop/cmd', self.on_twist, 1)

    def on_twist(self, message):
        """Map x/y to rates; release and the middle button both stop motion."""
        if not all(math.isfinite(v) for v in (
                message.angular.x, message.angular.y, message.linear.z)):
            self.get_logger().warning('Rejected non-finite gimbal teleop input')
            return
        self.echo.publish(message)
        stamp = self.get_clock().now().to_msg()
        # The real pad sends linear.z=1 on BOTH direction release and middle press.
        # Those identical packets cannot identify a center request. Ignore linear
        # fields; use the separate RelativeAngleCommand button for centering.
        command = AngularRateCommand()
        command.header.stamp = stamp
        command.roll_deg_s = float(message.angular.x)
        command.pitch_deg_s = float(message.angular.y)
        self.publisher.publish(command)


def main(args=None):
    """Run the relay; the driver watchdog remains responsible for stream loss."""
    rclpy.init(args=args)
    node = GimbalTeleopRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
