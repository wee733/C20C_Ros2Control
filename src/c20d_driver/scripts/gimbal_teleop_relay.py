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
from c20d_driver.msg import RelativeAngleCommand
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
        self.angle_publisher = self.create_publisher(
            RelativeAngleCommand, '/c20d_driver/command/relative_angle', 1)
        self.center_held = False
        self.centering = False
        echo_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.echo = self.create_publisher(Twist, '/gimbal_teleop/input_echo', echo_qos)
        self.subscription = self.create_subscription(
            Twist, '/gimbal_teleop/cmd', self.on_twist, 1)

    def on_twist(self, message):
        """Map x/y to rates and the rising edge of linear.z > 0.5 to centering."""
        if not all(math.isfinite(v) for v in (
                message.angular.x, message.angular.y, message.linear.z)):
            self.get_logger().warning('Rejected non-finite gimbal teleop input')
            return
        self.echo.publish(message)
        stamp = self.get_clock().now().to_msg()
        if message.linear.z > 0.5:
            if not self.center_held:
                # One message atomically replaces both velocity targets with zero-angle
                # targets. A separate rate stop on another topic could arrive after it.
                command = RelativeAngleCommand()
                command.header.stamp = stamp
                command.center_roll = True
                command.center_pitch = True
                self.angle_publisher.publish(command)
                self.centering = True
                self.get_logger().info('Requested gimbal centering')
            self.center_held = True
            return

        self.center_held = False
        if self.centering and message.angular.x == 0.0 and message.angular.y == 0.0:
            # The pad's release/idle packet must not cancel an unfinished return.
            return
        self.centering = False
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
