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

"""Exercise the compiled C++ ROS node through a pseudo-terminal; no hardware required."""

import binascii
import math
import os
import pty
import signal
import struct
import subprocess
import time

from c20d_driver.msg import CarrierData
from c20d_driver.msg import GimbalCommand
from c20d_driver.msg import GimbalState
from c20d_driver.srv import DeviceCommand
import rclpy
from rclpy.qos import qos_profile_sensor_data


def test_ros_serial_roundtrip():
    """Check feedback, command bits, acknowledgements, watchdog and reconnect behavior."""
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    domain = 100 + os.getpid() % 100
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('c20d_pty_test')
    states = []
    subscription = node.create_subscription(
        GimbalState, '/c20d_driver/state', states.append, qos_profile_sensor_data)
    publisher = node.create_publisher(GimbalCommand, '/c20d_driver/command', 1)
    carrier = node.create_publisher(CarrierData, '/c20d_driver/carrier', 1)
    client = node.create_client(DeviceCommand, '/c20d_driver/device_command')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    process = subprocess.Popen([
        os.environ['C20D_TEST_EXECUTABLE'], '--ros-args', '-p',
        'port:=' + os.ttyname(slave), '-p', 'command_timeout:=0.2',
        '-p', 'auxiliary_timeout:=0.2', '-p', 'feedback_timeout:=0.3'], env=environment)
    wire = bytearray()
    sent = []
    respond = True
    acknowledged_code = 0

    def pump(seconds):
        nonlocal acknowledged_code
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, 'C++ node exited unexpectedly'
            try:
                wire.extend(os.read(master, 8192))
            except BlockingIOError:
                pass
            while len(wire) >= 40:
                assert wire[:2] == b'\xa9\x5b'
                packet = bytes(wire[:40])
                del wire[:40]
                assert binascii.crc_hqx(packet[:38], 0) == int.from_bytes(packet[38:], 'big')
                sent.append(packet)
                if packet[2] >> 3:
                    acknowledged_code = packet[2] >> 3
                if respond:
                    reply = bytearray.fromhex(
                        'b5 9a 24 00 19 21 ff ff 00 00 fe ff '
                        '09 00 8c 35 b9 01 52 c9 5f 00 fe ff')
                    reply[5] = (acknowledged_code << 3) | 1
                    reply.extend(struct.pack('>H', binascii.crc_hqx(reply, 0)))
                    os.write(master, reply)
            rclpy.spin_once(node, timeout_sec=0.002)

    try:
        pump(2.0)
        assert states and publisher.get_subscription_count() == 1
        state = states[-1]
        assert state.firmware_version_raw == 36
        assert state.upright and state.temperature_ready
        assert list(state.camera_rate_raw_pyr) == [-1, 0, -2]
        assert math.isclose(state.relative_angle_rpy[0], math.radians(0.95))
        assert math.isclose(state.relative_angle_rpy[1], math.radians(-139.98))
        assert all(packet[2] >> 3 == 0 for packet in sent), 'Startup must only poll'

        command = GimbalCommand()
        command.roll = math.radians(2.0)
        command.pitch = math.radians(-3.0)
        command.center_roll = True
        publisher.publish(command)
        pump(0.15)
        assert any(packet[2] >> 3 == 4 for packet in sent)
        assert sent[-1][2] >> 3 == 0, 'Command code must clear after acknowledgement'
        assert sent[-1][4] == 0x18
        assert sent[-1][7] == 0x10
        assert struct.unpack_from('<h', sent[-1], 5)[0] == 200
        assert struct.unpack_from('<h', sent[-1], 8)[0] == -300
        previous = sent[-1][4:13]

        command.roll = float('nan')
        publisher.publish(command)
        pump(0.05)
        assert sent[-1][4:13] == previous
        command.roll = math.radians(61.0)
        publisher.publish(command)
        pump(0.05)
        assert sent[-1][4:13] == previous

        command = GimbalCommand()
        command.roll_mode = 2
        command.pitch_mode = 2
        command.pitch = math.radians(2.0)
        publisher.publish(command)
        pump(0.1)
        assert sent[-1][4] == 0x28
        assert sent[-1][7] == 0x20
        assert struct.unpack_from('<h', sent[-1], 8)[0] == -200

        command = GimbalCommand()
        command.roll_type = 2
        command.pitch_type = 2
        command.pitch = math.radians(5.0)
        publisher.publish(command)
        pump(0.1)
        assert sent[-1][7] == 0x90
        assert struct.unpack_from('<h', sent[-1], 8)[0] == 50
        pump(0.25)
        assert struct.unpack_from('<h', sent[-1], 8)[0] == 0

        inertial = CarrierData()
        inertial.valid = True
        inertial.attitude_rpy = [math.radians(5)] * 3
        inertial.acceleration_neu = [0.1] * 3
        carrier.publish(inertial)
        pump(0.1)
        assert sent[-1][13] == 0x80
        assert struct.unpack_from('<hhh', sent[-1], 14) == (500, 500, 500)
        pump(0.2)
        assert sent[-1][13] == 0

        assert client.service_is_ready()
        request = DeviceCommand.Request()
        request.command = 2
        future = client.call_async(request)
        pump(0.15)
        assert future.done() and future.result().accepted
        triggers = [p[2] & 7 for p in sent if p[2] >> 3 == 2]
        future = client.call_async(request)
        pump(0.15)
        assert future.done() and future.result().accepted
        new_triggers = [p[2] & 7 for p in sent if p[2] >> 3 == 2]
        assert triggers[-1] != new_triggers[-1], 'Repeat command must change trigger count'

        respond = False
        pump(0.45)
        reconnect_start = len(sent)
        pump(1.0)
        respond = True
        pump(1.2)
        assert len(sent) > reconnect_start
        assert all(p[2] >> 3 == 0 for p in sent[reconnect_start:])
        assert struct.unpack_from('<h', sent[-1], 8)[0] == 0
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        os.close(slave)
        os.close(master)
