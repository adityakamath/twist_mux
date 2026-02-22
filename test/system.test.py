# Copyright 2019 Canonical, Ltd
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

import os
import sys
sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(__file__))))

from rate_publishers import RatePublishers, TimeoutManager
import unittest

import launch
from launch.actions.execute_process import ExecuteProcess

import launch_ros.actions

import launch_testing

import time
import threading
from rclpy.executors import MultiThreadedExecutor

import rclpy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped


def generate_test_description():
    """Generate launch description for the twist_mux system test."""
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dir_path = os.path.dirname(os.path.realpath(__file__))

    parameters_file = os.path.join(
        dir_path, 'system_config.yaml'
    )

    twist_mux = launch_ros.actions.Node(
        package='twist_mux', executable='twist_mux',
        parameters=[parameters_file], env=proc_env)

    publisher = ExecuteProcess(
        cmd=['ros2 topic pub /lock_1 std_msgs/Bool "data: False" -r 20'],
        shell=True, env=proc_env
    )

    return launch.LaunchDescription([
        twist_mux,
        publisher,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ])


def twist(x=0.0, r=0.0):
    """Return a Twist for the given linear and rotation speed."""
    t = Twist()
    t.linear.x = x
    t.angular.z = r
    return t


class TestTwistMux(unittest.TestCase):

    # Value (in seconds) >= the highest topic/lock timeout.
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures: ROS context, node, publishers, and executor."""
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)

        cls.node = rclpy.create_node(
            'test_node', context=cls.context)

        # Subscribe to TwistStamped output (global use_stamped: true).
        # Use a lambda to avoid descriptor issues with threaded callbacks.
        cls._msg = None
        cls._subscription = cls.node.create_subscription(
            TwistStamped, '/cmd_vel_out',
            lambda msg: setattr(TestTwistMux, '_msg', msg),
            1)

        cls.executor = MultiThreadedExecutor(
            context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)

        cls._publishers = RatePublishers(cls.context)
        cls._vel1 = cls._publishers.add_topic('vel_1', Twist)
        cls._vel2 = cls._publishers.add_topic('vel_2', TwistStamped)
        cls._vel3 = cls._publishers.add_topic('vel_3', TwistStamped)

        cls._lock1 = cls._publishers.add_topic('lock_1', Bool)
        cls._lock2 = cls._publishers.add_topic('lock_2', Bool)

        cls.executor.add_node(cls._vel1)
        cls.executor.add_node(cls._vel2)
        cls.executor.add_node(cls._vel3)
        cls.executor.add_node(cls._lock1)
        cls.executor.add_node(cls._lock2)

        cls._timeout_manager = TimeoutManager()
        cls._timeout_manager.add(cls._publishers)
        cls._timeout_manager.spin_thread()

        cls.exec_thread = threading.Thread(target=cls.executor.spin)
        cls.exec_thread.start()

        # Give twist_mux time to start up and subscribe to topics.
        time.sleep(2.0)

        # Publish unlock so lock_1/lock_2 don't appear expired (stamp=0 -> locked)
        unlock = Bool()
        unlock.data = False
        cls._lock1.pub(unlock, rate=20)
        cls._lock2.pub(unlock, rate=20)
        time.sleep(1.0)

    def tearDown(self):
        """Stop velocity publishers, keep locks unlocked, and wait for timeouts.

        Stops all velocity publishers after each test, ensures locks remain
        unlocked, and sleeps long enough for any published velocity messages
        to expire in twist_mux before the next test starts.
        """
        self._vel1.stop()
        self._vel2.stop()
        self._vel3.stop()

        # Ensure locks remain unlocked
        unlock = Bool()
        unlock.data = False
        self._lock1.pub(unlock, rate=20)
        self._lock2.pub(unlock, rate=20)

        # Wait for previously published messages to time out,
        # since we aren't restarting twist_mux.
        #
        # This sleeping time must be higher than any of the
        # timeouts in system_config.yaml.
        time.sleep(self.TOPIC_TIMEOUT)
        TestTwistMux._msg = None

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS context and node."""
        cls._timeout_manager.shutdown()
        cls.executor.shutdown()
        cls.exec_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    @classmethod
    def _publish_and_wait(cls, publishers_msgs, timeout=3.0):
        """Actively publish messages and wait for output from twist_mux.

        Args:
            publishers_msgs: List of (rate_publisher, message) tuples.
            timeout: Maximum time in seconds to wait.

        Returns:
            The received TwistStamped message, or None.
        """
        TestTwistMux._msg = None
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            for pub, msg in publishers_msgs:
                pub._publisher.publish(msg)
            if TestTwistMux._msg is not None:
                return TestTwistMux._msg
            time.sleep(0.02)
        return TestTwistMux._msg

    def test_empty(self):
        """Verify twist_mux does not publish when no velocity inputs are active."""
        msg = self._publish_and_wait([])
        self.assertIsNone(msg, 'twist_mux should not be publishing without any input')

    def test_basic(self):
        """Verify basic twist forwarding and Twist-to-TwistStamped conversion.

        vel_1 (priority 5, use_stamped: false) publishes Twist. Global
        use_stamped: true means output is TwistStamped with a valid timestamp.
        vel_2 (priority 1, no per-topic use_stamped) inherits the global
        use_stamped: true default and is also active, but vel_1 wins on priority.
        """
        t1 = twist(2.0)
        t2 = TwistStamped()
        t2.twist = twist(0.5)

        # vel_1 (priority 5) wins over vel_2 (priority 1); also exercises the
        # "inherits global use_stamped default" code path for input2.
        msg = self._publish_and_wait([(self._vel1, t1), (self._vel2, t2)])

        self.assertIsNotNone(msg, 'Expected a TwistStamped on cmd_vel_out')
        self.assertIsInstance(msg, TwistStamped)
        self.assertEqual(t1, msg.twist)
        # Verify timestamp is set (not zero) - confirms Twist->TwistStamped conversion
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.assertGreater(stamp_sec, 0.0,
                           'Converted TwistStamped should have a valid timestamp')

    def test_basic_with_priorities(self):
        """Verify higher-priority topic wins and timeout causes fallback.

        vel_1 publishes Twist (priority 5), vel_3 publishes TwistStamped
        (priority 10). Higher priority wins, then falls back on timeout.
        """
        t1 = twist(2.0)
        t2 = TwistStamped()
        t2.twist = twist(0.0, 1.0)

        # Publish twist from input1 @ 3Hz, it should be used.
        msg = self._publish_and_wait([(self._vel1, t1)])
        self.assertIsNotNone(msg, 'Expected vel_1 message')
        self.assertEqual(t1, msg.twist)

        # Publish twist from input3, it should have priority
        # over the one from input1.
        msg = self._publish_and_wait([(self._vel1, t1), (self._vel3, t2)])
        self.assertIsNotNone(msg, 'Expected vel_3 message (higher priority)')
        self.assertEqual(t2.twist, msg.twist)

        # Stop publishing input 3 and wait for it to timeout.
        # Speed should fall back to input 1.
        time.sleep(0.5)  # input3 timeout is 0.3 in .yaml file
        msg = self._publish_and_wait([(self._vel1, t1)])
        self.assertIsNotNone(msg, 'Expected vel_1 fallback after vel_3 timeout')
        self.assertEqual(t1, msg.twist)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self):
        # Check that twist_mux exits cleanly. Exit code 2 is also allowed
        # because ROS 2 nodes killed via SIGINT can exit with code 2.
        launch_testing.asserts.assertExitCodes(
            self.proc_info, allowable_exit_codes=[0, 2])
