# Copyright 2024 PAL Robotics S.L.
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


def generate_test_description(use_stamped_override=None):
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    parameters_file = os.path.join(dir_path, "test_twist_stamped.yaml")
    params = (
        [{"use_stamped": use_stamped_override}]
        if use_stamped_override is not None
        else [parameters_file]
    )
    if use_stamped_override is not None:
        params.insert(0, parameters_file)
    twist_mux = launch_ros.actions.Node(
        package="twist_mux", executable="twist_mux", parameters=params, env=proc_env
    )
    publisher = ExecuteProcess(
        cmd=['ros2 topic pub /lock_1 std_msgs/Bool "data: False" -r 20'],
        shell=True,
        env=proc_env,
    )
    return launch.LaunchDescription(
        [
            twist_mux,
            publisher,
            launch_testing.actions.ReadyToTest(),
        ]
    )


def twist(x=0.0, r=0.0):
    t = Twist()
    t.linear.x = x
    t.angular.z = r
    return t


def twist_stamped(x=0.0, r=0.0):
    t = TwistStamped()
    t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
    t.twist.linear.x = x
    t.twist.angular.z = r
    return t


class TestTwistStamped(unittest.TestCase):
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("test_node", context=cls.context)
        cls._msg = None
        cls._subscription = cls.node.create_subscription(
            TwistStamped,
            "/cmd_vel_out",
            lambda msg: setattr(TestTwistStamped, "_msg", msg),
            1,
        )
        cls.executor = MultiThreadedExecutor(context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)
        cls._publishers = RatePublishers(cls.context)
        cls._vel1 = cls._publishers.add_topic("vel_1", Twist)
        cls._vel2 = cls._publishers.add_topic("vel_2", TwistStamped)
        cls._vel3 = cls._publishers.add_topic("vel_3", TwistStamped)
        cls._lock1 = cls._publishers.add_topic("lock_1", Bool)
        cls._lock2 = cls._publishers.add_topic("lock_2", Bool)
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
        time.sleep(2.0)
        unlock = Bool()
        unlock.data = False
        cls._lock1.pub(unlock, rate=20)
        cls._lock2.pub(unlock, rate=20)
        time.sleep(1.0)

    @classmethod
    def _publish_and_wait(cls, publishers_msgs, timeout=3.0):
        TestTwistStamped._msg = None
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            for pub, msg in publishers_msgs:
                pub._publisher.publish(msg)
            if TestTwistStamped._msg is not None:
                return TestTwistStamped._msg
            time.sleep(0.02)
        return TestTwistStamped._msg

    def tearDown(self):
        self._vel1.stop()
        self._vel2.stop()
        self._vel3.stop()
        unlock = Bool()
        unlock.data = False
        self._lock1.pub(unlock, rate=20)
        self._lock2.pub(unlock, rate=20)
        time.sleep(self.TOPIC_TIMEOUT)
        TestTwistStamped._msg = None

    @classmethod
    def tearDownClass(cls):
        cls._timeout_manager.shutdown()
        cls.executor.shutdown()
        cls.exec_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_empty(self):
        msg = self._publish_and_wait([])
        self.assertIsNone(msg, "twist_mux should not be publishing without any input")

    def test_twist_to_twist_stamped(self):
        t1 = twist(2.0)
        msg = self._publish_and_wait([(self._vel1, t1)])
        self.assertIsNotNone(msg, "Expected TwistStamped on cmd_vel_out")
        self.assertIsInstance(msg, TwistStamped)
        self.assertEqual(t1, msg.twist)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.assertGreater(stamp_sec, 0.0, "TwistStamped should have valid timestamp")
        self.assertEqual(msg.header.frame_id, "odom", "frame_id should be set")

    def test_twist_stamped_passthrough(self):
        t2 = twist_stamped(0.5)
        msg = self._publish_and_wait([(self._vel2, t2)])
        self.assertIsNotNone(msg, "Expected TwistStamped on cmd_vel_out")
        self.assertIsInstance(msg, TwistStamped)
        self.assertEqual(t2.twist, msg.twist)


class TestStampedToTwist(unittest.TestCase):
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("test_node2", context=cls.context)
        cls._msg = None
        cls._subscription = cls.node.create_subscription(
            Twist,
            "/cmd_vel_out",
            lambda msg: setattr(TestStampedToTwist, "_msg", msg),
            1,
        )
        cls.executor = MultiThreadedExecutor(context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)
        cls._publishers = RatePublishers(cls.context)
        cls._vel1 = cls._publishers.add_topic("vel_1", TwistStamped)
        cls._vel2 = cls._publishers.add_topic("vel_2", Twist)
        cls._vel3 = cls._publishers.add_topic("vel_3", TwistStamped)
        cls._lock1 = cls._publishers.add_topic("lock_1", Bool)
        cls._lock2 = cls._publishers.add_topic("lock_2", Bool)
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
        time.sleep(2.0)
        unlock = Bool()
        unlock.data = False
        cls._lock1.pub(unlock, rate=20)
        cls._lock2.pub(unlock, rate=20)
        time.sleep(1.0)

    @classmethod
    def _publish_and_wait(cls, publishers_msgs, timeout=3.0):
        TestStampedToTwist._msg = None
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            for pub, msg in publishers_msgs:
                pub._publisher.publish(msg)
            if TestStampedToTwist._msg is not None:
                return TestStampedToTwist._msg
            time.sleep(0.02)
        return TestStampedToTwist._msg

    def tearDown(self):
        self._vel1.stop()
        self._vel2.stop()
        self._vel3.stop()
        unlock = Bool()
        unlock.data = False
        self._lock1.pub(unlock, rate=20)
        self._lock2.pub(unlock, rate=20)
        time.sleep(self.TOPIC_TIMEOUT)
        TestStampedToTwist._msg = None

    @classmethod
    def tearDownClass(cls):
        cls._timeout_manager.shutdown()
        cls.executor.shutdown()
        cls.exec_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_twist_stamped_to_twist(self):
        t1 = twist_stamped(2.0)
        msg = self._publish_and_wait([(self._vel1, t1)])
        self.assertIsNotNone(msg, "Expected Twist on cmd_vel_out")
        self.assertIsInstance(msg, Twist)
        self.assertEqual(t1.twist, msg)


def generate_test_description_stamped_to_twist():
    return generate_test_description(use_stamped_override=False)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self):
        launch_testing.asserts.assertExitCodes(
            self.proc_info, allowable_exit_codes=[0, 2]
        )
