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

from rate_publishers import RatePublishers, TimeoutManager
import os
import unittest
import sys

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

sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(__file__))))


def generate_test_description(use_stamped=None):
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    params = [{"use_stamped": use_stamped}] if use_stamped is not None else []
    twist_mux = launch_ros.actions.Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[os.path.join(dir_path, "system_config.yaml")] + params,
        env=proc_env,
    )
    return launch.LaunchDescription(
        [
            twist_mux,
            ExecuteProcess(
                cmd=['ros2 topic pub /lock_1 std_msgs/Bool "data: False" -r 20'],
                shell=True,
                env=proc_env,
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def twist(x=0.0, r=0.0):
    t = Twist()
    t.linear.x, t.angular.z = x, r
    return t


def twist_stamped(x=0.0, r=0.0):
    t = TwistStamped()
    t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
    t.twist.linear.x, t.twist.angular.z = x, r
    return t


class TestBase(unittest.TestCase):
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(f"test_{cls.__name__}", context=cls.context)
        cls._msg = None
        cls._subscription = cls.node.create_subscription(
            cls.SUB_TYPE,
            "/cmd_vel_out" if cls.SUB_TYPE == TwistStamped else "cmd_vel_out",
            lambda msg: setattr(cls, "_msg", msg),
            1,
        )
        cls.executor = MultiThreadedExecutor(context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)
        cls._publishers = RatePublishers(cls.context)
        for name, msg_type in cls.PUBS.items():
            setattr(cls, f"_vel_{name[-1]}", cls._publishers.add_topic(name, msg_type))
        cls._lock1 = cls._publishers.add_topic("lock_1", Bool)
        cls._lock2 = cls._publishers.add_topic("lock_2", Bool)
        for v in [cls._vel_1, cls._vel_2, cls._vel_3, cls._lock1, cls._lock2]:
            cls.executor.add_node(v)
        cls._timeout_manager = TimeoutManager()
        cls._timeout_manager.add(cls._publishers)
        cls._timeout_manager.spin_thread()
        cls.exec_thread = threading.Thread(target=cls.executor.spin)
        cls.exec_thread.start()
        time.sleep(3.0)
        unlock = Bool(data=False)
        cls._lock1.pub(unlock, rate=20)
        cls._lock2.pub(unlock, rate=20)

    @classmethod
    def _publish_and_wait(cls, pubs_msgs, timeout=3.0):
        cls._msg = None
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            for pub, msg in pubs_msgs:
                pub._publisher.publish(msg)
            if cls._msg is not None:
                return cls._msg
            time.sleep(0.02)
        return cls._msg

    @classmethod
    def tearDownClass(cls):
        for v in [cls._vel_1, cls._vel_2, cls._vel_3]:
            v.stop()
        unlock = Bool(data=False)
        cls._lock1.pub(unlock, rate=20)
        cls._lock2.pub(unlock, rate=20)
        time.sleep(cls.TOPIC_TIMEOUT)
        cls._timeout_manager.shutdown()
        cls.executor.shutdown()
        cls.exec_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)


class TestTwistMux(TestBase):
    SUB_TYPE = Twist
    PUBS = {"vel_1": Twist, "vel_2": Twist, "vel_3": Twist}

    def test_empty(self):
        self.assertIsNone(self._publish_and_wait([]))

    def test_basic(self):
        t = twist(2.0)
        self._vel_1.pub(t, rate=5)
        start = self.node.get_clock().now()
        while 0.3 > (self.node.get_clock().now() - start).nanoseconds / 1e9:
            if self._msg is not None:
                break
            time.sleep(0.01)
        self.assertEqual(t, self._msg)


class TestTwistStamped(TestBase):
    SUB_TYPE = TwistStamped
    PUBS = {"vel_1": Twist, "vel_2": TwistStamped, "vel_3": TwistStamped}

    def test_twist_to_twist_stamped(self):
        msg = self._publish_and_wait([(self._vel_1, twist(2.0))])
        self.assertIsNotNone(msg)
        self.assertIsInstance(msg, TwistStamped)
        self.assertEqual(twist(2.0), msg.twist)
        self.assertGreater(msg.header.stamp.sec, 0)
        self.assertEqual("odom", msg.header.frame_id)

    def test_twist_stamped_passthrough(self):
        msg = self._publish_and_wait([(self._vel_2, twist_stamped(0.5))])
        self.assertIsNotNone(msg)
        self.assertEqual(twist_stamped(0.5).twist, msg.twist)


class TestStampedToTwist(TestBase):
    SUB_TYPE = Twist
    PUBS = {"vel_1": TwistStamped, "vel_2": Twist, "vel_3": TwistStamped}

    def test_twist_stamped_to_twist(self):
        msg = self._publish_and_wait([(self._vel_1, twist_stamped(2.0))])
        self.assertIsNotNone(msg)
        self.assertIsInstance(msg, Twist)
        self.assertEqual(twist_stamped(2.0).twist, msg)


def generate_test_description_stamped_to_twist():
    return generate_test_description(use_stamped=False)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self):
        launch_testing.asserts.assertExitCodes(
            self.proc_info, allowable_exit_codes=[0, 2]
        )
