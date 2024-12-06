# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing


def generate_test_description():
    expected_file_name = 'nonexistent/long/file_name'
    create = Node(package='ros_gz_sim', executable='create',
                  parameters=[{'world': 'default', 'file': expected_file_name}], output='screen')
    test_create = Node(package='ros_gz_sim', executable='test_create', output='screen')
    return LaunchDescription([
        create,
        test_create,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class WaitForTests(unittest.TestCase):

    def test_termination(self, test_create, proc_info):
        proc_info.assertWaitForShutdown(process=test_create, timeout=200)


@launch_testing.post_shutdown_test()
class CreateTest(unittest.TestCase):

    def test_output(self, expected_file_name, test_create, proc_output):
        launch_testing.asserts.assertInStdout(proc_output, expected_file_name, test_create)
