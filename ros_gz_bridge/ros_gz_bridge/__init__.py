# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from dataclasses import dataclass

import os

from ros_gz_bridge.mappings import MAPPINGS

from rosidl_pycommon import expand_template

from . import actions

__all__ = [
    'actions',
]


@dataclass
class MessageMapping:
    # Class to represent mapping between ROS2 and Gazebo types
    ros2_package_name: str
    ros2_message_name: str
    gz_message_name: str

    def ros2_string(self):
        # Return ROS2 string version of a message (eg std_msgs/msg/Bool)
        return f'{self.ros2_package_name}/msg/{self.ros2_message_name}'

    def ros2_type(self):
        # Return ROS2 type of a message (eg std_msgs::msg::Bool)
        return f'{self.ros2_package_name}::msg::{self.ros2_message_name}'

    def ign_string(self):
        # Return GZ string version of a message (eg ignition.msgs.Bool)
        return f'ignition.msgs.{self.gz_message_name}'

    def ign_type(self):
        # Return GZ type of a message (eg gz::msgs::Bool)
        return f'gz::msgs::{self.gz_message_name}'

    def gz_string(self):
        # Return GZ string version of a message (eg ignition.msgs.Bool)
        return f'gz.msgs.{self.gz_message_name}'

    def gz_type(self):
        # Return GZ type of a message (eg gz::msgs::Bool)
        return f'gz::msgs::{self.gz_message_name}'

    def unique(self):
        return f'{self.gz_message_name.lower()}_' \
               f'{self.ros2_package_name}_{self.ros2_message_name.lower()}'


def mappings(gz_msgs_ver):
    # Generate MessageMapping object for all known mappings
    data = []
    for (ros2_package_name, mappings) in MAPPINGS.items():
        for mapping in sorted(mappings):
            data.append(MessageMapping(
                ros2_package_name=ros2_package_name,
                ros2_message_name=mapping.ros_type,
                gz_message_name=mapping.gz_type
            ))
    return sorted(data, key=lambda mm: mm.ros2_string())


def generate_cpp(output_path, template_dir, gz_msgs_ver):
    # Generate cpp/hpp files for ros-gz package/message mappings
    data = {}
    data['mappings'] = mappings(gz_msgs_ver)

    template_file = os.path.join(template_dir, 'get_mappings.cpp.em')
    output_file = os.path.join(output_path, 'get_mappings.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)

    unique_package_names = {
            mapping.ros2_package_name for mapping in data['mappings']}
    data['ros2_package_names'] = list(unique_package_names)

    template_file = os.path.join(template_dir, 'get_factory.cpp.em')
    output_file = os.path.join(output_path, 'get_factory.cpp')
    expand_template(template_file, data, output_file)

    for ros2_package_name in unique_package_names:
        data_pkg = {
                'ros2_package_name': ros2_package_name,
                'mappings': [
                    m for m in data['mappings']
                    if m.ros2_package_name == ros2_package_name
                ]
        }
        template_file = os.path.join(template_dir, 'pkg_factories.hpp.em')
        output_file = os.path.join(
            output_path, 'factories/%s.hpp' % ros2_package_name)
        expand_template(template_file, data_pkg, output_file)

        template_file = os.path.join(template_dir, 'pkg_factories.cpp.em')
        output_file = os.path.join(
            output_path, 'factories/%s.cpp' % ros2_package_name)
        expand_template(template_file, data_pkg, output_file)


def generate_test_cpp(output_path, template_dir, gz_msgs_ver):
    # Generate cpp/hpp files for ros-gz package/message mappings
    data = {}
    data['mappings'] = mappings(gz_msgs_ver)

    template_file = os.path.join(template_dir, 'gz_publisher.cpp.em')
    output_file = os.path.join(output_path, 'gz_publisher.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)

    template_file = os.path.join(template_dir, 'ros_publisher.cpp.em')
    output_file = os.path.join(output_path, 'ros_publisher.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)

    template_file = os.path.join(template_dir, 'gz_subscriber.cpp.em')
    output_file = os.path.join(output_path, 'gz_subscriber.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)

    unique_package_names = {
            mapping.ros2_package_name for mapping in data['mappings']}
    for ros2_package_name in unique_package_names:
        data_pkg = {
                'ros2_package_name': ros2_package_name,
                'mappings': [
                    m for m in data['mappings']
                    if m.ros2_package_name == ros2_package_name
                ]
        }

        template_file = os.path.join(template_dir, 'ros_pkg_subscriber.cpp.em')
        output_file = os.path.join(
            output_path, '%s_subscriber.cpp' % ros2_package_name)
        expand_template(template_file, data_pkg, output_file)
