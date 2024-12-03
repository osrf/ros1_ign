# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Launch gz_server in a component container."""

import os

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_sim.actions import GzServer


class GazeboRosPaths:

    @staticmethod
    def get_paths():
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []

        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
            if os.path.isfile(package_file_path):
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    if export.tagname == 'gazebo_ros':
                        if 'gazebo_model_path' in export.attributes:
                            xml_path = export.attributes['gazebo_model_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_model_path.append(xml_path)
                        if 'plugin_path' in export.attributes:
                            xml_path = export.attributes['plugin_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_plugin_path.append(xml_path)
                        if 'gazebo_media_path' in export.attributes:
                            xml_path = export.attributes['gazebo_media_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_media_path.append(xml_path)

        gazebo_model_path = os.pathsep.join(gazebo_model_path + gazebo_media_path)
        gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path)

        return gazebo_model_path, gazebo_plugin_path


def generate_launch_description():

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value=TextSubstitution(text=''),
        description='Path to the SDF world file')
    declare_world_sdf_string_cmd = DeclareLaunchArgument(
        'world_sdf_string', default_value=TextSubstitution(text=''),
        description='SDF world string')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',)
    declare_create_own_container_cmd = DeclareLaunchArgument(
        'create_own_container', default_value='False',
        description='Whether we should start our own ROS container when using composition.',)
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    model_paths, plugin_paths = GazeboRosPaths.get_paths()
    system_plugin_path_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.pathsep.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default=''),
            plugin_paths,
        ]))
    resource_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', default=''),
            model_paths,
        ]))

    gz_server_action = GzServer(
        world_sdf_file=LaunchConfiguration('world_sdf_file'),
        world_sdf_string=LaunchConfiguration('world_sdf_string'),
        container_name=LaunchConfiguration('container_name'),
        create_own_container=LaunchConfiguration('create_own_container'),
        use_composition=LaunchConfiguration('use_composition'),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_sdf_file_cmd)
    ld.add_action(declare_world_sdf_string_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_create_own_container_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(system_plugin_path_env)
    ld.add_action(resource_path_env)
    # Add the gz_server action
    ld.add_action(gz_server_action)

    return ld
