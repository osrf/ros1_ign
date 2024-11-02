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

"""Launch ros_gz bridge in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    declare_create_own_container_cmd = DeclareLaunchArgument(
        'create_own_container',
        default_value='False',
        description='Whether the bridge should start its own ROS container when using composition \
          (not recommended). This option should only be set to true if you plan to put your ROS \
          node in the container created by the bridge. This is not needed if you want Gazebo and \
          the bridge to be in the same ROS container.',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_bridge_params_cmd = DeclareLaunchArgument(
        'bridge_params', default_value='', description='Extra parameters to pass to the bridge.'
    )

    ros_gz_bridge_action = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file'),
        container_name=LaunchConfiguration('container_name'),
        create_own_container=LaunchConfiguration('create_own_container'),
        namespace=LaunchConfiguration('namespace'),
        use_composition=LaunchConfiguration('use_composition'),
        use_respawn=LaunchConfiguration('use_respawn'),
        log_level=LaunchConfiguration('log_level'),
        bridge_params=LaunchConfiguration('bridge_params')
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_create_own_container_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_bridge_params_cmd)
    # Add the ros_gz_bridge action
    ld.add_action(ros_gz_bridge_action)
    return ld
