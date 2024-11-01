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

"""Launch remove models in gz sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    world = LaunchConfiguration('world')
    entity_name = LaunchConfiguration('entity_name')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text=''),
        description='World name')
    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name', default_value=TextSubstitution(text=''),
        description='SDF filename')

    remove = Node(
        package='ros_gz_sim',
        executable='remove',
        output='screen',
        parameters=[{'world': world, 'entity_name': entity_name}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(remove)

    return ld
