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

"""Launch create to spawn models in gz sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    world = LaunchConfiguration('world')
    file = LaunchConfiguration('file')
    model_string = LaunchConfiguration('model_string')
    topic = LaunchConfiguration('topic')
    entity_name = LaunchConfiguration('entity_name')
    allow_renaming = LaunchConfiguration('allow_renaming')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    roll = LaunchConfiguration('R', default='0.0')
    pitch = LaunchConfiguration('P', default='0.0')
    yaw = LaunchConfiguration('Y', default='0.0')

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text=''),
        description='World name')
    declare_file_cmd = DeclareLaunchArgument(
        'file', default_value=TextSubstitution(text=''),
        description='SDF filename')
    declare_model_string_cmd = DeclareLaunchArgument(
        'model_string',
        default_value='',
        description='XML(SDF) string',
    )
    declare_topic_cmd = DeclareLaunchArgument(
        'topic', default_value=TextSubstitution(text=''),
        description='Get XML from this topic'
    )
    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name', default_value=TextSubstitution(text=''),
        description='Name of the entity'
    )
    declare_allow_renaming_cmd = DeclareLaunchArgument(
        'allow_renaming', default_value='False',
        description='Whether the entity allows renaming or not'
    )

    load_nodes = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': world,
                     'file': file,
                     'string': model_string,
                     'topic': topic,
                     'name': entity_name,
                     'allow_renaming': allow_renaming,
                     'x': x,
                     'y': y,
                     'z': z,
                     'R': roll,
                     'P': pitch,
                     'Y': yaw,
                     }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_file_cmd)
    ld.add_action(declare_model_string_cmd)
    ld.add_action(declare_topic_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(declare_allow_renaming_cmd)
    # Add the actions to launch all of the create nodes
    ld.add_action(load_nodes)

    return ld
