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

"""Launch gzsim + ros_gz_bridge in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')
    container_name = LaunchConfiguration('container_name')
    create_own_container = LaunchConfiguration('create_own_container')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    bridge_params = LaunchConfiguration('bridge_params')

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

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of the bridge'
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
        description='Whether we should start a ROS container when using composition.',
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

    ros_gz_bridge_action = RosGzBridge(
        bridge_name=bridge_name,
        config_file=config_file,
        container_name=container_name,
        create_own_container=create_own_container,
        namespace=namespace,
        use_composition=use_composition,
        use_respawn=use_respawn,
        log_level=log_level,
        bridge_params=bridge_params,
    )

    spawn_model_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
        launch_arguments=[('world', world),
                          ('file', file),
                          ('model_string', model_string),
                          ('topic', topic),
                          ('entity_name', entity_name),
                          ('allow_renaming', allow_renaming),
                          ('x', x),
                          ('y', y),
                          ('z', z),
                          ('R', roll),
                          ('P', pitch),
                          ('Y', yaw), ])

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
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_file_cmd)
    ld.add_action(declare_model_string_cmd)
    ld.add_action(declare_topic_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(declare_allow_renaming_cmd)
    # Add the actions to launch all of the bridge + spawn_model nodes
    ld.add_action(ros_gz_bridge_action)
    ld.add_action(spawn_model_description)

    return ld
