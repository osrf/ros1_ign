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
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


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

    def node_setup(context):
        bridge_name = LaunchConfiguration('bridge_name')
        config_file = LaunchConfiguration('config_file')
        container_name = LaunchConfiguration('container_name')
        create_own_container = LaunchConfiguration('create_own_container')
        namespace = LaunchConfiguration('namespace')
        use_composition = LaunchConfiguration('use_composition')
        use_respawn = LaunchConfiguration('use_respawn')
        log_level = LaunchConfiguration('log_level')
        bridge_params = LaunchConfiguration('bridge_params')

        string_bridge_params = bridge_params.perform(context)
        # Remove unnecessary symbols from bridge_params
        simplified_bridge_params = string_bridge_params.translate({ord(i): None for i in '{} "\''})
        if simplified_bridge_params:
            # Parse from string to dictionary
            bridge_params_pairs = simplified_bridge_params.split(',')
            parsed_bridge_params = dict(pair.split(':') for pair in bridge_params_pairs)
        else:
            parsed_bridge_params = {}

        load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='bridge_node',
                    name=bridge_name,
                    namespace=namespace,
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[{'config_file': config_file, **parsed_bridge_params}],
                    arguments=['--ros-args', '--log-level', log_level],
                ),
            ],
        )

        load_composable_nodes_with_container = ComposableNodeContainer(
            condition=IfCondition(
                PythonExpression([use_composition, ' and ', create_own_container])),
            name=LaunchConfiguration('container_name'),
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_bridge',
                    plugin='ros_gz_bridge::RosGzBridge',
                    name=bridge_name,
                    namespace=namespace,
                    parameters=[{'config_file': config_file, **parsed_bridge_params}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )

        load_composable_nodes_without_container = LoadComposableNodes(
            condition=IfCondition(
                PythonExpression([use_composition, ' and not ', create_own_container])),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_bridge',
                    plugin='ros_gz_bridge::RosGzBridge',
                    name=bridge_name,
                    namespace=namespace,
                    parameters=[{'config_file': config_file, **parsed_bridge_params}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        )

        return [
            load_nodes,
            load_composable_nodes_with_container,
            load_composable_nodes_without_container
        ]

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
    # Add the actions to launch all of the bridge nodes
    node_setup_function = OpaqueFunction(function=node_setup)
    ld.add_action(node_setup_function)
    return ld
