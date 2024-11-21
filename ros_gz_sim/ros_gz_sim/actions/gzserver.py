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

"""Module for the GzServer action."""

from typing import List
from typing import Optional

from launch.action import Action
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


@expose_action('gz_server')
class GzServer(Action):
    """Action that executes a gz_server ROS [composable] node."""

    def __init__(
        self,
        *,
        world_sdf_file: Optional[SomeSubstitutionsType] = '',
        world_sdf_string: Optional[SomeSubstitutionsType] = '',
        container_name: Optional[SomeSubstitutionsType] = 'ros_gz_container',
        create_own_container: Optional[SomeSubstitutionsType] = 'False',
        use_composition: Optional[SomeSubstitutionsType] = 'False',
        **kwargs
    ) -> None:
        """
        Construct a gz_server action.

        All arguments are forwarded to `ros_gz_sim.launch.gz_server.launch.py`,
        so see the documentation of that class for further details.

        :param: world_sdf_file Path to the SDF world file.
        :param: world_sdf_string SDF world string.
        :param: container_name Name of container that nodes will load in if use composition.
        :param: create_own_container Whether to start a ROS container when using composition.
        :param: use_composition Use composed bringup if True.
        """
        super().__init__(**kwargs)
        self.__world_sdf_file = world_sdf_file
        self.__world_sdf_string = world_sdf_string
        self.__container_name = container_name
        self.__create_own_container = create_own_container
        self.__use_composition = use_composition

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse gz_server."""
        _, kwargs = super().parse(entity, parser)

        world_sdf_file = entity.get_attr(
            'world_sdf_file', data_type=str,
            optional=True)

        world_sdf_string = entity.get_attr(
            'world_sdf_string', data_type=str,
            optional=True)

        container_name = entity.get_attr(
            'container_name', data_type=str,
            optional=True)

        create_own_container = entity.get_attr(
            'create_own_container', data_type=str,
            optional=True)

        use_composition = entity.get_attr(
            'use_composition', data_type=str,
            optional=True)

        if isinstance(world_sdf_file, str):
            world_sdf_file = parser.parse_substitution(world_sdf_file)
            kwargs['world_sdf_file'] = world_sdf_file

        if isinstance(world_sdf_string, str):
            world_sdf_string = parser.parse_substitution(world_sdf_string)
            kwargs['world_sdf_string'] = world_sdf_string

        if isinstance(container_name, str):
            container_name = parser.parse_substitution(container_name)
            kwargs['container_name'] = container_name

        if isinstance(create_own_container, str):
            create_own_container = \
                parser.parse_substitution(create_own_container)
            kwargs['create_own_container'] = create_own_container

        if isinstance(use_composition, str):
            use_composition = parser.parse_substitution(use_composition)
            kwargs['use_composition'] = use_composition

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        if isinstance(self.__use_composition, list):
            self.__use_composition = self.__use_composition[0]

        if isinstance(self.__create_own_container, list):
            self.__create_own_container = self.__create_own_container[0]

        # Standard node configuration
        load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', self.__use_composition])),
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='gzserver',
                    output='screen',
                    parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                                 'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
                ),
            ],
        )

        # Composable node with container configuration
        load_composable_nodes_with_container = ComposableNodeContainer(
            condition=IfCondition(
                PythonExpression([self.__use_composition, ' and ', self.__create_own_container])
            ),
            name=self.__container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_sim',
                    plugin='ros_gz_sim::GzServer',
                    name='gz_server',
                    parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                                 'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )

        # Composable node without container configuration
        load_composable_nodes_without_container = LoadComposableNodes(
            condition=IfCondition(
                PythonExpression(
                    [self.__use_composition, ' and not ', self.__create_own_container]
                )
            ),
            target_container=self.__container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_sim',
                    plugin='ros_gz_sim::GzServer',
                    name='gz_server',
                    parameters=[{'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                                 'world_sdf_string': LaunchConfiguration('world_sdf_string')}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        )

        return [
            load_nodes,
            load_composable_nodes_with_container,
            load_composable_nodes_without_container
        ]
