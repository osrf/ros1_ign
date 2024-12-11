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

"""Module for the ros_gz bridge action."""

from typing import Dict, List, Optional, Union

from launch.action import Action
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import TextSubstitution
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


@expose_action('ros_gz_bridge')
class RosGzBridge(Action):
    """Action that executes a ros_gz bridge ROS [composable] node."""

    def __init__(
        self,
        *,
        bridge_name: SomeSubstitutionsType,
        config_file: SomeSubstitutionsType,
        container_name: SomeSubstitutionsType = 'ros_gz_container',
        create_own_container: Union[bool, SomeSubstitutionsType] = False,
        namespace: SomeSubstitutionsType = '',
        use_composition: Union[bool, SomeSubstitutionsType] = False,
        use_respawn: Union[bool, SomeSubstitutionsType] = False,
        log_level: SomeSubstitutionsType = 'info',
        bridge_params: SomeSubstitutionsType = '',
        **kwargs
    ) -> None:
        """
        Construct a ros_gz bridge action.

        :param: bridge_name Name of ros_gz_bridge  node
        :param: config_file YAML config file.
        :param: container_name Name of container that nodes will load in if use composition.
        :param: create_own_container Whether to start a ROS container when using composition.
        :param: namespace Top-level namespace.
        :param: use_composition Use composed bringup if True.
        :param: use_respawn Whether to respawn if a node crashes (when composition is disabled).
        :param: log_level Log level.
        :param: bridge_params Extra parameters to pass to the bridge.
        """
        super().__init__(**kwargs)

        self.__bridge_name = bridge_name
        self.__config_file = config_file
        self.__container_name = container_name
        self.__namespace = namespace

        # This is here to allow using strings or booleans as values for boolean variables when
        # the Action is used from Python i.e., this allows users to do:
        #   RosGzBridge(bridge_name='bridge1', use_composition='true', create_own_container=True)
        # Note that use_composition is set to a string while create_own_container is set to a
        # boolean. The reverse would also work.
        # At some point, we might want to deprecate this and only allow setting booleans since
        # that's what users would expect when calling this from Python
        if isinstance(create_own_container, str):
            self.__create_own_container = normalize_typed_substitution(
                TextSubstitution(text=create_own_container), bool
            )
        else:
            self.__create_own_container = normalize_typed_substitution(
                create_own_container, bool
            )

        if isinstance(use_composition, str):
            self.__use_composition = normalize_typed_substitution(
                TextSubstitution(text=use_composition), bool
            )
        else:
            self.__use_composition = normalize_typed_substitution(use_composition, bool)

        self.__use_respawn = normalize_typed_substitution(use_respawn, bool)
        self.__log_level = log_level
        self.__bridge_params = bridge_params

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse ros_gz_bridge."""
        kwargs: Dict = super().parse(entity, parser)[1]

        bridge_name = entity.get_attr(
            'bridge_name', data_type=str,
            optional=False)

        config_file = entity.get_attr(
            'config_file', data_type=str,
            optional=False)

        container_name = entity.get_attr(
            'container_name', data_type=str,
            optional=True)

        create_own_container = entity.get_attr(
            'create_own_container', data_type=str,
            optional=True)

        namespace = entity.get_attr(
            'namespace', data_type=str,
            optional=True)

        use_composition = entity.get_attr(
            'use_composition', data_type=str,
            optional=True)

        use_respawn = entity.get_attr(
            'use_respawn', data_type=str,
            optional=True)

        log_level = entity.get_attr(
            'log_level', data_type=str,
            optional=True)

        bridge_params = entity.get_attr(
            'bridge_params', data_type=str,
            optional=True)

        if isinstance(bridge_name, str):
            bridge_name = parser.parse_substitution(bridge_name)
            kwargs['bridge_name'] = bridge_name

        if isinstance(config_file, str):
            config_file = parser.parse_substitution(config_file)
            kwargs['config_file'] = config_file

        if isinstance(container_name, str):
            container_name = parser.parse_substitution(container_name)
            kwargs['container_name'] = container_name

        if isinstance(create_own_container, str):
            create_own_container = \
                parser.parse_substitution(create_own_container)
            kwargs['create_own_container'] = create_own_container

        if isinstance(namespace, str):
            namespace = parser.parse_substitution(namespace)
            kwargs['namespace'] = namespace

        if isinstance(use_composition, str):
            use_composition = parser.parse_substitution(use_composition)
            kwargs['use_composition'] = use_composition

        if isinstance(use_respawn, str):
            use_respawn = parser.parse_substitution(use_respawn)
            kwargs['use_respawn'] = use_respawn

        if isinstance(log_level, str):
            log_level = parser.parse_substitution(log_level)
            kwargs['log_level'] = log_level

        if isinstance(bridge_params, str):
            bridge_params = parser.parse_substitution(bridge_params)
            kwargs['bridge_params'] = bridge_params

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        if hasattr(self.__bridge_params, 'perform'):
            string_bridge_params = self.__bridge_params.perform(context)
        elif isinstance(self.__bridge_params, list):
            if hasattr(self.__bridge_params[0], 'perform'):
                string_bridge_params = self.__bridge_params[0].perform(context)
        else:
            string_bridge_params = str(self.__bridge_params)
        # Remove unnecessary symbols
        simplified_bridge_params = string_bridge_params.translate(
            {ord(i): None for i in '{} "\''}
        )
        # Parse to dictionary
        parsed_bridge_params = {}
        if simplified_bridge_params:
            bridge_params_pairs = simplified_bridge_params.split(',')
            parsed_bridge_params = dict(pair.split(':') for pair in bridge_params_pairs)

        use_composition_eval = perform_typed_substitution(
            context, self.__use_composition, bool
        )
        create_own_container_eval = perform_typed_substitution(
            context, self.__create_own_container, bool
        )

        launch_descriptions: List[Action] = []

        if not use_composition_eval:
            # Standard node configuration
            launch_descriptions.append(Node(
                package='ros_gz_bridge',
                executable='bridge_node',
                name=self.__bridge_name,
                namespace=self.__namespace,
                output='screen',
                respawn=perform_typed_substitution(context, self.__use_respawn, bool),
                respawn_delay=2.0,
                parameters=[{'config_file': self.__config_file, **parsed_bridge_params}],
                arguments=['--ros-args', '--log-level', self.__log_level],
                ))

        # Composable node with container configuration
        if use_composition_eval and create_own_container_eval:
            launch_descriptions.append(ComposableNodeContainer(
                name=self.__container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros_gz_bridge',
                        plugin='ros_gz_bridge::RosGzBridge',
                        name=self.__bridge_name,
                        namespace=self.__namespace,
                        parameters=[{'config_file': self.__config_file, **parsed_bridge_params}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                output='screen',
                ))

        # Composable node without container configuration
        if use_composition_eval and not create_own_container_eval:
            launch_descriptions.append(LoadComposableNodes(
                target_container=self.__container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros_gz_bridge',
                        plugin='ros_gz_bridge::RosGzBridge',
                        name=self.__bridge_name,
                        namespace=self.__namespace,
                        parameters=[{'config_file': self.__config_file, **parsed_bridge_params}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                ))

        return launch_descriptions
