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

import os
from typing import Dict, List, Optional, Union

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from launch.action import Action
from launch.actions import SetEnvironmentVariable
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import TextSubstitution
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ros2pkg.api import get_package_names


"""
Search for model, plugin and media paths exported by packages.

e.g.  <export>
          <gazebo_ros gazebo_model_path="${prefix}/../"/>
          <gazebo_ros gazebo_media_path="${prefix}/../"/>
      </export>
${prefix} is replaced by package's share directory in install.

Thus the required directory needs to be installed from CMakeLists.txt
e.g.  install(DIRECTORY models
          DESTINATION share/${PROJECT_NAME})
"""


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


@expose_action('gz_server')
class GzServer(Action):
    """Action that executes a gz_server ROS [composable] node."""

    def __init__(
        self,
        *,
        world_sdf_file: SomeSubstitutionsType = '',
        world_sdf_string: SomeSubstitutionsType = '',
        container_name: SomeSubstitutionsType = 'ros_gz_container',
        create_own_container: Union[bool, SomeSubstitutionsType] = False,
        use_composition: Union[bool, SomeSubstitutionsType] = False,
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

        # This is here to allow using strings or booleans as values for boolean variables when
        # the Action is used from Python See the RosGzBridge.__init__ function for more details.
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

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse gz_server."""
        kwargs: Dict = super().parse(entity, parser)[1]

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
        launch_descriptions: List[Action] = []

        model_paths, plugin_paths = GazeboRosPaths.get_paths()
        launch_descriptions.append(SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.pathsep.join([
                os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                os.environ.get('LD_LIBRARY_PATH', default=''),
                plugin_paths,
            ])))
        launch_descriptions.append(SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([
                os.environ.get('GZ_SIM_RESOURCE_PATH', default=''),
                model_paths,
            ])))

        use_composition_eval = perform_typed_substitution(
            context, self.__use_composition, bool
        )
        create_own_container_eval = perform_typed_substitution(
            context, self.__create_own_container, bool
        )
        if not use_composition_eval:
            # Standard node configuration
            launch_descriptions.append(Node(
                package='ros_gz_sim',
                executable='gzserver',
                output='screen',
                parameters=[{'world_sdf_file': self.__world_sdf_file,
                             'world_sdf_string': self.__world_sdf_string}],
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
                        package='ros_gz_sim',
                        plugin='ros_gz_sim::GzServer',
                        name='gz_server',
                        parameters=[{'world_sdf_file': self.__world_sdf_file,
                                     'world_sdf_string': self.__world_sdf_string}],
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
                        package='ros_gz_sim',
                        plugin='ros_gz_sim::GzServer',
                        name='gz_server',
                        parameters=[{'world_sdf_file': self.__world_sdf_file,
                                     'world_sdf_string': self.__world_sdf_string}],
                        extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                ))

        return launch_descriptions
