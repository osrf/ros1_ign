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

"""Module for the gz_spawn_model action."""

from typing import List
from typing import Optional

from launch.action import Action
from launch.actions import IncludeLaunchDescription
from launch.frontend import Entity, expose_action, Parser
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


@expose_action('gz_spawn_model')
class GzSpawnModel(Action):
    """Action that executes a gz_spawn_model ROS node."""

    def __init__(
        self,
        *,
        world: Optional[SomeSubstitutionsType] = None,
        file: Optional[SomeSubstitutionsType] = None,
        model_string: Optional[SomeSubstitutionsType] = None,
        topic: Optional[SomeSubstitutionsType] = None,
        entity_name: Optional[SomeSubstitutionsType] = None,
        allow_renaming: Optional[SomeSubstitutionsType] = None,
        x: Optional[SomeSubstitutionsType] = None,
        y: Optional[SomeSubstitutionsType] = None,
        z: Optional[SomeSubstitutionsType] = None,
        roll: Optional[SomeSubstitutionsType] = None,
        pitch: Optional[SomeSubstitutionsType] = None,
        yaw: Optional[SomeSubstitutionsType] = None,
        **kwargs
    ) -> None:
        """
        Construct a gz_spawn_model action.

        All arguments are forwarded to `ros_gz_sim.launch.gz_spawn_model.launch.py`,
        so see the documentation of that class for further details.

        :param: world World name.
        :param: file SDF filename.
        :param: model_string XML(SDF) string.
        :param: topic Get XML from this topic.
        :param: entity_name Name of the entity.
        :param: allow_renaming Whether the entity allows renaming or not.
        :param: x X coordinate.
        :param: y Y coordinate.
        :param: z Z coordinate.
        :param: roll Roll orientation.
        :param: pitch Pitch orientation.
        :param: yaw Yaw orientation.
        """
        super().__init__(**kwargs)
        self.__world = world
        self.__file = file
        self.__model_string = model_string
        self.__topic = topic
        self.__entity_name = entity_name
        self.__allow_renaming = allow_renaming
        self.__x = x
        self.__y = y
        self.__z = z
        self.__roll = roll
        self.__pitch = pitch
        self.__yaw = yaw

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse gz_spawn_model."""
        _, kwargs = super().parse(entity, parser)

        world = entity.get_attr(
            'world', data_type=str,
            optional=True)

        file = entity.get_attr(
            'file', data_type=str,
            optional=True)

        model_string = entity.get_attr(
            'model_string', data_type=str,
            optional=True)

        topic = entity.get_attr(
            'topic', data_type=str,
            optional=True)

        entity_name = entity.get_attr(
            'entity_name', data_type=str,
            optional=True)

        allow_renaming = entity.get_attr(
            'allow_renaming', data_type=str,
            optional=True)

        x = entity.get_attr(
            'x', data_type=str,
            optional=True)

        y = entity.get_attr(
            'y', data_type=str,
            optional=True)

        z = entity.get_attr(
            'z', data_type=str,
            optional=True)

        roll = entity.get_attr(
            'roll', data_type=str,
            optional=True)

        pitch = entity.get_attr(
            'pitch', data_type=str,
            optional=True)

        yaw = entity.get_attr(
            'yaw', data_type=str,
            optional=True)

        if isinstance(world, str):
            world = parser.parse_substitution(world)
            kwargs['world'] = world

        if isinstance(file, str):
            file = parser.parse_substitution(file)
            kwargs['file'] = file

        if isinstance(model_string, str):
            model_string = parser.parse_substitution(model_string)
            kwargs['model_string'] = model_string

        if isinstance(topic, str):
            topic = parser.parse_substitution(topic)
            kwargs['topic'] = topic

        if isinstance(entity_name, str):
            entity_name = parser.parse_substitution(entity_name)
            kwargs['entity_name'] = entity_name

        if isinstance(allow_renaming, str):
            allow_renaming = parser.parse_substitution(allow_renaming)
            kwargs['allow_renaming'] = allow_renaming

        if isinstance(x, str):
            x = parser.parse_substitution(x)
            kwargs['x'] = x

        if isinstance(y, str):
            y = parser.parse_substitution(y)
            kwargs['y'] = y

        if isinstance(z, str):
            z = parser.parse_substitution(z)
            kwargs['z'] = z

        if isinstance(roll, str):
            roll = parser.parse_substitution(roll)
            kwargs['roll'] = roll

        if isinstance(pitch, str):
            pitch = parser.parse_substitution(pitch)
            kwargs['pitch'] = pitch

        if isinstance(yaw, str):
            yaw = parser.parse_substitution(yaw)
            kwargs['yaw'] = yaw

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        gz_spawn_model_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_spawn_model.launch.py'])]),
            launch_arguments=[('world', self.__world),
                              ('file', self.__file),
                              ('model_string',   self.__model_string),
                              ('topic',  self.__topic),
                              ('entity_name', self.__entity_name),
                              ('allow_renaming', self.__allow_renaming),
                              ('x',   self.__x),
                              ('y',  self.__y),
                              ('z', self.__z),
                              ('roll', self.__roll),
                              ('pitch',   self.__pitch),
                              ('yaw',  self.__yaw), ])

        return [gz_spawn_model_description]
