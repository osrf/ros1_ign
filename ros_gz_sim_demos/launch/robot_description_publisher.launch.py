# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Import the model urdf (load from file, xacro ...)
    robot_desc = \
        '<?xml version="1.0" ?>'\
        '<robot name="will_be_ignored">'\
        '<link name="link">'\
        '<visual>'\
        '<geometry>'\
        '<sphere radius="1.0"/>'\
        '</geometry>'\
        '</visual>'\
        '<collision>'\
        '<geometry>'\
        '<sphere radius="1.0"/>'\
        '</geometry>'\
        '</collision>'\
        '<inertial>'\
        '<mass value="1"/>'\
        '<inertia ixx="1" ixy="0.0" ixz="0.0" iyy="1" iyz="0.0" izz="1"/>'\
        '</inertial>'\
        '</link>'\
        '</robot>'

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_desc}
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # RViz
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'robot_description_publisher.rviz')
        ]
    )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
<<<<<<< HEAD
                 arguments=[
                    '-name', 'my_custom_model',
                    '-x', '1.2',
                    '-z', '2.3',
                    '-Y', '3.4',
                    '-topic', '/robot_description'],
=======
                 parameters=[{
                    'name': 'my_custom_model',
                    'x': 1.2,
                    'z': 2.3,
                    'Y': 3.4,
                    'topic': '/robot_description'}],
>>>>>>> 63b651a (Garden EOL (#662))
                 output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz,
        spawn
    ])
