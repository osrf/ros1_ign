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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_gz_sim_demos, 'models', 'rrbot.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Gazebo Sim
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    # )

    # Gzserver
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'ros_gz_server.launch.py'])]),
        launch_arguments=[('world_sdf_file', 'empty.sdf'),
                          ('create_own_container', str(True)),
                          ('use_composition', str(True)),
                          ('bridge_name', 'ros_gz_bridge'),
                          ('config_file', os.path.join(pkg_ros_gz_sim_demos, 'config', 'joint_states.yaml')), ])

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'joint_states.rviz')],
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'rrbot',
                    'topic': 'robot_description'}],
        output='screen',
    )

    # Gz - ROS Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         # Clock (IGN -> ROS2)
    #         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    #         # Joint states (IGN -> ROS2)
    #         '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    #     ],
    #     remappings=[
    #         ('/world/empty/model/rrbot/joint_state', 'joint_states'),
    #     ],
    #     output='screen'
    # )

    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            spawn,
            # bridge,
            robot_state_publisher,
            rviz,
        ]
    )
