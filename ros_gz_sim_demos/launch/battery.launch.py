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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

    # RQt
    rqt = Node(
        package='rqt_plot',
        executable='rqt_plot',
        # FIXME: Why isn't the topic being populated on the UI? RQt issue?
        arguments=['--force-discover',
                   '/model/vehicle_blue/battery/linear_battery/state/percentage'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -z 1000000 linear_battery_demo.sdf'
        }.items(),
    )

    # Bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_ros_gz_sim_demos, 'config', 'battery.yaml'),
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rqt', default_value='true',
                              description='Open RQt.'),
        ros_gz_bridge,
        rqt
    ])
