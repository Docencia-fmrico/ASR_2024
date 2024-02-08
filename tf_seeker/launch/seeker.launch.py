# Copyright 2024 Intelligent Robotics Lab
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
    tf_seeker_dir = get_package_share_directory('tf_seeker')

    kobuki_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki'),
            'launch',
            'simulation.launch.py')),
        launch_arguments={'world': tf_seeker_dir + '/worlds/empty.world'}.items()
        )

    tf_publisher_cmd = Node(
        package='tf_seeker',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ])

    seeker_cmd = Node(
        package='tf_seeker',
        executable='seeker',
        name='seeker',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ])

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['-d', [os.path.join(tf_seeker_dir, 'config', 'seeker.rviz')]],
        )

    ld = LaunchDescription()

    ld.add_action(tf_publisher_cmd)
    ld.add_action(seeker_cmd)
    ld.add_action(rviz2_cmd)
    ld.add_action(kobuki_cmd)

    return ld
