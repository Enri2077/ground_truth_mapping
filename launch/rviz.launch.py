# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import Node


def generate_launch_description():

    declare_use_remappings_cmd = DeclareLaunchArgument(
        'use_remappings', default_value='false',
        description='Arguments to pass to all nodes launched by the file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        description='Full path to the RVIZ config file to use')

    start_rviz_cmd = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        output='log',

        use_remappings=IfCondition(LaunchConfiguration('use_remappings')),
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_remappings_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)

    return ld
