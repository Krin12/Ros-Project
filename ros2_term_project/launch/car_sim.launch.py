#!/usr/bin/env python3
#
# Copyright 2023. Prof. Jong Min Lee @ Dong-eui University
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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import json

print(os.path.realpath(__file__))

ld = LaunchDescription()

def generate_launch_description():
    # configuration
    world = LaunchConfiguration('world')
    print('world =', world)
    world_file_name = 'car_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'), 'worlds', world_file_name)
    print('world file name = %s' % world)

    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    gazebo_model_path = os.path.join(get_package_share_directory('ros2_term_project'), 'models')
    gazebo_env = {'GAZEBO_MODEL_PATH': gazebo_model_path, 'LD_LIBRARY_PATH': os.environ['LD_LIBRARY_PATH']}
    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
        additional_env=gazebo_env
    )

    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)

    prius_hybrid_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
        '-entity', 'prius_hybrid',
        '-file', '/home/ros2/AutoCar/Ros-Project/ros2_term_project/models/prius_hybrid/model.sdf'
        
    ],
    output='screen',
    )

    ld.add_action(prius_hybrid_spawn)

    return ld

