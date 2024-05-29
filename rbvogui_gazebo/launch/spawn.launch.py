# Copyright (c) 2022, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import launch
import launch_ros
import os
import tempfile

from ament_index_python.packages import get_package_share_directory

#from robotnik_common.launch import RewrittenYaml

# Environment variables
# CONFIG_FILE: Path to override the default configuration file (e.g. /home/user/config.yaml)
# PORT:        Device port (e.g. /dev/ttyUSB0)
# BAUD:        Device baud rate (e.g. 115200)
# NAME:        Name of the node.
# NAMESPACE:   Namespace of the node.
# FRAME_ID:    Frame id of the sensor. (e.g. vectornav_link)

def read_params(ld : launch.LaunchDescription):
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    description_pkg = launch.substitutions.LaunchConfiguration('description_pkg')
    description_name = launch.substitutions.LaunchConfiguration('description_name')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    x_pose = launch.substitutions.LaunchConfiguration('x_pose')
    y_pose = launch.substitutions.LaunchConfiguration('y_pose')
    z_pose = launch.substitutions.LaunchConfiguration('z_pose')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='description_pkg',
        description='Description package of the robot.',
        default_value='rbvogui_description')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='description_name',
        description='Description package of the robot.',
        default_value='default.urdf.xacro')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. vectornav_link).',
        default_value='vectornav_link')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='world',
        description='World to load.',
        default_value=os.path.join(get_package_share_directory('rbvogui_gazebo'), 'worlds', 'demo.world'))
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='x_pose',
        description='X position of the robot.',
        default_value='0.5')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='y_pose',
        description='Y position of the robot.',
        default_value='0.5')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='z_pose',
        description='Z position of the robot.',
        default_value='0.5')
    )

    # Parse the launch options
    return {
        'use_sim_time': use_sim_time,
        'robot_description': os.path.join(get_package_share_directory('rbvogui_description'), 'robot', 'test.urdf.xacro'),
        'robot_id': robot_id,
        'x_pose': x_pose,
        'y_pose': y_pose,
        'z_pose': z_pose,
    }


def generate_launch_description():
    ld = launch.LaunchDescription()

    params = read_params(ld)

    start_gazebo_ros_spawner_cmd = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', "rbvogui",
            '-topic', "robot_description",
            '-x', params['x_pose'],
            '-y', params['y_pose'],
            '-z', params['z_pose'],
        ],
        output='screen',
    )

    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
