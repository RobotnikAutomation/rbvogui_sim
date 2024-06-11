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

from ament_index_python.packages import get_package_share_directory

def read_params(ld : launch.LaunchDescription):
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    cart = launch.substitutions.LaunchConfiguration('cart')
    connected = launch.substitutions.LaunchConfiguration('connected')
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
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='false')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the node.',
        default_value='cart')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. robot).',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='cart',
        description='bool rbvogui with cart',
        default_value='false')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='x_pose',
        description='X position of the robot.',
        default_value='-5')
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

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='connected',
        description='bool if cart is connected',
        default_value='false')
    )
    # Parse the launch options
    ret = {}

    ret = {
    'use_sim_time': use_sim_time,
    'namespace': namespace,
    'robot_id': robot_id,
    'cart': cart,
    'connected': connected,
    'x_pose': x_pose,
    'y_pose': y_pose,
    'z_pose': z_pose,
    }

    return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = launch.LaunchDescription()

    params = read_params(ld)

    cart_state_publisher_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rbvogui_description'), 'launch/cart_state_publisher.launch.py')
        ),
        launch_arguments={
            'launch_joint': 'false',
            'connected': params['connected']
        }.items()
    )

    start_gazebo_ros_spawner_cmd = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', "cart",
            '-topic', "robot_description",
            '-x', '-2',
            '-y', params['y_pose'],
            '-z', params['z_pose'],
        ],
        output='screen',
    )

    ld.add_action(launch_ros.actions.PushRosNamespace(namespace='cart'))
    ld.add_action(cart_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld