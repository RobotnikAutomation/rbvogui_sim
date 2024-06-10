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

#from robotnik_common.launch import RewrittenYaml

# Environment variables
#  USE_SIM_TIME: Use simulation (Gazebo) clock if true
#  NAMESPACE: Namespace of the node stack.
#  ROBOT_ID: Frame id of the robot. (e.g. vectornav_link).
#  WORLD: World to load.

def read_params(ld : launch.LaunchDescription):
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    world_name = launch.substitutions.LaunchConfiguration('world_name')
    world = launch.substitutions.LaunchConfiguration('world')
    cart = launch.substitutions.LaunchConfiguration('cart')
    connected = launch.substitutions.LaunchConfiguration('connected')
    namespace = launch.substitutions.LaunchConfiguration('namespace')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='world_name',
        description='Name of the world to load.',
        default_value='demo')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the node.',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='world',
        description='World to load path.',
        default_value=[get_package_share_directory('rbvogui_gazebo'), '/worlds/', world_name, '.world'])
    )
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='cart',
        description='bool rbvogui with cart',
        default_value='true')
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
    'world': world,
    'cart': cart,
    'connected': connected,
    'world_name': world_name,
    'namespace': namespace,
    }

    return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = launch.LaunchDescription()
    default_launch_dir = os.path.join(get_package_share_directory('rbvogui_gazebo'), 'launch')
    nav_launch_dir = os.path.join(get_package_share_directory('rbvogui_navigation'), 'launch')
    docking_launch_dir = os.path.join(get_package_share_directory('rbvogui_docking'), 'launch')

    params = read_params(ld)

    default_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(default_launch_dir, 'default.launch.py')
        ),
        launch_arguments={
            'cart': params['cart'],
            'connected': params['connected'],
            'world': params['world'],
            'use_sim_time': params['use_sim_time'],
        }.items(),
    )

    trailer_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(default_launch_dir, 'trailer.launch.py')
        ),
        launch_arguments={
            'cart': params['cart'],
            'connected': params['connected']
        }.items(),
        condition = launch.conditions.IfCondition(
            launch.substitutions.AndSubstitution(
                launch.substitutions.NotSubstitution(params['connected']),
                params['cart']
            ))
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["--ros-args", "--log-level", "FATAL"],
        )

    ld.add_action(default_launch)
    ld.add_action(trailer_launch)
    ld.add_action(rviz)

    return ld