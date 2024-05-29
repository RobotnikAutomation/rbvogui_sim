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
    environment = launch.substitutions.LaunchConfiguration('environment')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    world_name = launch.substitutions.LaunchConfiguration('world_name')
    world = launch.substitutions.LaunchConfiguration('world')
    cart = launch.substitutions.LaunchConfiguration('cart')

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
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the node.',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. robot).',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='world_name',
        description='Name of the world to load.',
        default_value='demo')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='world',
        description='World to load path.',
        default_value=[get_package_share_directory('rbvogui_gazebo'), '/worlds/', world_name, '.world'])
    )
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='cart',
        description='bool rbvogui with cart',
        default_value='false')
    )
    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'world': world,
        'cart': cart
        }
    
    else:
        if 'USE_SIM_TIME' in os.environ:
            ret['use_sim_time'] = os.environ['USE_SIM_TIME']
        else: ret['use_sim_time'] = use_sim_time

        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        else:  ret['namespace'] = namespace

        if 'ROBOT_ID' in os.environ:
            ret['robot_id'] = os.environ['ROBOT_ID']
        else: ret['robot_id'] = robot_id

        if 'WORLD' in os.environ:
            ret['world'] = os.environ['WORLD']
        elif 'WORLD_NAME' in os.environ:
            ret['world'] = [get_package_share_directory('rbvogui_gazebo'), '/worlds/', os.environ['WORLD_NAME'], '.world']
        else: ret['world'] = world

        if 'CART' in os.environ:
            ret['cart'] = os.environ['CART']
        else:  ret['cart'] = cart

    return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = launch.LaunchDescription()
    launch_dir = os.path.join(get_package_share_directory('rbvogui_gazebo'), 'launch')
    gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    params = read_params(ld)

    gzserver_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'world': params['world'],
            'paused': 'false',
            'physics': 'ode',
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
            'params_file': [get_package_share_directory('rbvogui_gazebo'), '/config/gazebo.yaml'],
        }.items(),
    )

    gzclient_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': params['use_sim_time'],
            'robot_id': params['robot_id'],
            'cart': params['cart']
        }.items(),
    )

    spawn_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': params['use_sim_time'],
            'x_pose': '0.5',
            'y_pose': '0.5',
            'z_pose': '0.5',
        }.items()
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
    )

    base_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotnik_base_controller", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
    )

    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(base_controller_spawner)

    return ld