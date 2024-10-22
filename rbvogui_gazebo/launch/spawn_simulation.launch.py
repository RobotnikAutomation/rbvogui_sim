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
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    world_name = launch.substitutions.LaunchConfiguration('world_name')
    world = launch.substitutions.LaunchConfiguration('world')
    cart = launch.substitutions.LaunchConfiguration('cart')
    connected = launch.substitutions.LaunchConfiguration('connected')
    x_pose = launch.substitutions.LaunchConfiguration('x_pose')
    y_pose = launch.substitutions.LaunchConfiguration('y_pose')
    z_pose = launch.substitutions.LaunchConfiguration('z_pose')
    kinematics = launch.substitutions.LaunchConfiguration('kinematics')
    controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')

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
        description='Bool to spawn the rbvogui with a cart',
        default_value='false')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='connected',
        description='Bool to connect the cart',
        default_value='false')
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

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='kinematics',
        description='kinematics of the robot (omni or ackermann)',
        default_value='omni')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
            name='controllers_file',
            description='Absolute path to the controllers file.',
            default_value=[get_package_share_directory('rbvogui_gazebo'), '/config/', kinematics, '_controller.yaml'])
    )

    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'world': world,
        'world_name': world_name,
        'cart': cart,
        'connected': connected,
        'x_pose': x_pose,
        'y_pose': y_pose,
        'z_pose': z_pose,
        'kinematics': kinematics,
        'controllers_file': controllers_file
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

        if 'CONNECTED' in os.environ:
            ret['connected'] = os.environ['CONNECTED']
        else:  ret['connected'] = connected

        ret['world_name']=world_name
        ret['x_pose']=x_pose
        ret['y_pose']=y_pose
        ret['z_pose']=z_pose
        ret['kinematics']=kinematics
        ret['controllers_file']=controllers_file

    return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = launch.LaunchDescription()
    gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    description_dir = os.path.join(get_package_share_directory('rbvogui_description'), 'launch')

    params = read_params(ld)

    gazebo_launch_group = launch.actions.GroupAction(
        actions=[
            launch_ros.actions.PushRosNamespace(namespace=params['namespace']),
            launch.actions.IncludeLaunchDescription(
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
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_dir, 'gzclient.launch.py')
                ),
                launch_arguments={
                    'verbose': 'false',
                }.items(),
            )
        ]
    )

    robot_state_publisher_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': params['use_sim_time'],
            'robot_id': params['robot_id'],
            'cart': params['cart'],
            'connected': params['connected'],
            'namespace': params['namespace'],
            'kinematics': params['kinematics'],
            'launch_joint': 'false',
            'controllers_file': params['controllers_file'],
        }.items(),
    )

    rbvogui_gazebo_ros_spawner_cmd = launch_ros.actions.Node(
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
        namespace=params['namespace']
    )

    cart_group = launch.actions.GroupAction(
        actions = [
            launch_ros.actions.Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', "cart",
                    '-topic', "robot_description",
                    '-x', '-2',
                    '-y', '0.5',
                    '-z', '0.5',
                ],
                output='screen',
                namespace=[params['namespace'], '/cart'],
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('rbvogui_description'), 'launch/cart_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': params['use_sim_time'],
                    'launch_joint': 'false',
                    'connected': params['connected'],
                    'namespace': [params['namespace'],'/cart'],
                    'robot_id': [params['robot_id'],'_cart'],
                }.items(),
            )
        ],
        condition = launch.conditions.IfCondition(
                    launch.substitutions.AndSubstitution(launch.substitutions.NotSubstitution(
                        params['connected']),params['cart'])
                )
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
        namespace=params['namespace']
    )

    base_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotnik_base_controller", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
        namespace=params['namespace']
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('rbvogui_gazebo'), 'rviz', 'default.rviz')]
    )

    ld.add_action(gazebo_launch_group)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rbvogui_gazebo_ros_spawner_cmd)
    ld.add_action(cart_group)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(base_controller_spawner)
    ld.add_action(rviz)
    ld.add_action(
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=base_controller_spawner,
                on_exit=[
                    launch.actions.LogInfo(msg='Spawn finished'),
                ]
            )
        ),
    )

    return ld