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
from launch.actions import LogInfo, RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def read_params(ld : launch.LaunchDescription):
    gui = launch.substitutions.LaunchConfiguration('gui')
    server = launch.substitutions.LaunchConfiguration('server')
    rviz = launch.substitutions.LaunchConfiguration('rviz')
    environment = launch.substitutions.LaunchConfiguration('environment')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    simulator = launch.substitutions.LaunchConfiguration('simulator')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    world_name = launch.substitutions.LaunchConfiguration('world_name')
    world_path = launch.substitutions.LaunchConfiguration('world_path')
    cart = launch.substitutions.LaunchConfiguration('cart')
    connected = launch.substitutions.LaunchConfiguration('connected')
    x_pose = launch.substitutions.LaunchConfiguration('x_pose')
    y_pose = launch.substitutions.LaunchConfiguration('y_pose')
    z_pose = launch.substitutions.LaunchConfiguration('z_pose')
    kinematics = launch.substitutions.LaunchConfiguration('kinematics')
    controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')
    gazebo_debug_on = launch.substitutions.LaunchConfiguration('gazebo_debug_on')

    # Declare the launch options

    ld.add_action(DeclareLaunchArgument(
        name='gazebo_debug_on',
        description='Nivel de log para Gazebo (true or false)',
        default_value='false')
    )


    ld.add_action(DeclareLaunchArgument(
        name='gui',
        description='Launch Gazebo client (gui) if true',
        choices=['true', 'false'],
        default_value='true')
    )   
    
    ld.add_action(DeclareLaunchArgument(
        name='server',
        description='Launch Gazebo server if true',
        choices=['true', 'false'],
        default_value='true')
    )   
    
    ld.add_action(DeclareLaunchArgument(
        name='rviz',
        description='Launch Rviz if true',
        choices=['true', 'false'],
        default_value='false')
    )    

    ld.add_action(DeclareLaunchArgument(
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(DeclareLaunchArgument(
        name='simulator',
        description='Gazebo classic or ignition',
        choices=['classic', 'ignition'],
        default_value='classic')
    )

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the node.',
        default_value='robot')
    )

    ld.add_action(DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. robot).',
        default_value='robot')
    )

    ld.add_action(DeclareLaunchArgument(
        name='world_name',
        description='Name of the world to load.',
        default_value='grid_map.world')
        #default_value='robotnik_logo_min.world')
        #default_value='robotnik_logo_black.world')
        #default_value='intensity_simple.world')
    )

    ld.add_action(DeclareLaunchArgument(
        name='world_path',
        description='World to load path.',
        default_value=[get_package_share_directory('rbvogui_gazebo'), '/worlds/', world_name])
    )

    ld.add_action(DeclareLaunchArgument(
        name='cart',
        description='Bool to spawn the rbvogui with a cart',
        default_value='false')
    )

    ld.add_action(DeclareLaunchArgument(
        name='connected',
        description='Bool to connect the cart',
        default_value='false')
    )

    ld.add_action(DeclareLaunchArgument(
        name='x_pose',
        description='X position of the robot.',
        default_value='0.0')
    )

    ld.add_action(DeclareLaunchArgument(
        name='y_pose',
        description='Y position of the robot.',
        default_value='0.0')
    )

    ld.add_action(DeclareLaunchArgument(
        name='z_pose',
        description='Z position of the robot.',
        default_value='0.0')
    )

    ld.add_action(DeclareLaunchArgument(
        name='kinematics',
        description='kinematics of the robot (omni or ackermann)',
        default_value='omni')
    )

    ld.add_action(DeclareLaunchArgument(
            name='controllers_file',
            description='Absolute path to the controllers file.',
            default_value=[get_package_share_directory('rbvogui_gazebo'), '/config/', kinematics, '_controller.yaml'])
    )

    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'gui' : gui,
        'server' : server,
        'rviz' : rviz,
        'simulator' : simulator,
        'use_sim_time': use_sim_time,
        'namespace': namespace,
        'robot_id': robot_id,
        'world_path': world_path,
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

        ret['gui'] = gui
        ret['server'] = server
        ret['rviz'] = rviz

        if 'SIMULATOR' in os.environ:
            ret['simulator'] = os.environ['SIMULATOR']
        else: ret['simulator'] = simulator

        if 'USE_SIM_TIME' in os.environ:
            ret['use_sim_time'] = os.environ['USE_SIM_TIME']
        else: ret['use_sim_time'] = use_sim_time

        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        else:  ret['namespace'] = namespace

        if 'ROBOT_ID' in os.environ:
            ret['robot_id'] = os.environ['ROBOT_ID']
        else: ret['robot_id'] = robot_id

        if 'WORLD_PATH' in os.environ:
            ret['world_path'] = os.environ['WORLD']
        elif 'WORLD_NAME' in os.environ:
            ret['world_path'] = [get_package_share_directory('rbvogui_gazebo'), '/worlds/', os.environ['WORLD_NAME'], '.world']
        else: ret['world_path'] = world_path

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
        ret['gazebo_debug_on'] = gazebo_debug_on


    return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = launch.LaunchDescription()

    description_dir = os.path.join(get_package_share_directory('rbvogui_description'), 'launch')

    params = read_params(ld)

    ### Gazebo classic

    gazebo_classic_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rbvogui_gazebo'), 'launch', 'gazebo_classic_sim.launch.py')]
        ),
        launch_arguments={
            'gui':params['gui'],
            'server':params['server'],
            'world_path': params['world_path'],
            'gazebo_debug_on': params['gazebo_debug_on'],
        }.items(),
        condition=IfCondition(
        PythonExpression(["'", LaunchConfiguration('simulator'), "' == 'classic'"])
        )
    )

    ### Gazebo Ignition

    gazebo_ignition_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rbvogui_gazebo'), 'launch', 'gazebo_ign_sim.launch.py')]
        ),
        launch_arguments={
            'gui':params['gui'],
            'server':params['server'],
            'world_path': params['world_path'],
        }.items(),
        condition=IfCondition(
        PythonExpression(["'", LaunchConfiguration('simulator'), "' == 'ignition'"])
        )
    )

    ### Robot State publisher

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'simulator': params['simulator'],
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

    cart_group = launch.actions.GroupAction(
        actions = [
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', "cart",
                    '-topic', "robot_description",
                    '-x', '-2',
                    '-y', '0.5',
                    '-z', '0.5',
                ],
                output='screen',
                namespace=[params['namespace'], '/cart'],
            ),
            IncludeLaunchDescription(
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

    ### Joint state broadcaster

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
        namespace=params['namespace']
    )
    
    ### Robotnik base Controller

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotnik_base_controller", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
        namespace=params['namespace']
    )
    
    rviz_launch = launch.actions.GroupAction(
        actions = [
            Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=['-d' + os.path.join(get_package_share_directory('rbvogui_gazebo'), 'rviz', 'default.rviz')]
            )
        ],
        condition = IfCondition(launch.substitutions.LaunchConfiguration('rviz'))
    )    

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(base_controller_spawner)
    ld.add_action(rviz_launch)
    ld.add_action(gazebo_ignition_launch)
    ld.add_action(gazebo_classic_launch)    
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