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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def read_params(ld : launch.LaunchDescription):
    gui = launch.substitutions.LaunchConfiguration('gui')
    server = launch.substitutions.LaunchConfiguration('server')
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

    # Declare the launch options
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
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(DeclareLaunchArgument(
        name='simulator',
        description='Gazebo classic or ignition',
        choices=['classic', 'ignition'],
        default_value='ignition')
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
        default_value='demo.sdf.world')
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
        default_value='0.5')
    )

    ld.add_action(DeclareLaunchArgument(
        name='y_pose',
        description='Y position of the robot.',
        default_value='0.5')
    )

    ld.add_action(DeclareLaunchArgument(
        name='z_pose',
        description='Z position of the robot.',
        default_value='0.5')
    )

    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'gui' : gui,
        'server' : server,
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
        }
    
    else:

        ret['gui'] = gui
        ret['server'] = server        

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
            ret['world_path'] = os.environ['WORLD_PATH']
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

    return ret


def generate_launch_description():

    ld = launch.LaunchDescription()

    params = read_params(ld)

    gazebo_ignition_launch_group = launch.actions.GroupAction(
        actions=[
            launch_ros.actions.PushRosNamespace(namespace=params['namespace']),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ), 
                            'launch'
                        ),
                        'gz_sim.launch.py')
                ),
                launch_arguments={                    
                    'gz_args':[
                        '-r ',
                        '-s ',
                        '-v4 ',
                        params['world_path']
                    ], 
                    'on_exit_shutdown':'true'
                }.items(),
                condition = IfCondition(LaunchConfiguration('server'))
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ), 
                            'launch'
                        ), 
                        'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args':[
                        '-g '
                    ], 
                    'on_exit_shutdown':'true'
                }.items(),
                condition = IfCondition(launch.substitutions.LaunchConfiguration('gui'))
            )
        ]
    )

    rbvogui_ignition_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "rbvogui",
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

    bridge_params = os.path.join(get_package_share_directory('rbvogui_gazebo'),'config','gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/robot/front_rgbd_camera/color/image_raw", 
            "/robot/rear_rgbd_camera/color/image_raw"
            #"/robot/front_rgbd_camera/ired1/image_raw", 
            #"/robot/rear_rgbd_camera/ired1/image_raw",
            #"/robot/front_rgbd_camera/ired2/image_raw", 
            #"/robot/rear_rgbd_camera/ired2/image_raw",
            #"/robot/front_rgbd_camera/depth/image_raw",
            #"/robot/rear_rgbd_camera/depth/image_raw"
        ]
    )
        
    ld.add_action(ros_gz_bridge)
    ld.add_action(ros_gz_image_bridge)
    ld.add_action(gazebo_ignition_launch_group)
    ld.add_action(rbvogui_ignition_ros_spawner_cmd)

    return ld

