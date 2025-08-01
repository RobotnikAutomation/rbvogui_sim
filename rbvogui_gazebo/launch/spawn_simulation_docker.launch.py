import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    # Argumentos
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('server', default_value='true'))
    ld.add_action(DeclareLaunchArgument('simulator', default_value='classic'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value='robot'))
    ld.add_action(DeclareLaunchArgument('robot_id', default_value='robot'))
    ld.add_action(DeclareLaunchArgument('world_name', default_value='grid_map.world'))
    ld.add_action(DeclareLaunchArgument('world_path', default_value=[get_package_share_directory('rbvogui_gazebo'), '/worlds/', LaunchConfiguration('world_name')]))
    ld.add_action(DeclareLaunchArgument('cart', default_value='false'))
    ld.add_action(DeclareLaunchArgument('connected', default_value='false'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('kinematics', default_value='omni'))
    ld.add_action(DeclareLaunchArgument('controllers_file', default_value=[get_package_share_directory('rbvogui_gazebo'), '/config/', LaunchConfiguration('kinematics'), '_controller.yaml']))
    ld.add_action(DeclareLaunchArgument('gazebo_debug_on', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('wait_spawn_time', default_value='5.0', description='Tiempo de espera (segundos) antes de lanzar los nodos ROS tras el spawn'))

    description_dir = os.path.join(get_package_share_directory('rbvogui_description'), 'launch')

    # Lanzar gzserver (servidor Gazebo)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': LaunchConfiguration('gazebo_debug_on'),
            'world': LaunchConfiguration('world_path'),
            'paused': 'false',
            'physics': 'ode',
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
            'params_file': [get_package_share_directory('rbvogui_gazebo'), '/config/gazebo_classic.yaml'],
        }.items(),
        condition=IfCondition(LaunchConfiguration('server'))
    )
    ld.add_action(gzserver)

    # Lanzar gzclient (GUI de Gazebo)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(gzclient)

    # Spawnea la entidad robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rbvogui',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose')
        ],
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )
    ld.add_action(spawn_entity)

    # Resto de nodos (robot_state_publisher, controladores, ...)
    late_launch_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'simulator': LaunchConfiguration('simulator'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_id': LaunchConfiguration('robot_id'),
                'cart': LaunchConfiguration('cart'),
                'connected': LaunchConfiguration('connected'),
                'namespace': LaunchConfiguration('namespace'),
                'kinematics': LaunchConfiguration('kinematics'),
                'launch_joint': 'false',
                'controllers_file': LaunchConfiguration('controllers_file'),
            }.items(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", ["/", LaunchConfiguration('namespace'), "/controller_manager"]],
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotnik_base_controller", "--controller-manager", ["/", LaunchConfiguration('namespace'), "/controller_manager"]],
            namespace=LaunchConfiguration('namespace')
        ),
    ]

    # ¡Aquí el tiempo de espera es parametrizable!
    ld.add_action(TimerAction(
        period=LaunchConfiguration('wait_spawn_time'),
        actions=late_launch_actions
    ))

    # Lanzar RViz opcional
    ld.add_action(
        GroupAction(
            actions=[
                Node(
                    package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d' + os.path.join(get_package_share_directory('rbvogui_gazebo'), 'rviz', 'default.rviz')]
                )
            ],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    )

    return ld
