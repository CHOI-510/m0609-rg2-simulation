from runpy import run_path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_simulation(context):
    with_rg2 = IfCondition(LaunchConfiguration('rg2')).evaluate(context)
    with_pick = IfCondition(LaunchConfiguration('pick')).evaluate(context)
    if with_pick and not with_rg2:
        raise ValueError('pick:=true requires rg2:=true')
    description_share = get_package_share_path('dsr_description2')
    sim_share = get_package_share_path('ros_gz_sim')
    model = get_package_share_path('m0609_gz') / 'urdf' / 'm0609.urdf.xacro'
    robot_description = xacro.process_file(
        str(model), mappings={'rg2': str(with_rg2).lower()}
    ).toxml()
    world_name = 'pick' if with_pick else 'empty'
    world_file = str(model.parents[1] / 'worlds' / 'pick.sdf') if with_pick else 'empty.sdf'
    if with_pick:
        pick_robot = run_path(str(model.parents[1] / 'config' / 'pick_robot.py'))
        robot_description = pick_robot['with_pick_pads'](robot_description)
    controller_names = ['joint_state_broadcaster', 'joint_trajectory_controller']
    resource_paths = [description_share.parent]
    if with_rg2:
        controller_names.append('rg2_controller')
        resource_paths.append(get_package_share_path('onrobot_rg_description').parent)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [LaunchConfiguration('gz_flags'), f' -r "{world_file}"'],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            *([
                '/model/cube/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/pick/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            ] if with_pick else []),
        ],
        remappings=[(f'/world/{world_name}/clock', '/clock')],
        output='screen',
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', world_name, '-name', 'm0609', '-topic', '/robot_description'],
        output='screen',
    )
    controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *controller_names,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Failed to spawn M0609'))]
        return [controllers]

    def after_controllers(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Failed to activate controllers'))]
        return []

    return [
        *(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(path)) for path in resource_paths),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(
            OnProcessExit(target_action=controllers, on_exit=after_controllers)
        ),
        gazebo,
        state_publisher,
        clock_bridge,
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_flags', default_value='-s', description='Gazebo flags'),
        DeclareLaunchArgument(
            'rg2', default_value='false', choices=['true', 'false'],
            description='Attach and control the RG2 gripper',
        ),
        DeclareLaunchArgument('pick', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_simulation),
    ])
