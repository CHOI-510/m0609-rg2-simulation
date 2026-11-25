from runpy import run_path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def launch_moveit(context):
    with_rg2 = IfCondition(LaunchConfiguration('rg2')).evaluate(context)
    with_pick = IfCondition(LaunchConfiguration('pick')).evaluate(context)
    if with_pick and not with_rg2:
        raise ValueError('pick:=true requires rg2:=true')
    mappings = {'rg2': str(with_rg2).lower()}
    package = get_package_share_path('m0609_gz')
    config = package / 'config'
    doosan = get_package_share_path('dsr_moveit_config_m0609')

    def load_yaml(path):
        with path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    kinematics = load_yaml(doosan / 'config' / 'kinematics.yaml')
    joint_limits = load_yaml(doosan / 'config' / 'joint_limits.yaml')
    joint_limits.update(
        default_velocity_scaling_factor=0.1,
        default_acceleration_scaling_factor=0.1,
    )
    controllers = load_yaml(config / 'moveit_controllers.yaml')
    ompl = load_yaml(doosan / 'config' / 'ompl_planning.yaml')
    if with_rg2:
        controller_manager = controllers['moveit_simple_controller_manager']
        controller_manager['controller_names'].append('rg2_controller')
        controller_manager['rg2_controller'] = {
            'action_ns': 'follow_joint_trajectory',
            'type': 'FollowJointTrajectory',
            'default': True,
            'joints': ['rg2_finger_joint'],
        }
        joint_limits['joint_limits']['rg2_finger_joint'] = {
            'has_velocity_limits': True,
            'max_velocity': 1.0,
            'has_acceleration_limits': True,
            'max_acceleration': 2.0,
        }
        ompl['gripper'] = {
            'default_planner_config': 'RRTConnect',
            'planner_configs': ['RRTConnect'],
            'longest_valid_segment_fraction': 0.005,
        }
    robot_description = xacro.process_file(
        str(package / 'urdf' / 'm0609.urdf.xacro'), mappings=mappings
    ).toxml()
    if with_pick:
        pick_robot = run_path(str(config / 'pick_robot.py'))
        robot_description = pick_robot['with_pick_pads'](robot_description)
    robot_parameters = {
        'use_sim_time': True,
        'robot_description': robot_description,
        'robot_description_semantic': xacro.process_file(
            str(config / 'm0609.srdf.xacro'), mappings=mappings
        ).toxml(),
        'robot_description_kinematics': {'manipulator': kinematics['manipulator']},
        'robot_description_planning': joint_limits,
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_parameters,
            controllers,
            {
                'allow_trajectory_execution': True,
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(doosan / 'launch' / 'moveit.rviz')],
        parameters=[robot_parameters],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rg2', default_value='false', choices=['true', 'false'],
            description='Plan with the RG2 gripper and use rg2_tcp as the arm tip',
        ),
        DeclareLaunchArgument('pick', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_moveit),
    ])
