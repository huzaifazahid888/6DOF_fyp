"""Start the existing Gazebo lab plus MoveIt. No motion is commanded on launch."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('pieas_arm_moveit_config'))
    description_package = Path(get_package_share_directory('pieas_arm_description'))
    # Read the same control URDF as Gazebo, preserving geometry and joint frames.
    robot = ET.parse(description_package / 'urdf/pieas_control.urdf.in').getroot()
    for parameter in robot.findall('./gazebo/plugin/parameters'):
        parameter.text = str(description_package / 'config/controllers.yaml')
    description = ET.tostring(robot, encoding='unicode')

    def config(name):
        return yaml.safe_load((package / 'config' / name).read_text())

    common = {
        'use_sim_time': True,
        'robot_description': description,
        'robot_description_semantic': (package / 'config/pieas_arm.srdf').read_text(),
        'robot_description_planning': config('joint_limits.yaml'),
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': config('ompl_planning.yaml'),
    }
    execution = {
        **config('moveit_controllers.yaml'),
        'allow_trajectory_execution': True,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 3.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(description_package / 'launch/lab.launch.py'))),
        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[common, execution]),
        # Populate the planning scene with the floor. This node never moves joints.
        Node(package='pieas_arm_moveit_config', executable='plan_joint_goal',
             name='pieas_scene_setup', output='screen',
             parameters=[{'use_sim_time': True, 'setup_only': True}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(package / 'config/planning.rviz')],
             parameters=[common], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
