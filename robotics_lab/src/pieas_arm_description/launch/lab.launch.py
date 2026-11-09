from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('pieas_arm_description'))
    description = (package / 'urdf/pieas_control.urdf.in').read_text()
    description = description.replace(
        '@CONTROLLERS_FILE@', str(package / 'config/controllers.yaml'))
    resources = str(package.parent) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gazebo = Path(get_package_share_directory('ros_gz_sim'))
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r ' + str(package / 'worlds/lab.sdf')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'lab', '-name', 'pieas_arm', '-string', description],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(package / 'config/bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
    ]
    for controller in ['joint_state_broadcaster', 'arm_controller']:
        actions.append(Node(
            package='controller_manager', executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'], output='screen'))
    frames = [
        ('world', 'camera_link', ['0', '0', '1', '0', '1.57079632679', '0']),
        ('camera_link', 'camera_optical_frame', ['0', '0', '0', '-1.57079632679', '0', '-1.57079632679']),
        ('world', 'lidar_link', ['-0.6', '0', '0.65', '0', '0', '0']),
    ]
    for parent, child, pose in frames:
        args = []
        for flag, value in zip(['--x', '--y', '--z', '--roll', '--pitch', '--yaw'], pose):
            args.extend([flag, value])
        args.extend(['--frame-id', parent, '--child-frame-id', child])
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            arguments=args, parameters=[{'use_sim_time': True}]))
    return LaunchDescription(actions)
