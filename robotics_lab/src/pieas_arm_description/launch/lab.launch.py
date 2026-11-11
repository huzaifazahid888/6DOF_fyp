from pathlib import Path
import os
import shlex
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def prepare_world(package):
    # Both ROS and Gazebo use this description; no separate hand-edited SDF robot.
    robot = ET.parse(package / 'urdf/pieas_control.urdf.in').getroot()
    for parameter in robot.findall('./gazebo/plugin/parameters'):
        if parameter.text == '@CONTROLLERS_FILE@':
            parameter.text = str(package / 'config/controllers.yaml')
    for mesh in robot.iter('mesh'):
        prefix = 'package://pieas_arm_description/'
        filename = mesh.get('filename', '')
        if filename.startswith(prefix):
            mesh.set('filename', (package / filename[len(prefix):]).resolve().as_uri())
    description = ET.tostring(robot, encoding='unicode')
    directory = Path(tempfile.mkdtemp(prefix='pieas_arm_lab_'))
    try:
        urdf = directory / 'arm.urdf'
        urdf.write_text(description)
        result = subprocess.run(
            ['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            raise RuntimeError('URDF to SDF conversion failed:\n' + result.stderr)
        try:
            converted = ET.fromstring(result.stdout)
        except ET.ParseError as error:
            raise RuntimeError('gz sdf returned invalid XML:\n' + result.stderr) from error
        model = converted.find('model')
        if model is None:
            raise RuntimeError('URDF conversion did not produce an SDF model.')
        expected = {f'joint_{index}' for index in range(1, 7)}
        actual = {joint.get('name') for joint in model.findall('joint')}
        if not expected.issubset(actual):
            raise RuntimeError('URDF conversion lost arm joints: ' + str(expected - actual))
        if not any('gz_ros2_control' in plugin.get('filename', '')
                   for plugin in model.findall('plugin')):
            raise RuntimeError('URDF conversion lost the Gazebo control plugin.')
        model.set('name', 'pieas_arm')
        world_tree = ET.parse(package / 'worlds/lab.sdf')
        world = world_tree.getroot().find('world')
        if world is None or world.get('name') != 'lab':
            raise RuntimeError('Expected a world named lab.')
        if world.find("model[@name='pieas_arm']") is not None:
            raise RuntimeError('The world already contains pieas_arm.')
        world_tree.getroot().set('version', converted.get('version', '1.9'))
        world.append(model)
        world_path = directory / 'lab_with_arm.sdf'
        world_tree.write(world_path, encoding='utf-8', xml_declaration=True)
        return description, world_path, directory
    except Exception:
        shutil.rmtree(directory, ignore_errors=True)
        raise


def generate_launch_description():
    package = Path(get_package_share_directory('pieas_arm_description'))
    description, world_path, directory = prepare_world(package)
    resources = str(package.parent) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gazebo = Path(get_package_share_directory('ros_gz_sim'))
    actions = [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: shutil.rmtree(directory, ignore_errors=True))),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r ' + shlex.quote(str(world_path))}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
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
