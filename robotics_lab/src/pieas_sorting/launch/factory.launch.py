"""Factory + real sensors + MoveIt. Auto motion is deliberately opt-in."""
import os
from pathlib import Path
import runpy
import shlex
import shutil
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from pieas_sorting.world import load_config,build_world,robot_variant


def generate_launch_description():
    package=Path(get_package_share_directory('pieas_sorting'))
    description_package=Path(get_package_share_directory('pieas_arm_description'))
    moveit=Path(get_package_share_directory('pieas_arm_moveit_config'))
    config_path=package/'config/cell.yaml';c=load_config(config_path)
    directory=Path(tempfile.mkdtemp(prefix='pieas_factory_'))
    (directory/'cell.sdf').write_text(build_world(c))
    (directory/'suction.urdf').write_text(robot_variant(description_package/'urdf/pieas_control.urdf.in'))
    prepare=runpy.run_path(str(description_package/'launch/lab.launch.py'))['prepare_world']
    try:
        description,world_path,generated=prepare(description_package,directory/'cell.sdf',directory/'suction.urdf')
    except Exception:
        shutil.rmtree(directory,ignore_errors=True);raise
    def clean(event,context):
        shutil.rmtree(directory,ignore_errors=True);shutil.rmtree(generated,ignore_errors=True)
    def cfg(name):return yaml.safe_load((moveit/'config'/name).read_text())
    common={'use_sim_time':True,'robot_description':description,
            'robot_description_semantic':(moveit/'config/pieas_arm.srdf').read_text(),
            'robot_description_kinematics':cfg('kinematics.yaml'),
            'robot_description_planning':cfg('joint_limits.yaml'),
            'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':cfg('ompl_planning.yaml')}
    execution={**cfg('moveit_controllers.yaml'),'allow_trajectory_execution':True,'moveit_manage_controllers':False,
               'trajectory_execution.allowed_execution_duration_scaling':1.5,
               'trajectory_execution.allowed_goal_duration_margin':3.0,
               'trajectory_execution.allowed_start_tolerance':0.01,
               'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,
               'publish_transforms_updates':True,'publish_robot_description':True,'publish_robot_description_semantic':True}
    # Add sensor displays to the familiar RViz configuration.
    rviz=yaml.safe_load((moveit/'config/planning.rviz').read_text())
    rviz['Visualization Manager']['Displays'] += [
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Detected shapes','Enabled':True,
         'Topic':{'Value':'/factory/detection_markers','Reliability Policy':'Reliable','Durability Policy':'Volatile'}},
        {'Class':'rviz_default_plugins/PointCloud2','Name':'LiDAR','Enabled':False,
         'Topic':{'Value':'/factory/lidar/points','Reliability Policy':'Best Effort','Durability Policy':'Volatile'},
         'Size (m)':0.003,'Style':'Points'},
        {'Class':'rviz_default_plugins/Image','Name':'Inspection camera','Enabled':True,
         'Topic':{'Value':'/factory/camera/image','Reliability Policy':'Best Effort','Durability Policy':'Volatile'}}]
    (directory/'factory.rviz').write_text(yaml.safe_dump(rviz))
    params={'use_sim_time':True,'config':str(config_path)}
    actions=[DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('start_sorting',default_value='false'),
             DeclareLaunchArgument('gui',default_value='true'),
             RegisterEventHandler(OnShutdown(on_shutdown=clean)),
             SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(description_package.parent)+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
             IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
                launch_arguments={'gz_args':[PythonExpression(["'' if '",LaunchConfiguration('gui'),"' == 'true' else '-s --headless-rendering '"]),'-r '+shlex.quote(str(world_path))], 'on_exit_shutdown':'true'}.items()),
             Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':description}]),
             Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'use_sim_time':True,'config_file':str(package/'config/bridge.yaml')}],output='screen'),
             Node(package='moveit_ros_move_group',executable='move_group',parameters=[common,execution],output='screen'),
             Node(package='pieas_sorting',executable='sort_batch',name='factory_scene_setup',parameters=[params,{'setup_only':True}],output='screen'),
             Node(package='pieas_sorting',executable='perception',parameters=[params],output='screen'),
             Node(package='rviz2',executable='rviz2',arguments=['-d',str(directory/'factory.rviz')],parameters=[common],condition=IfCondition(LaunchConfiguration('rviz'))),
             Node(package='pieas_sorting',executable='sort_batch',parameters=[params,{'execute':True}],output='screen',condition=IfCondition(LaunchConfiguration('start_sorting')))]
    for controller in ['joint_state_broadcaster','arm_controller']:
        actions.append(Node(package='controller_manager',executable='spawner',arguments=[controller,'--controller-manager-timeout','120'],output='screen'))
    for key in ['camera','lidar']:
        values=c[key]['xyz']+c[key]['rpy'];args=[]
        for flag,value in zip(['--x','--y','--z','--roll','--pitch','--yaw'],values):args.extend([flag,str(value)])
        args+=['--frame-id','world','--child-frame-id',f'factory_{key}']
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',arguments=args,parameters=[{'use_sim_time':True}]))
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',arguments=[
        '--roll','-1.5707963267948966','--yaw','-1.5707963267948966',
        '--frame-id','factory_camera','--child-frame-id','factory_camera_optical'],parameters=[{'use_sim_time':True}]))
    return LaunchDescription(actions)
