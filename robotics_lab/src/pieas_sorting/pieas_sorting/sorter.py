"""Finite-state sorting task. Every robot move is planned and checked by MoveIt.

The simulator plugin supplies only machine interlocks and attachment/drop status.
Object class, pickup XYZ and size come exclusively from RGBD + LiDAR perception.
"""
import copy
import csv
import json
import math
import os
from pathlib import Path
import runpy
import time
import uuid
import traceback
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, PlanningScene,
                             PlanningSceneComponents, AllowedCollisionEntry, RobotState, Constraints, JointConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock,ClockType
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool,String
from .kinematics import Kinematics,JOINTS,canonical_positions
from .world import load_config,boxes
from .task_logic import validate_detection,DropVerifier

# The existing, tested ROS action/service client is still our execution backend.
_script=Path(get_package_prefix('pieas_arm_moveit_config'))/'lib/pieas_arm_moveit_config/tool_sequence'
ToolSequence=runpy.run_path(str(_script))['ToolSequence']


class Sorter(ToolSequence):
    def __init__(self):
        super().__init__()
        self.declare_parameter('config',str(Path(get_package_share_directory('pieas_sorting'))/'config/cell.yaml'))
        self.declare_parameter('cycles',6)
        self.declare_parameter('log_directory',str(Path.home()/'.ros/pieas_sorting'))
        self.c=load_config(self.get_parameter('config').value)
        self.machine=None;self.machine_wall=0.;self.detection=None;self.detection_wall=0.
        self.expect_payload=False;self.phase='STARTUP';self.last_sim=None;self.reset_detected=False;self.executing=False
        self.commands=self.create_publisher(String,'/factory/command',10)
        self.heartbeats=self.create_publisher(Bool,'/factory/heartbeat',10)
        self.status=self.create_publisher(String,'/factory/task_state',10)
        self.create_subscription(String,'/factory/state',self.machine_callback,10)
        self.create_subscription(String,'/factory/detections',self.detection_callback,10)
        self.create_timer(0.2,lambda:self.heartbeats.publish(Bool(data=True)),clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.placed={name:0 for name in self.c['bins']}
        self.scene_object=None;self.records=[]

    def machine_callback(self,message):
        value=json.loads(message.data)
        if self.last_sim is not None and value['sim_time']<self.last_sim:
            self.phase='FAULT_RESET';self.reset_detected=True
        self.last_sim=value['sim_time'];self.machine=value;self.machine_wall=time.monotonic()

    def detection_callback(self,message):
        self.detection=json.loads(message.data);self.detection_wall=time.monotonic()

    def phase_to(self,name):
        self.phase=name;self.get_logger().info(f'FACTORY: {name}')
        self.status.publish(String(data=json.dumps({'phase':name,'placed':self.placed})))

    def spin_until(self,predicate,timeout,description):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.05)
            if self.reset_detected:raise RuntimeError('World reset: stop this task and relaunch the factory.')
            if predicate():return
        raise RuntimeError('Timeout: '+description)

    def wait(self,future,seconds=30):
        if self.executing and seconds==180:seconds=self.execution_timeout
        deadline=time.monotonic()+seconds
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.05)
            if self.reset_detected:raise RuntimeError('Gazebo reset during task; relaunch required.')
            if self.executing:
                if time.monotonic()-self.machine_wall>3 or time.monotonic()-self.received>3:
                    raise RuntimeError('Execution feedback stopped; cancellation requested.')
                if self.expect_payload and self.machine and not self.machine['held']:
                    raise RuntimeError('Suction attachment lost during execution.')
        if not future.done():raise RuntimeError('ROS request timed out.')
        return future.result()

    def execute_leg(self,trajectory):
        self.executing=True
        duration=trajectory.joint_trajectory.points[-1].time_from_start
        self.execution_timeout=max(180,5*(duration.sec+duration.nanosec*1e-9)+30)
        try:return super().execute_leg(trajectory)
        finally:self.executing=False

    def command(self,command,timeout=5):
        token=uuid.uuid4().hex
        def acknowledged():
            if self.machine and self.machine['ack']==token:
                if self.machine['error']:raise RuntimeError('Factory command failed: '+self.machine['error'])
                return True
            self.commands.publish(String(data=f'{token}|{command}'))
            return False
        self.spin_until(acknowledged,timeout,'factory '+command)

    def scene(self,objects=(),attached=(),matrix=None):
        scene=PlanningScene(is_diff=True)
        scene.robot_state.is_diff=True
        scene.world.collision_objects=list(objects)
        scene.robot_state.attached_collision_objects=list(attached)
        if matrix is not None:scene.allowed_collision_matrix=matrix
        response=self.service(ApplyPlanningScene,'/apply_planning_scene',ApplyPlanningScene.Request(scene=scene))
        if not response.success:raise RuntimeError('MoveIt refused factory scene update.')

    def allow_contact(self,a,b,allowed):
        query=GetPlanningScene.Request()
        query.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=self.service(GetPlanningScene,'/get_planning_scene',query).scene.allowed_collision_matrix
        for name in [a,b]:
            if name not in matrix.entry_names:
                for row in matrix.entry_values:row.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        i,j=matrix.entry_names.index(a),matrix.entry_names.index(b)
        matrix.entry_values[i].enabled[j]=allowed;matrix.entry_values[j].enabled[i]=allowed
        self.scene(matrix=matrix)

    @staticmethod
    def collision(name,xyz,size,yaw=0.,shape='box'):
        obj=CollisionObject(id=name,operation=CollisionObject.ADD)
        obj.header.frame_id='world'
        primitive=SolidPrimitive()
        if shape=='cylinder':
            primitive.type=SolidPrimitive.CYLINDER
            primitive.dimensions=[size[2],max(size[:2])/2]
        else:
            primitive.type=SolidPrimitive.BOX;primitive.dimensions=list(size)
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,xyz)
        pose.orientation.z=math.sin(yaw/2);pose.orientation.w=math.cos(yaw/2)
        obj.primitives=[primitive];obj.primitive_poses=[pose]
        return obj

    def measured(self):
        self.check_measured_state()
        state=copy.deepcopy(self.state)
        state.position=canonical_positions(state.name,state.position,self.kinematics.continuous)
        return RobotState(joint_state=state,is_diff=True)

    def move(self,xyz,label):
        self.phase_to(label)
        start=self.measured()
        q=np.array(self.positions(start))
        chosen=None
        # IK only answers where the joints should end. Each candidate must also
        # pass MoveIt's collision check, including the carried object when held.
        for solution in self.kinematics.candidates(xyz,q):
            sample=copy.deepcopy(start)
            values=dict(zip(sample.joint_state.name,sample.joint_state.position))
            values.update(zip(JOINTS,solution.tolist()))
            sample.joint_state.position=[values[name] for name in sample.joint_state.name]
            request=GetStateValidity.Request(robot_state=sample,group_name='arm')
            if self.service(GetStateValidity,'/check_state_validity',request).valid:
                chosen=solution;break
        if chosen is None:raise RuntimeError(f'No bounded, upright, collision-free IK at {xyz}.')
        self.get_logger().info(f'IK {label}: rail metres, then radians = {np.round(chosen,5).tolist()}')
        # Restrict the entire path, not just the IK endpoint, to the usable rail.
        envelope=Constraints(name='factory_rail_envelope')
        envelope.joint_constraints=[JointConstraint(joint_name='joint_1',position=0.,
            tolerance_above=self.c['motion']['rail_limit'],
            tolerance_below=self.c['motion']['rail_limit'],weight=1.)]
        plan=self.plan_targets(chosen.tolist(),start_state=start,plan_only=True,precise=True,
                               path_constraints=envelope)
        rail_index=plan.joint_trajectory.joint_names.index('joint_1')
        if any(abs(point.positions[rail_index])>self.c['motion']['rail_limit']+1e-6 for point in plan.joint_trajectory.points):
            raise RuntimeError('Planned rail path exceeds the factory operating envelope.')
        endpoint=self.predicted_end(plan,start)
        error=self.distance(self.xyz(self.forward(endpoint)),xyz)
        if error>self.c['motion']['endpoint_tolerance']:raise RuntimeError('Planned endpoint outside tolerance.')
        # Planning can take time; never execute if the machine stopped reporting.
        rclpy.spin_once(self,timeout_sec=0.05)
        if time.monotonic()-self.machine_wall>1.:raise RuntimeError('Stale machine state before execution.')
        current=self.measured()
        delta=np.array(self.positions(current))-q
        delta[-1]=math.atan2(math.sin(delta[-1]),math.cos(delta[-1]))
        if np.any(np.abs(delta)>np.array([.002,.01,.01,.01,.01,.01])):raise RuntimeError('Robot moved during planning.')
        self.execute_leg(plan)
        actual=self.measured()
        pose=self.forward(actual)
        error=self.distance(self.xyz(pose),xyz)
        if error>self.c['motion']['endpoint_tolerance']:raise RuntimeError('Measured endpoint outside tolerance.')
        transform=self.kinematics.forward(self.positions(actual))
        if transform[2,2]<math.cos(self.c['motion']['upright_tolerance']):raise RuntimeError('Tool not upright at endpoint.')
        if self.expect_payload and (not self.machine or not self.machine['held']):raise RuntimeError('Payload attachment lost.')
        self.get_logger().info(f'{label}: measured-state FK endpoint error {error*1000:.2f} mm')
        return error

    def wait_detection(self):
        selected=[]
        def fresh():
            if not self.machine or self.machine['running'] or not self.machine['at_station']:return False
            if not self.detection or time.monotonic()-self.detection_wall>0.6:return False
            values=self.detection['detections']
            age=self.get_clock().now().nanoseconds*1e-9-self.detection['stamp']
            if len(values)!=1 or not 0<=age<0.5:return False
            validate_detection(values[0],self.c)
            selected.append(copy.deepcopy(values[0]));return True
        self.spin_until(fresh,20,'one stationary camera detection corroborated by LiDAR')
        return selected[0]

    def attach_scene(self,obj):
        # Keep this object's real dimensions in MoveIt during transport. Only the
        # suction link may touch it; other arm links remain collision checked.
        attached=AttachedCollisionObject(link_name='rg2_link',object=copy.deepcopy(obj),touch_links=['rg2_link'])
        # MoveIt attaches first and automatically removes the world object.
        # A second explicit world REMOVE in this same diff would fail.
        self.scene(attached=[attached])
        self.verify_attachment(obj.id,True)

    def verify_attachment(self,name,expected):
        query=GetPlanningScene.Request()
        query.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        response=self.service(GetPlanningScene,'/get_planning_scene',query)
        present=any(item.object.id==name for item in response.scene.robot_state.attached_collision_objects)
        if present!=expected:raise RuntimeError('MoveIt payload attachment state disagrees with the task.')

    def detach_scene(self,obj,xyz,remove_attachment=True):
        detached=AttachedCollisionObject(link_name='rg2_link')
        detached.object.id=obj.id;detached.object.operation=CollisionObject.REMOVE
        placed=copy.deepcopy(obj)
        p=placed.primitive_poses[0]
        p.position.x,p.position.y,p.position.z=map(float,xyz)
        # The dropped object's yaw is not measured by the camera. Use a square
        # bounding footprint so collision coverage does not depend on guessed yaw.
        if placed.primitives[0].type==SolidPrimitive.BOX:
            d=placed.primitives[0].dimensions
            extent=math.hypot(d[0],d[1]);placed.primitives[0].dimensions=[extent,extent,d[2]]
        p.orientation.x=p.orientation.y=p.orientation.z=0.;p.orientation.w=1.
        self.scene(objects=[placed],attached=[detached] if remove_attachment else [])
        self.verify_attachment(obj.id,False)

    def install_cell_scene(self):
        self.setup_scene()
        self.scene(objects=[self.collision(name,xyz,size) for name,xyz,size,_ in boxes(self.c)])
        b=self.c['belt']
        # A conservative planning-only exclusion volume covers queued parts.
        # The active pick part is outside this volume and has measured geometry.
        self.scene(objects=[self.collision('queued_parts_keepout',
            [b['x'],(b['y_min']-0.06)/2,b['top']+0.035],
            [b['width'], -0.06-b['y_min'],0.07])])

    def run(self):
        if self.get_parameter('setup_only').value:
            self.install_cell_scene();return
        if not self.get_parameter('execute').value:
            self.get_logger().info('Factory starts idle. Use execute:=true for the automatic sorting cycle.');return
        count=self.get_parameter('cycles').value
        if not 1<=count<=min(self.c['motion']['max_cycles'],self.c['parts']['count']):raise RuntimeError('cycles must be 1..6.')
        self.spin_until(lambda:self.machine is not None,30,'Gazebo factory plugin and bridge')
        if self.machine.get('removed',0)>0:raise RuntimeError('This batch already started; relaunch the factory before a new task.')
        if self.machine['held']:raise RuntimeError('A payload is already attached; relaunch the factory for a clean batch.')
        self.command('stop')
        response=self.service(GetParameters,'/robot_state_publisher/get_parameters',
                              GetParameters.Request(names=['robot_description']))
        self.kinematics=Kinematics(response.values[0].string_value,self.c['motion']['rail_limit'])
        self.install_cell_scene()
        baseline=self.measured()
        q=self.positions(baseline)
        if self.distance(self.kinematics.forward(q)[:3,3],self.xyz(self.forward(baseline)))>1e-5:
            raise RuntimeError('Local URDF kinematics and MoveIt disagree.')
        clearance=self.c['motion']['clearance_z']
        # Park upstream of the inspection footprint: sensors must see the object.
        park=[0.24,0.02,clearance]
        self.move(park,'PARK_FOR_INSPECTION')
        for cycle in range(count):
            started=time.monotonic();self.phase_to('INDEX_BELT');self.command('run')
            self.spin_until(lambda:self.machine and self.machine['at_station'] and not self.machine['running'],
                            90,'belt indexing at inspection station')
            self.phase_to('DETECT_AND_CLASSIFY')
            detected=self.wait_detection()
            shape=detected['shape'];index=self.placed[shape]
            if index>=2:raise RuntimeError('Bin capacity reached; empty the bins before restarting.')
            self.get_logger().info(f"Camera class={shape}, XYZ={detected['xyz']}, LiDAR support={detected['lidar_points']}")
            obj=self.collision(f'sorted_{cycle}',detected['xyz'],detected['size'],detected['yaw'],
                               'cylinder' if shape=='cylinder' else 'box')
            self.scene_object=obj;self.scene(objects=[obj])
            # Only the intended target/suction contact is allowed for the pick.
            self.allow_contact(obj.id,'rg2_link',True)
            pickup=[detected['xyz'][0],detected['xyz'][1],detected['top_z']+0.001]
            self.move([pickup[0],pickup[1],clearance],'APPROACH_PICK')
            self.move(pickup,'DESCEND_TO_PICK')
            self.phase_to('ATTACH_SUCTION');self.command('grip')
            self.spin_until(lambda:self.machine['held'],3,'suction acknowledgement')
            self.expect_payload=True
            self.allow_contact(obj.id,'belt_deck',True)
            self.attach_scene(obj)
            self.move([pickup[0],pickup[1],clearance],'LIFT_PAYLOAD')
            self.allow_contact(obj.id,'belt_deck',False)
            bx,by,_=self.c['bins'][shape]
            bx+=(-0.028 if index==0 else 0.028)
            self.move([bx,by,clearance],'TRANSFER_TO_'+shape.upper())
            release=[bx,by,self.c['motion']['release_top_z']]
            self.move(release,'LOWER_TO_BIN')
            # Remove attached payload from planning state only after Gazebo accepts
            # release. The released object then falls under gravity into the bin.
            self.phase_to('RELEASE');self.command('release');self.expect_payload=False
            self.detach_scene(obj,[bx,by,release[2]-detected['size'][2]/2])
            self.move([bx,by,clearance],'RETRACT')
            settled=[]
            verifier=DropVerifier(self.c['bins'][shape],self.c['bin_inner'],self.c['bin_floor'],
                                  self.c['bin_height'],detected['size'],shape)
            def inside_bin():
                if self.machine is None or time.monotonic()-self.machine_wall>0.5:return False
                xyz=self.machine.get('release_xyz')
                if not verifier.update(xyz,self.machine['sim_time']):return False
                settled.append(list(xyz));return True
            self.spin_until(inside_bin,8,'released object inside the intended bin')
            self.detach_scene(obj,settled[-1],remove_attachment=False)
            # Pickup contact permission ends once the tool is clear and placement
            # is verified. Later paths must avoid this part with every robot link.
            self.allow_contact(obj.id,'rg2_link',False)
            self.placed[shape]+=1
            self.records.append({'cycle':cycle+1,'shape':shape,'confidence':detected['confidence'],
                                 'lidar_points':detected['lidar_points'],'wall_seconds':time.monotonic()-started,
                                 'outcome':'verified_in_bin'})
            self.write_log()
            self.move(park,'PARK_FOR_INSPECTION')
        self.command('stop');self.phase_to('COMPLETE')

    def write_log(self):
        directory=Path(self.get_parameter('log_directory').value).expanduser()
        directory.mkdir(parents=True,exist_ok=True)
        if not hasattr(self,'log_path'):self.log_path=directory/f'batch_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        if not self.records:return
        temporary=self.log_path.with_suffix('.tmp')
        with temporary.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(self.records[0]));writer.writeheader();writer.writerows(self.records)
        os.replace(temporary,self.log_path)
        self.get_logger().info('Batch record: '+str(self.log_path))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None;code=0
    try:
        node=Sorter();node.run()
    except (Exception,KeyboardInterrupt) as error:
        code=1
        if not isinstance(error,KeyboardInterrupt):traceback.print_exc()
        if node is not None:
            node.get_logger().error(str(error) or 'Interrupted')
            node.phase_to('FAULT')
            try:node.cancel()
            except Exception as failure:node.get_logger().error('Cancel unconfirmed: '+str(failure))
            try:node.command('stop',2)
            except Exception:pass
            # Keep an attached payload attached on failure; do not drop it blindly.
        else:print('Factory initialization failed:',error)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
    return code
