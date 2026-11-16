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
from moveit_msgs.msg import PositionConstraint, BoundingVolume, OrientationConstraint
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, PlanningScene,
                             PlanningSceneComponents, AllowedCollisionEntry, RobotState, Constraints, JointConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock,ClockType
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool,String
from pieas_sorting.kinematics import Kinematics,JOINTS,canonical_positions
from .world import load_config,boxes
from pieas_sorting.task_logic import DropVerifier

# The existing, tested ROS action/service client is still our execution backend.
_script=Path(get_package_prefix('pieas_arm_moveit_config'))/'lib/pieas_arm_moveit_config/tool_sequence'
ToolSequence=runpy.run_path(str(_script))['ToolSequence']


class Harvester(ToolSequence):
    def __init__(self):
        super().__init__()
        self.declare_parameter('config',str(Path(get_package_share_directory('pieas_orchard'))/'config/orchard.yaml'))
        self.declare_parameter('cycles',3)
        self.declare_parameter('log_directory',str(Path.home()/'.ros/pieas_orchard'))
        self.c=load_config(self.get_parameter('config').value)
        self.machine=None;self.machine_wall=0.;self.detection=None;self.detection_wall=0.
        self.expect_payload=False;self.phase='STARTUP';self.last_sim=None;self.reset_detected=False;self.executing=False
        self.commands=self.create_publisher(String,'/orchard/command',10)
        self.heartbeats=self.create_publisher(Bool,'/orchard/heartbeat',10)
        self.status=self.create_publisher(String,'/orchard/task_state',10)
        self.create_subscription(String,'/orchard/state',self.machine_callback,10)
        self.create_subscription(String,'/orchard/detections',self.detection_callback,10)
        self.create_timer(0.2,lambda:self.heartbeats.publish(Bool(data=True)),clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.placed={name:0 for name in self.c['bins']}
        self.scene_object=None;self.records=[];self.corridor=None;self.harvested=[]

    def machine_callback(self,message):
        value=json.loads(message.data)
        if self.last_sim is not None and value['sim_time']<self.last_sim:
            self.phase='FAULT_RESET';self.reset_detected=True
        self.last_sim=value['sim_time'];self.machine=value;self.machine_wall=time.monotonic()
        if value.get('tree_contact') or value.get('reset_fault'):
            self.reset_detected=True

    def detection_callback(self,message):
        self.detection=json.loads(message.data);self.detection_wall=time.monotonic()

    def phase_to(self,name):
        self.phase=name;self.get_logger().info(f'ORCHARD: {name}')
        self.status.publish(String(data=json.dumps({'phase':name,'placed':self.placed})))

    def spin_until(self,predicate,timeout,description):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.05)
            if self.reset_detected:raise RuntimeError('World reset: stop this task and relaunch the orchard.')
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
        self.spin_until(acknowledged,timeout,'orchard '+command)

    def scene(self,objects=(),attached=(),matrix=None):
        scene=PlanningScene(is_diff=True)
        scene.robot_state.is_diff=True
        scene.world.collision_objects=list(objects)
        scene.robot_state.attached_collision_objects=list(attached)
        if matrix is not None:scene.allowed_collision_matrix=matrix
        response=self.service(ApplyPlanningScene,'/apply_planning_scene',ApplyPlanningScene.Request(scene=scene))
        if not response.success:raise RuntimeError('MoveIt refused orchard scene update.')

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
        if shape=='sphere':
            primitive.type=SolidPrimitive.SPHERE;primitive.dimensions=[size[0]]
        elif shape=='cylinder':
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
        envelope=Constraints(name='orchard_rail_envelope')
        envelope.joint_constraints=[JointConstraint(joint_name='joint_1',position=0.,
            tolerance_above=self.c['motion']['rail_limit'],
            tolerance_below=self.c['motion']['rail_limit'],weight=1.)]
        if self.corridor is not None:
            low,high=self.corridor
            centre=(np.array(low)+np.array(high))/2
            size=np.abs(np.array(high)-low)+np.array([.008,.008,.008])
            region=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size.tolist())
            pose=Pose();pose.position.x,pose.position.y,pose.position.z=centre.tolist();pose.orientation.w=1.
            constraint=PositionConstraint(link_name='tool0',weight=1.)
            constraint.header.frame_id='world';constraint.constraint_region=BoundingVolume(primitives=[region],primitive_poses=[pose])
            envelope.position_constraints=[constraint]
            # Keep the suction axis upright while allowing wrist yaw.
            orientation=OrientationConstraint(link_name='tool0',weight=1.)
            orientation.header.frame_id='world';orientation.orientation=self.forward(start).pose.orientation
            orientation.absolute_x_axis_tolerance=.06;orientation.absolute_y_axis_tolerance=.06;orientation.absolute_z_axis_tolerance=math.pi
            envelope.orientation_constraints=[orientation]
        plan=self.plan_targets(chosen.tolist(),start_state=start,plan_only=True,precise=True,
                               path_constraints=envelope)
        rail_index=plan.joint_trajectory.joint_names.index('joint_1')
        if any(abs(point.positions[rail_index])>self.c['motion']['rail_limit']+1e-6 for point in plan.joint_trajectory.points):
            raise RuntimeError('Planned rail path exceeds the orchard operating envelope.')
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
        if error>self.c['motion']['endpoint_tolerance']:
            raise RuntimeError(f'Measured endpoint error {error*1000:.2f} mm at {label}; target={xyz}, actual={self.xyz(pose)}')
        transform=self.kinematics.forward(self.positions(actual))
        if transform[2,2]<math.cos(self.c['motion']['upright_tolerance']):raise RuntimeError('Tool not upright at endpoint.')
        if self.expect_payload and (not self.machine or not self.machine['held']):raise RuntimeError('Payload attachment lost.')
        self.get_logger().info(f'{label}: measured-state FK endpoint error {error*1000:.2f} mm')
        return error

    def wait_detection(self):
        accepted=[]
        def fresh():
            if not self.detection or time.monotonic()-self.detection_wall>.6:return False
            age=self.get_clock().now().nanoseconds*1e-9-self.detection['stamp']
            if not 0<=age<.5:return False
            fruits=self.detection['detections']
            fruits=[f for f in fruits if all(np.linalg.norm(np.array(f['xyz'])-old)>.025 for old in self.harvested)]
            if not fruits:return False
            accepted.extend(copy.deepcopy(fruits));return True
        self.spin_until(fresh,30,'stable visible fruit with RGB-D and LiDAR evidence')
        return accepted

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
        margin=self.c['motion']['branch_margin']
        objects=[]
        for name,xyz,size,_ in boxes(self.c):
            padding=2*margin if name.startswith(('tree_','twig_','canopy_')) else 0.
            objects.append(self.collision(name,xyz,[v+padding for v in size]))
        self.scene(objects=objects)

    def run(self):
        if self.get_parameter('setup_only').value:
            self.install_cell_scene();return
        if not self.get_parameter('execute').value:
            self.get_logger().info('Orchard is idle. execute:=true starts the harvesting experiment.');return
        count=self.get_parameter('cycles').value
        if not 1<=count<=self.c['motion']['max_cycles']:raise RuntimeError('cycles outside configured bounds.')
        self.spin_until(lambda:self.machine is not None,30,'orchard backend')
        if self.machine['removed'] or self.machine['held']:raise RuntimeError('A fresh orchard launch is required.')
        response=self.service(GetParameters,'/robot_state_publisher/get_parameters',GetParameters.Request(names=['robot_description']))
        self.kinematics=Kinematics(response.values[0].string_value,self.c['motion']['rail_limit'])
        self.install_cell_scene()
        clearance=self.c['motion']['clearance_z']
        park=[.24,.02,clearance]
        self.move(park,'PARK_FOR_OBSERVATION')
        for cycle in range(count):
            self.phase_to('OBSERVE_FRUIT')
            visible=self.wait_detection()
            # All accepted fruits enter the scene. IDs derive from observations,
            # not Gazebo entity identities. Undetected canopy is a protected region.
            objects=[]
            for i,fruit in enumerate(visible):
                objects.append(self.collision(f'observed_{cycle}_{i}',fruit['xyz'],[fruit['radius']],shape='sphere'))
            self.scene(objects=objects)
            fruit=visible[0];obj=objects[0];grade=fruit['grade'];radius=fruit['radius']
            if grade not in self.placed:raise RuntimeError('Unknown fruit grade.')
            if self.placed[grade]>=2:raise RuntimeError('Basket capacity reached.')
            self.get_logger().info('FRUIT: '+json.dumps(fruit))
            pickup=list(fruit['xyz']);pickup[2]+=radius+.001
            approach=[pickup[0],pickup[1],clearance]
            self.move(approach,'APPROACH_FRUIT')
            self.allow_contact(obj.id,'rg2_link',True)
            self.corridor=(pickup,approach)
            self.move(pickup,'DESCEND_TO_FRUIT')
            self.phase_to('DETACH_STEM_AND_GRIP');self.command('grip')
            self.spin_until(lambda:self.machine['held'],3,'attachment acknowledgement')
            self.expect_payload=True;self.attach_scene(obj)
            self.move(approach,'WITHDRAW_FRUIT');self.corridor=None
            bx,by,_=self.c['bins'][grade]
            bx+=(-.028 if self.placed[grade]==0 else .028)
            self.move([bx,by,clearance],'TRANSFER_TO_'+grade.upper())
            release=[bx,by,self.c['motion']['release_top_z']]
            drop=release[2]-2*radius-self.c['bin_floor']
            if drop>self.c['motion']['max_drop_height']:raise RuntimeError('Release height exceeds configured drop bound.')
            self.move(release,'LOWER_TO_BASKET')
            self.command('release');self.expect_payload=False
            centre=[bx,by,release[2]-radius]
            detached=AttachedCollisionObject(link_name='rg2_link')
            detached.object.id=obj.id;detached.object.operation=CollisionObject.REMOVE
            placed=self.collision(obj.id,centre,[radius],shape='sphere')
            self.scene(objects=[placed],attached=[detached]);self.verify_attachment(obj.id,False)
            self.move([bx,by,clearance],'RETRACT')
            verifier=DropVerifier(self.c['bins'][grade],self.c['bin_inner'],self.c['bin_floor'],self.c['bin_height'],[2*radius]*3,'cylinder')
            settled=[]
            def in_bin():
                if time.monotonic()-self.machine_wall>.5:return False
                position=self.machine.get('release_xyz')
                if not verifier.update(position,self.machine['sim_time']):return False
                settled.extend(position);return True
            self.spin_until(in_bin,10,'fruit settled in selected basket')
            self.scene(objects=[self.collision(obj.id,settled,[radius],shape='sphere')])
            self.allow_contact(obj.id,'rg2_link',False)
            # Remove stale unpicked observations; the next scan recreates them.
            for other in objects[1:]:other.operation=CollisionObject.REMOVE
            self.scene(objects=objects[1:])
            self.harvested.append(np.array(fruit['xyz']));self.placed[grade]+=1
            self.records.append({'cycle':cycle+1,'grade':grade,'confidence':fruit['confidence'],
                'visible_fraction':fruit['visible_fraction'],'fit_error_m':fruit['fit_error'],
                'tree_contact':self.machine['tree_contact'],'outcome':'verified_in_basket'})
            self.write_log();self.move(park,'PARK_FOR_OBSERVATION')
        self.phase_to('COMPLETE')

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
        node=Harvester();node.run()
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
