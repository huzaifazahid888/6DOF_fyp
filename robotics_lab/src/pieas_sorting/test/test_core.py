"""CPU-only tests of sensing geometry, decisions, world consistency and IK."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pieas_sorting.world import load_config,build_world,robot_variant,boxes
from pieas_sorting.perception_core import classify,project,sensor_transform,detect,Stability
from pieas_sorting.kinematics import Kinematics

PACKAGE=Path(__file__).resolve().parents[1]
CONFIG=load_config(PACKAGE/'config/cell.yaml')
SOURCE=PACKAGE.parent/'pieas_arm_description/urdf/pieas_control.urdf.in'


def synthetic(shape,with_lidar=True):
    c=CONFIG;h,w=c['camera']['height'],c['camera']['width']
    focal=w/(2*np.tan(c['camera']['hfov']/2));intrinsic=[focal,focal,w/2,h/2]
    depth=np.full((h,w),c['camera']['xyz'][2]-c['belt']['top'],np.float32)
    rgb=np.full((h,w,3),30,np.uint8);mask=np.zeros((h,w),np.uint8)
    top_depth=c['camera']['xyz'][2]-c['belt']['top']-c['parts']['height']
    radius=round(.018*focal/top_depth)
    if shape=='cylinder':cv2.circle(mask,(w//2,h//2),radius,255,-1)
    elif shape=='triangle':cv2.fillPoly(mask,[np.array([[300,220],[340,220],[320,260]])],255)
    else:
        a=.03 if shape=='cube' else .025;b=.03 if shape=='cube' else .05
        dx,dy=round(a/2*focal/top_depth),round(b/2*focal/top_depth)
        cv2.rectangle(mask,(w//2-dx,h//2-dy),(w//2+dx,h//2+dy),255,-1)
    depth[mask>0]=top_depth;rgb[mask>0]=[38,217,51]
    grid=np.array([[.36+x,y,.15] for x in np.linspace(-.01,.01,10) for y in np.linspace(-.01,.01,10)])
    return rgb,depth,intrinsic,grid if with_lidar else np.empty((0,3))


@pytest.mark.parametrize('shape',['cube','rectangle','cylinder'])
def test_sensor_shapes_and_metric_localisation(shape):
    result,_=detect(*synthetic(shape),CONFIG)
    assert len(result)==1 and result[0]['shape']==shape
    assert np.linalg.norm(np.array(result[0]['xyz'])-[.36,0,.135])<.002
    assert abs(result[0]['size'][2]-.03)<.001
    assert result[0]['lidar_points']>=5


def test_unknown_shape_and_missing_lidar_are_rejected():
    assert not detect(*synthetic('triangle'),CONFIG)[0]
    assert not detect(*synthetic('cube',False),CONFIG)[0]
    rgb,depth,intrinsic,cloud=synthetic('cube');cloud[:,2]=.19
    assert not detect(rgb,depth,intrinsic,cloud,CONFIG)[0]


def test_depth_projection_axes_and_metres():
    r,t=sensor_transform(CONFIG['camera'],True)
    points=project(np.array([[320,240],[330,240],[320,250]]),np.ones(3)*.55,[750,750,320,240],r,t)
    assert np.allclose(points[0],[.36,0,.15])
    assert points[1,1]<points[0,1] and points[2,0]<points[0,0]


def test_stability_resets_on_motion_or_ambiguity():
    stable=Stability(3);a={'shape':'cube','xyz':[.36,0,.135]}
    assert stable.update([a]) is None and stable.update([a]) is None
    assert stable.update([a])==a
    assert stable.update([dict(a,xyz=[.36,.01,.135])]) is None
    assert stable.update([]) is None and stable.update([a,a]) is None


def test_world_matches_shared_collision_geometry_and_sensor_definitions():
    world=ET.fromstring(build_world(CONFIG)).find('world')
    for name,xyz,size,_ in boxes(CONFIG):
        model=world.find(f"model[@name='{name}']")
        assert np.allclose(list(map(float,model.findtext('pose').split()[:3])),xyz)
        assert np.allclose(list(map(float,model.findtext('link/collision/geometry/box/size').split())),size)
    parts=[m for m in world.findall('model') if m.get('name').startswith('part_')]
    assert len(parts)==CONFIG['parts']['count']
    assert len({m.findtext('link/visual/material/diffuse') for m in parts})==1
    assert world.find("model[@name='factory_camera']/link/sensor").get('type')=='rgbd_camera'
    assert world.find("model[@name='factory_lidar']/link/sensor").get('type')=='gpu_lidar'


def test_suction_variant_preserves_kinematics_and_original_source():
    before=SOURCE.read_text();variant=robot_variant(SOURCE)
    original=ET.fromstring(before);modified=ET.fromstring(variant)
    for joint in original.findall('joint'):
        assert ET.tostring(joint)==ET.tostring(modified.find(f"joint[@name='{joint.get('name')}']"))
    assert SOURCE.read_text()==before
    assert modified.find("link[@name='rg2_link']/collision/geometry/cylinder") is not None
    kin=Kinematics(variant)
    assert np.linalg.norm(kin.forward(np.zeros(6))[:3,3]-[.2625327293,-.0000036604,.1223977756])<1e-8


def test_station_and_bin_targets_have_bounded_upright_ik():
    kin=Kinematics(robot_variant(SOURCE));q=np.zeros(6)
    targets=[[.24,.02,.22],[.36,0,.22],[.36,0,.151]]
    for x,y,_ in CONFIG['bins'].values():
        for dx in [-.028,.028]:targets.extend([[x+dx,y,.22],[x+dx,y,.16]])
    for target in targets:
        solution=next(kin.candidates(target,q,32),None)
        assert solution is not None, target
        t=kin.forward(solution)
        assert np.linalg.norm(t[:3,3]-target)<.0008
        assert t[2,2]>np.cos(.04)
        assert np.all(solution>=kin.lower) and np.all(solution<=kin.upper)
        q=solution


def test_unreachable_target_does_not_produce_fake_solution():
    kin=Kinematics(robot_variant(SOURCE))
    assert list(kin.candidates([10,10,10],np.zeros(6),attempts=1))==[]
    with pytest.raises(ValueError):list(kin.candidates([float('nan'),0,0],np.zeros(6)))


def test_task_rejects_corrupted_sensor_goals():
    import copy
    from pieas_sorting.task_logic import validate_detection
    value=detect(*synthetic('cube'),CONFIG)[0][0]
    validate_detection(value,CONFIG)
    for key,bad in [('xyz',[float('nan'),0,.1]),('xyz',[.9,0,.1]),('shape','unknown'),
                    ('top_z',.5),('size',[.03,.03,-.03]),('lidar_points',0),('confidence',.1)]:
        corrupted=copy.deepcopy(value);corrupted[key]=bad
        with pytest.raises(ValueError):validate_detection(corrupted,CONFIG)


def test_drop_requires_stationary_independent_samples_in_correct_bin():
    from pieas_sorting.task_logic import DropVerifier
    v=DropVerifier([.2,-.21,0],.12,.03,.09,[.03,.03,.03],'cube')
    assert not v.update([.2,-.21,.045],1.)
    assert not v.update([.2,-.21,.045],1.) # repeated message is not evidence
    assert not v.update([.2,-.21,.045],1.2)
    assert v.update([.2,-.21,.045],1.5)
    assert not v.update([.5,-.21,.045],1.6)
    assert not v.update([.2,-.21,.15],1.7)


def test_ros_string_callbacks_read_fields_and_detect_clock_reset():
    # Execute the adapter callbacks with ROS-shaped messages (data is a FIELD,
    # unlike Gazebo protobuf's data() accessor). No ROS runtime is needed here.
    import ast,json,time
    from types import SimpleNamespace
    tree=ast.parse((PACKAGE/'pieas_sorting/sorter.py').read_text())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='Sorter')
    methods=[node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name in ('machine_callback','detection_callback')]
    scope={'json':json,'time':time}
    exec(compile(ast.Module(body=methods,type_ignores=[]),'<callbacks>','exec'),scope)
    node=SimpleNamespace(last_sim=None,reset_detected=False)
    scope['machine_callback'](node,SimpleNamespace(data=json.dumps({'sim_time':10.,'held':False})))
    assert node.machine['held'] is False and node.last_sim==10.
    scope['detection_callback'](node,SimpleNamespace(data=json.dumps({'detections':[]})))
    assert node.detection['detections']==[] and node.detection_wall>0
    scope['machine_callback'](node,SimpleNamespace(data=json.dumps({'sim_time':1.})))
    assert node.reset_detected is True


def test_continuous_encoder_turns_are_canonical_without_changing_fk():
    from pieas_sorting.kinematics import canonical_positions,JOINTS
    kin=Kinematics(robot_variant(SOURCE))
    assert kin.continuous=={'joint_6'}
    for turns in [-3,-1,1,4]:
        q=np.array([.04,-2.,-.5,.85,-.39,-1.2+turns*2*np.pi])
        normalized=canonical_positions(JOINTS,q,kin.continuous)
        assert -np.pi<=normalized[-1]<=np.pi
        assert np.allclose(normalized[:-1],q[:-1])
        assert np.allclose(kin.forward(q),kin.forward(normalized),atol=1e-12)
