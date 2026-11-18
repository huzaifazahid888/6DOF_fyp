"""CPU checks; sensor rendering and physical execution are separate runtime tests."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'pieas_sorting')]
from pieas_orchard.vision import AppearanceModel,fit_sphere,detect,StableDetections
from pieas_orchard.world import load_config,boxes,build_world,robot_description
from pieas_sorting.perception_core import sensor_transform
from pieas_sorting.kinematics import Kinematics
C=load_config(ROOT/'config/orchard.yaml')
MODEL=AppearanceModel(ROOT/'models/appearance.json')


def render(category='ripe',occluded=0.):
    # Analytic ray/sphere image with an unrelated leaf occluder. This is a test
    # fixture only; production perception consumes Gazebo sensor messages.
    h,w=240,320;fx=w/(2*np.tan(C['camera']['hfov']/2));k=[fx,fx,w/2,h/2]
    r,t=sensor_transform(C['camera'],True)
    v,u=np.indices((h,w));rays=np.stack([(u-k[2])/fx,(v-k[3])/fx,np.ones((h,w))],axis=-1)@r.T
    centre=np.array([.36,0,.145]);radius=.016
    origin=t-centre;a=np.sum(rays*rays,axis=-1);b=2*(rays@origin);disc=b*b-4*a*(origin@origin-radius**2)
    mask=disc>0;depth=np.full((h,w),np.nan,dtype=np.float32)
    depth[mask]=((-b[mask]-np.sqrt(disc[mask]))/(2*a[mask])).astype(np.float32)
    rgb=np.zeros((h,w,3),dtype=np.uint8);rgb[:]=[80,100,40]
    rgb[mask]=np.array([.65,.82,.06] if category=='unripe' else [.82,.055,.025])*255
    if category=='damaged':rgb[mask&(u>w/2)&(u<w/2+5)]=np.array([.18,.035,.08])*255
    if occluded:
        edge=np.quantile(u[mask],1-occluded)
        rgb[mask&(u>=edge)]=np.array([.08,.30,.06])*255
    optical=np.stack([(u-k[2])*depth/fx,(v-k[3])*depth/fx,depth],axis=-1)
    lidar=(optical@r.T+t)[mask][::8]
    return rgb,depth,k,r,t,lidar


@pytest.mark.parametrize('category',['ripe','unripe','damaged'])
def test_sensor_only_grade_and_localization(category):
    args=render(category,.12)
    results,_=detect(*args,C['perception'],MODEL)
    assert len(results)==1
    fruit=results[0]
    assert fruit['grade']==category
    assert np.linalg.norm(np.array(fruit['xyz'])-[.36,0,.145])<.001
    assert abs(fruit['radius']-.016)<.001


def test_missing_lidar_rejects():
    args=list(render());args[-1]=np.empty((0,3))
    assert detect(*args,C['perception'],MODEL)[0]==[]


def test_heavy_occlusion_rejects():
    assert detect(*render(occluded=.8),C['perception'],MODEL)[0]==[]


def test_invalid_depth_rejects():
    args=list(render());args[1][:]=np.nan
    assert detect(*args,C['perception'],MODEL)[0]==[]


def test_flat_depth_cannot_be_fruit():
    x,y=np.meshgrid(np.linspace(-.01,.01,20),np.linspace(-.01,.01,20))
    with pytest.raises(ValueError):fit_sphere(np.c_[x.ravel(),y.ravel(),np.ones(400)],.012,.021)


def test_temporal_stability_resets():
    stable=StableDetections(3);fruit=[{'xyz':[.36,0,.145],'grade':'ripe'}]
    assert not stable.update(fruit);assert not stable.update(fruit);assert stable.update(fruit)
    assert not stable.update([]);assert not stable.update(fruit)


def test_world_and_protected_scene_match():
    world=ET.fromstring(build_world(C)).find('world')
    names=[m.get('name') for m in world.findall('model')]
    assert len(names)==len(set(names))
    for name,xyz,size,_ in boxes(C):
        model=world.find(f"model[@name='{name}']")
        assert model is not None
        assert np.allclose(list(map(float,model.find('pose').text.split()[:3])),xyz)
        assert np.allclose(list(map(float,model.find('./link/collision/geometry/box/size').text.split())),size)
    assert len(world.findall(".//sensor[@type='contact']"))>=6
    assert not world.findall(".//sensor[@type='rgbd_camera']") # sensor is on robot


def test_sensors_and_original_arm_transforms():
    source=ROOT.parent/'pieas_arm_description/urdf/pieas_control.urdf.in'
    before=ET.parse(source).getroot();after=ET.fromstring(robot_description(source,C))
    for name in ['joint_'+str(i) for i in range(1,7)]+['tool0_fixed']:
        assert ET.tostring(before.find(f"joint[@name='{name}']")).strip()==ET.tostring(after.find(f"joint[@name='{name}']")).strip()
    for name in ['orchard_camera','orchard_lidar']:
        assert after.find(f"link[@name='{name}']/visual") is not None
        assert after.find(f"gazebo[@reference='{name}']/sensor") is not None


def test_reachable_pick_and_baskets():
    source=ROOT.parent/'pieas_arm_description/urdf/pieas_control.urdf.in'
    kin=Kinematics(robot_description(source,C),.16);q=np.zeros(6)
    targets=[[.24,.02,.23]]
    targets += [[f['xyz'][0],f['xyz'][1],f['xyz'][2]+.017] for f in C['fruits']]
    targets += [[v[0]-.028,v[1],.14] for v in C['bins'].values()]
    for target in targets:
        q=next(kin.candidates(target,q),None)
        assert q is not None,target
        assert np.linalg.norm(kin.forward(q)[:3,3]-target)<.001


def test_model_is_trained_and_reproducible_metadata():
    data=json.loads((ROOT/'models/appearance.json').read_text())
    assert data['synthetic_accuracy']>.95
    assert data['training_seed']!=data['test_seed']
    assert MODEL.weights[0].shape==(3,24)



def test_lidar_sightlines_clear_camera_housing_and_mount():
    # Ray/slab intersections catch hardware placed between a sensor and fruit.
    robot=ET.fromstring(robot_description(ROOT.parent/'pieas_arm_description/urdf/pieas_control.urdf.in',C))
    start=np.array(C['lidar']['xyz'])
    obstacles=[]
    base=robot.find("link[@name='base_link']")
    for collision in base.findall('collision'):
        box=collision.find('geometry/box')
        if box is not None:
            centre=np.array(list(map(float,collision.find('origin').get('xyz').split())))
            half=np.array(list(map(float,box.get('size').split())))/2
            obstacles.append((centre-half,centre+half))
    # Camera housing is behind its local X lens axis. Transform eight corners.
    rotation,translation=sensor_transform(C['camera'])
    corners=np.array([[x,y,z] for x in [-.032,-.004] for y in [-.0175,.0175] for z in [-.0125,.0125]])
    corners=corners@rotation.T+translation
    obstacles.append((corners.min(axis=0),corners.max(axis=0)))
    for fruit in C['fruits']:
        direction=np.array(fruit['xyz'])-start
        for low,high in obstacles:
            near,far=0.,1.
            for axis in range(3):
                if abs(direction[axis])<1e-12:
                    if not low[axis]<=start[axis]<=high[axis]:near,far=1.,0.;break
                else:
                    hits=sorted([(low[axis]-start[axis])/direction[axis],(high[axis]-start[axis])/direction[axis]])
                    near=max(near,hits[0]);far=min(far,hits[1])
            assert near>far, 'LiDAR line of sight blocked by sensor mounting geometry'


def test_gazebo_recorded_frame_regression():
    # Development regression, not an independent evaluation scene.
    data=np.load(ROOT/'test/fixtures/gazebo_frame.npz')
    camera=dict(C['camera'],xyz=data['camera_xyz'].tolist())
    lidar=dict(C['lidar'],xyz=data['lidar_xyz'].tolist())
    rotation,translation=sensor_transform(camera,True)
    lr,lt=sensor_transform(lidar)
    found,_=detect(data['rgb'],data['depth'],data['intrinsics'],rotation,translation,
                   data['lidar']@lr.T+lt,C['perception'],MODEL)
    assert [f['grade'] for f in found]==['ripe','unripe','damaged']
    for fruit,y in zip(found,[-.065,0,.065]):
        assert np.linalg.norm(np.array(fruit['xyz'])-[.36,y,.145])<.002


def test_branch_padding_leaves_room_for_fruit_estimation_error():
    # Fruit pose estimates can differ from truth within the accepted tolerance.
    geometry={name:(np.array(xyz),np.array(size)) for name,xyz,size,_ in boxes(C)}
    for i,fruit in enumerate(C['fruits']):
        centre,size=geometry[f'twig_{i}']
        padded_near=centre[0]-size[0]/2-C['motion']['branch_margin']
        fruit_far=fruit['xyz'][0]+C['fruit_radius']+C['motion']['endpoint_tolerance']
        assert padded_near>fruit_far


def test_reference_extraction_clears_overhanging_leaf():
    geometry={name:(np.array(xyz),np.array(size)/2) for name,xyz,size,_ in boxes(C)}
    for i,fruit in enumerate(C['fruits']):
        centre=np.array(fruit['xyz']);leaf,half=geometry[f'leaf_guard_{i}']
        extracted=centre+np.array([C['motion']['extraction_x_offset'],0,0])
        lifted=extracted.copy();lifted[2]=C['motion']['clearance_z']-C['fruit_radius']
        for start,end in [(centre,extracted),(extracted,lifted)]:
            for p in np.linspace(start,end,100):
                nearest=np.clip(p,leaf-half,leaf+half)
                assert np.linalg.norm(p-nearest)>C['fruit_radius']+C['motion']['endpoint_tolerance']
