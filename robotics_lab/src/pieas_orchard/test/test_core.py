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
