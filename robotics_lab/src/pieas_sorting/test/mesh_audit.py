#!/usr/bin/env python3
"""Optional offline mesh collision audit: requires trimesh and python-fcl.

Checks sampled endpoint states only. Not a Gazebo or continuous-path test.
Run from any directory with the repository source tree present.
"""
import sys,xml.etree.ElementTree as ET
from pathlib import Path
PACKAGE=Path(__file__).resolve().parents[1]
DESCRIPTION=PACKAGE.parent/'pieas_arm_description'
sys.path.insert(0,str(PACKAGE))
from pieas_sorting.world import robot_variant,boxes,load_config
from pieas_sorting.kinematics import Kinematics,JOINTS
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
c=load_config(PACKAGE/'config/cell.yaml')
src=DESCRIPTION/'urdf/pieas_control.urdf.in'
r=ET.fromstring(robot_variant(src));kin=Kinematics(ET.tostring(r).decode());bychild={j.find('child').get('link'):j for j in r.findall('joint')}
def origin(o):
 T=np.eye(4)
 if o is not None:T[:3,3]=list(map(float,o.get('xyz','0 0 0').split()));T[:3,:3]=Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_matrix()
 return T
meshes={}
for l in r.findall('link'):
 parts=[]
 for collision in l.findall('collision'):
  g=collision.find('geometry');m=g.find('mesh')
  if m is not None:mesh=trimesh.load(DESCRIPTION/'meshes'/m.get('filename').split('/')[-1])
  else:
   p=g.find('cylinder');mesh=trimesh.creation.cylinder(radius=float(p.get('radius')),height=float(p.get('length')),sections=24)
  mesh.apply_transform(origin(collision.find('origin')));parts.append(mesh)
 if parts:meshes[l.get('name')]=trimesh.util.concatenate(parts)
static=trimesh.collision.CollisionManager()
for name,xyz,size,_ in boxes(c):
 T=np.eye(4);T[:3,3]=xyz;static.add_object(name,trimesh.creation.box(size),T)
allowed={frozenset([j.find('parent').get('link'),j.find('child').get('link')]) for j in r.findall('joint')}
def check(q,payload=False):
 trans={'world':np.eye(4)};vals=dict(zip(JOINTS,q))
 def tf(link):
  if link in trans:return trans[link]
  j=bychild[link];T=tf(j.find('parent').get('link'))@origin(j.find('origin'));M=np.eye(4)
  if j.get('type')!='fixed':
   axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
   if j.get('type')=='prismatic':M[:3,3]=axis*vals[j.get('name')]
   else:M[:3,:3]=Rotation.from_rotvec(axis*vals[j.get('name')]).as_matrix()
  trans[link]=T@M;return trans[link]
 robot=trimesh.collision.CollisionManager()
 for name,m in meshes.items():robot.add_object(name,m,tf(name))
 if payload:
  T=kin.forward(q);T[:3,3]-=T[:3,2]*.015;robot.add_object('payload',trimesh.creation.box([.05,.025,.03]),T)
 _,pairs=robot.in_collision_internal(return_names=True)
 pairs={pair for pair in pairs if frozenset(pair) not in allowed and set(pair)!={'rg2_link','payload'}}
 _,external=robot.in_collision_other(static,return_names=True)
 return pairs|external
print('HOME',check(np.zeros(6)),flush=True)
failures=bool(check(np.zeros(6)))
q=np.zeros(6)
targets=[('park',[.24,.02,.22]),('approach',[.36,0,.22]),('pick',[.36,0,.151])]
for shape,(x,y,z) in c['bins'].items():
 for dx in [-.028,.028]:targets.extend([(shape+'_high',[x+dx,y,.22]),(shape+'_release',[x+dx,y,.16])])
for name,p in targets:
 errors=[];sol=None
 for candidate in kin.candidates(p,q,32):
  err=check(candidate, name.endswith('high') or name.endswith('release'))
  if not err:sol=candidate;break
  errors.append(err)
 print(name,p, 'OK '+str(sol.round(3)) if sol is not None else 'FAIL '+str(errors[:4]),flush=True)
 if sol is not None:q=sol
 else:failures=True

raise SystemExit(1 if failures else 0)
