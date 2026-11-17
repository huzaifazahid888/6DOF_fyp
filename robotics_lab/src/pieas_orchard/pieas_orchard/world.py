"""One geometry definition for the orchard world and MoveIt scene."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pieas_sorting.world import text, make_model, robot_variant, load_config


def boxes(c):
    # Rigid branches and leaf envelopes are protected collision objects.
    result = [('tree_trunk',[.46,0,.16],[.025,.025,.32],[.32,.18,.075]),
              ('tree_branch',[.435,0,.145],[.018,.24,.014],[.40,.23,.10]),
              ('canopy_guard',[.495,0,.26],[.10,.30,.20],[.12,.3,.06]),
              ('planter',[.46,0,.025],[.09,.12,.05],[.42,.26,.15])]
    for i,f in enumerate(c['fruits']):
        x,y,z=f['xyz']
        result.append((f'twig_{i}',[(x+.435)/2,y,z-.007],[.435-x-.04,.005,.005],[.36,.20,.08]))
        # Small leaf over the far edge of each fruit: partial visual occlusion.
        result.append((f'leaf_guard_{i}',[x+.020,y+.009,z+.021],[.012,.025,.0015],[.08,.30,.06]))
    d,w,h,f=c['bin_inner'],c['bin_wall'],c['bin_height'],c['bin_floor']
    colours={'ripe':[.70,.23,.12],'unripe':[.72,.60,.15],'damaged':[.40,.25,.53]}
    for name,(x,y,z) in c['bins'].items():
        colour=colours[name]
        result.append((f'bin_{name}_floor',[x,y,z+f/2],[d+2*w,d+2*w,f],colour))
        for sign in [-1,1]:
            result.append((f'bin_{name}_x{sign}',[x+sign*(d+w)/2,y,z+h/2],[w,d+2*w,h],colour))
            result.append((f'bin_{name}_y{sign}',[x,y+sign*(d+w)/2,z+h/2],[d,w,h],colour))
    return result


def sphere(world,name,xyz,radius,colour):
    # Reuse inertial/friction setup, then change cylinder to sphere.
    model=make_model(world,name,xyz,'cylinder',[radius,2*radius],colour,False)
    for geom in model.findall('./link/*/geometry'):
        geom.clear();text(ET.SubElement(geom,'sphere'),'radius',radius)
    inertia=model.find('./link/inertial/inertia')
    for tag in ['ixx','iyy','izz']:inertia.find(tag).text=str(.4*.025*radius**2)
    return model


def build_world(c):
    root=ET.Element('sdf',version='1.9');world=ET.SubElement(root,'world',name='lab')
    physics=ET.SubElement(world,'physics',name='physics',type='ignored')
    text(physics,'max_step_size',.001);text(physics,'real_time_factor',1)
    for library,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('contact','Contact')]:
        plugin=ET.SubElement(world,'plugin',filename=f'gz-sim-{library}-system',name=f'gz::sim::systems::{name}')
        if name=='Sensors':text(plugin,'render_engine','ogre2')
    scene=ET.SubElement(world,'scene');text(scene,'ambient','0.55 0.55 0.55 1');text(scene,'background','0.62 0.79 0.93 1');text(scene,'shadows','true')
    sun=ET.SubElement(world,'light',name='sun',type='directional')
    text(sun,'pose','0 0 4 0 0 0');text(sun,'direction','-0.2 0.1 -1');text(sun,'diffuse','0.8 0.8 0.75 1')
    make_model(world,'orchard_floor',[0,0,-.025],'box',[6,6,.05],[.23,.29,.12])
    # Walking path and rows give context but remain outside the work envelope.
    make_model(world,'path',[0,-.65,.002],'box',[4,.5,.004],[.48,.40,.28])
    for row in [1.,1.7]:
        for x in [-1.2,-.4,.4,1.2]:
            make_model(world,f'row_{row}_{x}',[x,row,.35],'cylinder',[.045,.7],[.3,.18,.08])
            for j in range(3):
                m=make_model(world,f'canopy_{row}_{x}_{j}',[x+(j-1)*.13,row,.70+(j%2)*.1],
                             'cylinder',[.22,.28],[.12,.31,.055])
                for geom in m.findall('./link/*/geometry'):
                    geom.clear();text(ET.SubElement(geom,'sphere'),'radius',.24)
    for name,xyz,size,colour in boxes(c):
        model=make_model(world,name,xyz,'box',size,colour)
        if name=='canopy_guard':model.find('link').remove(model.find('./link/visual'))
        if name.startswith(('tree_','twig_','leaf_','canopy_')):
            sensor=ET.SubElement(model.find('link'),'sensor',name='contact',type='contact')
            text(sensor,'always_on','true');text(sensor,'update_rate',100)
            text(sensor,'topic','/orchard/tree_contacts')
            text(ET.SubElement(sensor,'contact'),'collision','collision')
    # Decorative foliage is kept behind the protected branch envelope.
    rng=np.random.default_rng(c['seed'])
    for i in range(70):
        xyz=[rng.uniform(.45,.54),rng.uniform(-.14,.14),rng.uniform(.17,.35)]
        m=make_model(world,f'foliage_{i}',xyz,'box',[.028,.013,.001],[.08,.30+.12*rng.random(),.045])
        m.find('pose').text=' '.join(map(str,xyz+[rng.uniform(-.6,.6),rng.uniform(-.6,.6),rng.uniform(-math.pi,math.pi)]))
    for i,f in enumerate(c['fruits']):
        colour=[.82,.055,.025] if f['appearance']!='unripe' else [.65,.82,.06]
        m=sphere(world,f'apple_{i:03}',f['xyz'],c['fruit_radius'],colour)
        link=m.find('link')
        if f['appearance']=='damaged':
            vis=ET.SubElement(link,'visual',name='surface_blemish')
            text(vis,'pose','.006 0 .014 0 0 0')
            text(ET.SubElement(ET.SubElement(vis,'geometry'),'sphere'),'radius',.005)
            mat=ET.SubElement(vis,'material')
            for tag in ['ambient','diffuse']:text(mat,tag,'.18 .035 .08 1')
        # Thin side stem, leaving the upper surface available to the suction cup.
        vis=ET.SubElement(link,'visual',name='stem')
        text(vis,'pose','.018 0 -.003 0 1.57079632679 0')
        geom=ET.SubElement(ET.SubElement(vis,'geometry'),'cylinder')
        text(geom,'radius',.001);text(geom,'length',.010)
        mat=ET.SubElement(vis,'material')
        for tag in ['ambient','diffuse']:text(mat,tag,'.3 .16 .05 1')
    plugin=ET.SubElement(world,'plugin',filename='pieas_orchard_system',name='pieas::OrchardSystem')
    text(plugin,'fruit_count',len(c['fruits']));text(plugin,'fruit_radius',c['fruit_radius'])
    return ET.tostring(root,encoding='unicode')


def sensor_xml(key,c):
    cfg=c[key]
    sensor=ET.Element('sensor',name=key,type='rgbd_camera' if key=='camera' else 'gpu_lidar')
    text(sensor,'always_on','true');text(sensor,'update_rate',10);text(sensor,'topic',f'/orchard/{key}')
    text(sensor,'gz_frame_id','orchard_camera_optical' if key=='camera' else 'orchard_lidar')
    if key=='camera':
        cam=ET.SubElement(sensor,'camera');text(cam,'horizontal_fov',cfg['hfov'])
        im=ET.SubElement(cam,'image')
        for tag,value in [('width',cfg['width']),('height',cfg['height']),('format','R8G8B8')]:text(im,tag,value)
        for clip in [ET.SubElement(cam,'clip'),ET.SubElement(ET.SubElement(cam,'depth_camera'),'clip')]:
            text(clip,'near',.04);text(clip,'far',3)
    else:
        lidar=ET.SubElement(sensor,'lidar');scan=ET.SubElement(lidar,'scan')
        for tag,samples in [('horizontal',161),('vertical',121)]:
            sc=ET.SubElement(scan,tag)
            for k,v in [('samples',samples),('resolution',1),('min_angle',-.24),('max_angle',.24)]:text(sc,k,v)
        ran=ET.SubElement(lidar,'range')
        for k,v in [('min',.04),('max',3),('resolution',.001)]:text(ran,k,v)
    return sensor


def robot_description(source,c):
    root=ET.fromstring(robot_variant(source))
    base=root.find("link[@name='base_link']")
    # Visible base-mounted mast: fixed to the robot, not to the moving wrist.
    for name,xyz,size in [('mast',[-.22,.19,.39],[.015,.015,.78]),
                           ('boom',[.07,.19,.78],[.595,.015,.015]),
                           ('camera_support',[.395,.095,.74],[.012,.20,.08])]:
        for kind in ['visual','collision']:
            el=ET.SubElement(base,kind,name=name+'_'+kind)
            ET.SubElement(el,'origin',xyz=' '.join(map(str,xyz)),rpy='0 0 0')
            ET.SubElement(ET.SubElement(el,'geometry'),'box',size=' '.join(map(str,size)))
            if kind=='visual':ET.SubElement(ET.SubElement(el,'material',name=name),'color',rgba='.15 .17 .2 1')
    for key in ['camera','lidar']:
        name='orchard_'+key
        link=ET.SubElement(root,'link',name=name)
        # Housing behind the lens so it cannot occlude its own sensor.
        for kind in ['visual','collision']:
            el=ET.SubElement(link,kind);ET.SubElement(el,'origin',xyz='-.018 0 0',rpy='0 0 0')
            ET.SubElement(ET.SubElement(el,'geometry'),'box',size='.028 .035 .025')
            if kind=='visual':ET.SubElement(ET.SubElement(el,'material',name=name),'color',rgba='.07 .09 .12 1')
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass',value='.05')
        ET.SubElement(inertial,'inertia',ixx='.00002',iyy='.00002',izz='.00002',ixy='0',ixz='0',iyz='0')
        joint=ET.SubElement(root,'joint',name=name+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=' '.join(map(str,c[key]['xyz'])),rpy=' '.join(map(str,c[key]['rpy'])))
        text(ET.SubElement(root,'gazebo',reference=name+'_mount'),'preserveFixedJoint','true')
        ET.SubElement(root,'gazebo',reference=name).append(sensor_xml(key,c))
    link=ET.SubElement(root,'link',name='orchard_camera_optical')
    joint=ET.SubElement(root,'joint',name='orchard_optical_fixed',type='fixed')
    ET.SubElement(joint,'parent',link='orchard_camera');ET.SubElement(joint,'child',link='orchard_camera_optical')
    ET.SubElement(joint,'origin',xyz='0 0 0',rpy='-1.5707963267948966 0 -1.5707963267948966')
    return ET.tostring(root,encoding='unicode')
