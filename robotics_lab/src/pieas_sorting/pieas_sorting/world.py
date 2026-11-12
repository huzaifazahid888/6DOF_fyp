"""Build the cell and its collision geometry from the same configuration."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def load_config(path):
    return yaml.safe_load(Path(path).read_text())


def boxes(c):
    """(name, centre XYZ, size XYZ, colour); used by Gazebo AND MoveIt."""
    b = c['belt']
    result = [('belt_deck', [b['x'], (b['y_min']+b['y_max'])/2, b['top']-0.02],
               [b['width'], b['y_max']-b['y_min'], 0.04], [0.12, 0.14, 0.16])]
    colours = {'cube': [0.2, 0.5, 0.9], 'rectangle': [0.9, 0.65, 0.1], 'cylinder': [0.5, 0.3, 0.8]}
    w, d, h, f = c['bin_wall'], c['bin_inner'], c['bin_height'], c['bin_floor']
    for name, (x, y, z) in c['bins'].items():
        colour = colours[name]
        result.append((f'bin_{name}_floor', [x,y,z+f/2], [d+2*w,d+2*w,f], colour))
        for sign in [-1,1]:
            result.append((f'bin_{name}_x{sign}', [x+sign*(d+w)/2,y,z+h/2], [w,d+2*w,h],colour))
            result.append((f'bin_{name}_y{sign}', [x,y+sign*(d+w)/2,z+h/2], [d,w,h],colour))
    # Factory scenery stays outside the arm workspace. Also collision modelled.
    result += [('back_wall',[0,1.5,0.7],[3.5,0.05,1.4],[0.45,0.5,0.55]),
               ('cabinet',[-1.0,-0.7,0.35],[0.4,0.35,0.7],[0.15,0.35,0.4])]
    return result


def text(parent, tag, value):
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def geometry(parent, kind, dimensions):
    geom = ET.SubElement(parent, 'geometry')
    shape = ET.SubElement(geom, kind)
    if kind == 'box':
        text(shape, 'size', ' '.join(map(str, dimensions)))
    else:
        text(shape, 'radius', dimensions[0])
        text(shape, 'length', dimensions[1])


def make_model(world, name, xyz, kind, dimensions, colour, static=True):
    model = ET.SubElement(world, 'model', name=name)
    text(model, 'static', str(static).lower())
    text(model, 'pose', ' '.join(map(str, xyz))+' 0 0 0')
    link = ET.SubElement(model, 'link', name='body')
    if not static:
        mass = 0.025
        inertial = ET.SubElement(link, 'inertial')
        text(inertial, 'mass', mass)
        inertia = ET.SubElement(inertial, 'inertia')
        if kind == 'box':
            x,y,z = dimensions
            diagonal = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
        else:
            r,h = dimensions
            diagonal = [mass*(3*r*r+h*h)/12]*2 + [mass*r*r/2]
        for tag, value in zip(['ixx','iyy','izz'], diagonal): text(inertia,tag,value)
        for tag in ['ixy','ixz','iyz']: text(inertia,tag,0)
    collision = ET.SubElement(link, 'collision', name='collision')
    geometry(collision,kind,dimensions)
    surface = ET.SubElement(collision,'surface')
    ode = ET.SubElement(ET.SubElement(surface,'friction'),'ode')
    # Belt traction is applied by the cell plugin; a low-friction deck avoids
    # counting both static-world friction and conveyor traction as separate belts.
    for tag in ['mu','mu2']: text(ode,tag,0.01 if name == 'belt_deck' else 0.6)
    visual = ET.SubElement(link,'visual',name='visual')
    geometry(visual,kind,dimensions)
    material = ET.SubElement(visual,'material')
    for tag in ['ambient','diffuse']: text(material,tag,' '.join(map(str,colour))+' 1')
    return model


def robot_variant(source):
    """Keep arm kinematics; replace rigid RG2 with an explicit suction surrogate."""
    root = ET.parse(source).getroot()
    link = root.find("link[@name='rg2_link']")
    for child in list(link): link.remove(child)
    # Shaft follows the original RG2 +Z. tool0 is at its end and +Z points back.
    for name, radius, length, centre in [('shaft',0.009,0.18,0.09),('cup',0.012,0.012,0.189355912714)]:
        for kind in ['visual','collision']:
            item = ET.SubElement(link,kind,name=f'suction_{name}_{kind}')
            ET.SubElement(item,'origin',xyz=f'0.00152937072616 -0.0000956811660029 {centre}',rpy='0 0 0')
            ET.SubElement(ET.SubElement(item,'geometry'),'cylinder',radius=str(radius),length=str(length))
            if kind == 'visual':
                ET.SubElement(ET.SubElement(item,'material',name=f'suction_{name}'),'color',rgba='0.15 0.2 0.25 1')
    inertial = ET.SubElement(link,'inertial')
    ET.SubElement(inertial,'origin',xyz='0 0 0.09',rpy='0 0 0')
    ET.SubElement(inertial,'mass',value='0.12')
    ET.SubElement(inertial,'inertia',ixx='0.00033',iyy='0.00033',izz='0.000008',ixy='0',ixz='0',iyz='0')
    return ET.tostring(root,encoding='unicode')


def build_world(c):
    root = ET.Element('sdf',version='1.9')
    world = ET.SubElement(root,'world',name='lab')
    physics = ET.SubElement(world,'physics',name='physics',type='ignored')
    text(physics,'max_step_size',0.001); text(physics,'real_time_factor',1)
    for library, name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        plugin = ET.SubElement(world,'plugin',filename=f'gz-sim-{library}-system',name=f'gz::sim::systems::{name}')
        if name == 'Sensors':text(plugin,'render_engine','ogre2')
    sun = ET.SubElement(world,'light',name='sun',type='directional')
    text(sun,'pose','0 0 4 0 0 0');text(sun,'direction','-0.3 -0.2 -1');text(sun,'diffuse','0.9 0.9 0.9 1')
    make_model(world,'factory_floor',[0,0,-0.025],'box',[4,4,0.05],[0.5,0.52,0.55])
    for name,xyz,size,colour in boxes(c):make_model(world,name,xyz,'box',size,colour)
    p,b = c['parts'],c['belt']
    for i in range(p['count']):
        shape = ['cube','rectangle','cylinder'][i%3]
        size = [p['cube_size']]*3 if shape == 'cube' else list(p['rectangle_size'])+[p['height']]
        if shape == 'cylinder':size=[p['cylinder_radius'],p['height']]
        # Every class uses the same colour. Classification cannot use a colour ID.
        make_model(world,f'part_{i:03}',[b['x'],p['first_y']-i*p['spacing'],b['top']+p['height']/2+0.001],
                   'cylinder' if shape == 'cylinder' else 'box',size,[0.15,0.85,0.2],False)
    for key in ['camera','lidar']:
        cfg = c[key]
        model = ET.SubElement(world,'model',name=f'factory_{key}')
        text(model,'static','true');text(model,'pose',' '.join(map(str,cfg['xyz']+cfg['rpy'])))
        link = ET.SubElement(model,'link',name=f'{key}_link')
        sensor = ET.SubElement(link,'sensor',name=key,type='rgbd_camera' if key == 'camera' else 'gpu_lidar')
        text(sensor,'always_on','true');text(sensor,'update_rate',10)
        text(sensor,'topic',f'/factory/{key}')
        text(sensor,'gz_frame_id',f'factory_{key}_optical' if key == 'camera' else 'factory_lidar')
        if key == 'camera':
            cam=ET.SubElement(sensor,'camera');text(cam,'horizontal_fov',cfg['hfov'])
            im=ET.SubElement(cam,'image')
            text(im,'width',cfg['width']);text(im,'height',cfg['height']);text(im,'format','R8G8B8')
            for clip in [ET.SubElement(cam,'clip'), ET.SubElement(ET.SubElement(cam,'depth_camera'),'clip')]:
                text(clip,'near',0.05);text(clip,'far',2)
        else:
            lidar=ET.SubElement(sensor,'lidar');scan=ET.SubElement(lidar,'scan')
            for tag,samples in [('horizontal',81),('vertical',61)]:
                s=ET.SubElement(scan,tag)
                for k,v in [('samples',samples),('resolution',1),('min_angle',-0.12),('max_angle',0.12)]:text(s,k,v)
            ran=ET.SubElement(lidar,'range')
            for k,v in [('min',0.05),('max',2),('resolution',0.001)]:text(ran,k,v)
            noise=ET.SubElement(lidar,'noise');text(noise,'type','gaussian');text(noise,'mean',0);text(noise,'stddev',0.001)
    plugin=ET.SubElement(world,'plugin',filename='pieas_factory_system',name='pieas::FactorySystem')
    for k,v in {'belt_x':b['x'],'belt_top':b['top'],'belt_half_width':b['width']/2,'belt_y_min':b['y_min'],
                'belt_y_max':b['y_max'],'stop_y':b['stop_y'],'speed':b['speed'],'part_count':p['count'],'part_height':p['height']}.items():text(plugin,k,v)
    return ET.tostring(root,encoding='unicode')
