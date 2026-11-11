"""Offline consistency checks. These do not simulate contacts or run MoveIt."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT.parent / 'pieas_arm_description'


def test_model_group_controller_and_limits_match():
    urdf = ET.parse(DESCRIPTION / 'urdf/pieas_control.urdf.in').getroot()
    srdf = ET.parse(ROOT / 'config/pieas_arm.srdf').getroot()
    assert urdf.attrib['name'] == srdf.attrib['name']
    names = [f'joint_{i}' for i in range(1, 7)]
    assert [j.attrib['name'] for j in srdf.find('group').findall('joint')] == names
    control = yaml.safe_load((ROOT / 'config/moveit_controllers.yaml').read_text())
    assert control['moveit_simple_controller_manager']['arm_controller']['joints'] == names
    limits = yaml.safe_load((ROOT / 'config/joint_limits.yaml').read_text())['joint_limits']
    for name in names:
        physical = urdf.find(f"joint[@name='{name}']/limit")
        assert 0 < limits[name]['max_velocity'] <= float(physical.get('velocity'))
        assert limits[name]['has_acceleration_limits'] and limits[name]['max_acceleration'] > 0
    for state in srdf.findall('group_state'):
        assert [j.get('name') for j in state] == names


def test_only_adjacent_collisions_are_excluded():
    urdf = ET.parse(DESCRIPTION / 'urdf/pieas_control.urdf.in').getroot()
    srdf = ET.parse(ROOT / 'config/pieas_arm.srdf').getroot()
    adjacent = {frozenset((j.find('parent').get('link'), j.find('child').get('link')))
                for j in urdf.findall('joint')}
    excluded = {frozenset((p.get('link1'), p.get('link2')))
                for p in srdf.findall('disable_collisions')}
    assert excluded == adjacent


def test_pipeline_checks_start_and_final_solution():
    config = yaml.safe_load((ROOT / 'config/ompl_planning.yaml').read_text())
    assert 'default_planning_request_adapters/CheckStartStateCollision' in config['request_adapters']
    assert 'default_planning_response_adapters/ValidateSolution' in config['response_adapters']
    assert config['arm']['longest_valid_segment_fraction'] <= 0.001


def test_python_and_xml_syntax():
    for path in list(ROOT.glob('launch/*.py')) + list(ROOT.glob('scripts/*')):
        ast.parse(path.read_text())
    ET.parse(ROOT / 'package.xml')
    for path in ROOT.glob('config/*.yaml'):
        assert isinstance(yaml.safe_load(path.read_text()), dict)


def test_world_generation_enables_self_contact_even_with_false_overrides():
    import shutil
    import subprocess
    import tempfile
    from unittest.mock import patch
    launch = DESCRIPTION / 'launch/lab.launch.py'
    tree = ast.parse(launch.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_world')
    scope = dict(Path=Path, ET=ET, shutil=shutil, subprocess=subprocess, tempfile=tempfile)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(launch), 'exec'), scope)

    def converted(args, **kwargs):
        model = ET.Element('model', name='converted')
        ET.SubElement(model, 'self_collide').text = 'false'
        for i in range(1, 7):
            ET.SubElement(model, 'joint', name=f'joint_{i}')
            link = ET.SubElement(model, 'link', name=f'link_{i}')
            ET.SubElement(link, 'self_collide').text = 'false'
        ET.SubElement(model, 'plugin', filename='gz_ros2_control-system')
        sdf = ET.Element('sdf', version='1.9')
        sdf.append(model)
        return subprocess.CompletedProcess(args, 0, ET.tostring(sdf, encoding='unicode'), '')

    with patch.object(subprocess, 'run', side_effect=converted):
        _, world, directory = scope['prepare_world'](DESCRIPTION.resolve())
    try:
        model = ET.parse(world).find("world/model[@name='pieas_arm']")
        assert model.findtext('self_collide') == 'true'
        assert all(link.findtext('self_collide') == 'true' for link in model.findall('link'))
    finally:
        shutil.rmtree(directory)
