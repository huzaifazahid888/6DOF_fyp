#!/usr/bin/env python3
"""Real Jazzy/Gazebo acceptance: three grades, three placements, no tree contact."""
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from collections import Counter
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

launch_log=Path('/tmp/pieas_orchard_launch.log');task_log=Path('/tmp/pieas_orchard_task.log')
with launch_log.open('w') as output:
    process=subprocess.Popen(['ros2','launch','pieas_orchard','orchard.launch.py','rviz:=false','gui:=false'],
        stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        subprocess.run(['ros2','run','pieas_orchard','check_cell','--ros-args','-p','use_sim_time:=true','-p','timeout:=120.0'],check=True,timeout=135)
        rclpy.init();node=Node('orchard_acceptance');seen={};latest={};frames={};reasons=Counter()
        bridge=CvBridge()
        for kind,topic,key in [(Image,'/orchard/camera/image','rgb'),(Image,'/orchard/camera/depth_image','depth'),
                               (Image,'/orchard/perception_mask','mask'),(PointCloud2,'/orchard/lidar/points','lidar'),
                               (CameraInfo,'/orchard/camera/camera_info','info')]:
            node.create_subscription(kind,topic,lambda m,k=key:frames.update({k:m}),qos_profile_sensor_data)
        def receive(m):
            value=json.loads(m.data);latest.update(value);reasons[value.get('reason')]+=1
            for fruit in value['detections']:seen[fruit['grade']]=fruit
        node.create_subscription(String,'/orchard/detections',receive,10)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline and len(seen)<3:rclpy.spin_once(node,timeout_sec=.2)
        print('PERCEPTION ACCEPTANCE:',json.dumps(seen),flush=True)
        print('LAST DETECTION:',json.dumps(latest),flush=True)
        print('DETECTION REASONS:',dict(reasons),flush=True)
        evidence=Path('/tmp/pieas_orchard_sensors');evidence.mkdir(exist_ok=True)
        if 'rgb' in frames:cv2.imwrite(str(evidence/'camera.png'),bridge.imgmsg_to_cv2(frames['rgb'],'bgr8'))
        if 'mask' in frames:cv2.imwrite(str(evidence/'mask.png'),bridge.imgmsg_to_cv2(frames['mask'],'mono8'))
        if 'depth' in frames:np.save(evidence/'depth.npy',bridge.imgmsg_to_cv2(frames['depth'],'32FC1'))
        if 'lidar' in frames:np.save(evidence/'lidar.npy',point_cloud2.read_points_numpy(frames['lidar'],field_names=('x','y','z'),skip_nans=True))
        if 'info' in frames:(evidence/'intrinsics.json').write_text(json.dumps(list(frames['info'].k)))
        (evidence/'detections.json').write_text(json.dumps(latest))
        node.destroy_node();rclpy.shutdown()
        assert set(seen)=={'ripe','unripe','damaged'},seen
        with task_log.open('w') as result:
            subprocess.run(['ros2','run','pieas_orchard','harvest','--ros-args','-p','use_sim_time:=true',
                '-p','execute:=true','-p','cycles:=3','-p','log_directory:=/tmp/pieas_orchard_batch'],
                stdout=result,stderr=subprocess.STDOUT,check=True,timeout=1800)
        assert 'ORCHARD: COMPLETE' in task_log.read_text()
        files=list(Path('/tmp/pieas_orchard_batch').glob('batch_*.csv'));assert len(files)==1
        with files[0].open() as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==3 and Counter(row['grade'] for row in rows)=={'ripe':1,'unripe':1,'damaged':1}
        assert all(row['outcome']=='verified_in_basket' and row['tree_contact']=='False' for row in rows)
        print('PASS: rendered RGB-D/LiDAR, three grades, three basket placements, zero reported robot-tree contacts.',flush=True)
        print(files[0].read_text(),flush=True)
    finally:
        try:process.send_signal(signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()
        for path in [launch_log,task_log]:
            if path.exists():print('\n'+str(path)+'\n'+path.read_text(errors='replace')[-45000:],flush=True)

