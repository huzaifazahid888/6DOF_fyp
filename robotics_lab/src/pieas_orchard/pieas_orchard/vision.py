"""Learned appearance segmentation and geometric RGB-D fruit localization.

No simulator object names, fruit positions or grades are consumed here.
The small MLP is trained on synthetic colours, not real orchard photographs.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares


class AppearanceModel:
    def __init__(self, path):
        data = json.loads(Path(path).read_text())
        self.labels = data['labels']
        self.weights = [np.array(x) for x in data['weights']]
        self.biases = [np.array(x) for x in data['biases']]

    def predict(self, rgb):
        shape = rgb.shape[:-1]
        x = np.asarray(rgb, dtype=float).reshape(-1, 3) / 255.0
        for w, b in zip(self.weights[:-1], self.biases[:-1]):
            x = np.maximum(0, x @ w + b)
        x = x @ self.weights[-1] + self.biases[-1]
        x -= x.max(axis=1, keepdims=True)
        x = np.exp(x)
        return (x / x.sum(axis=1, keepdims=True)).reshape(*shape, -1)


def fit_sphere(points, radius_min, radius_max):
    points = np.asarray(points)
    if len(points) < 20 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite depth points.')
    # Algebraic sphere initialization, then robust radial least squares.
    a = np.c_[2 * points, np.ones(len(points))]
    b = np.sum(points * points, axis=1)
    sol, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
    if rank < 4:
        raise ValueError('Degenerate sphere fit.')
    centre = sol[:3]
    radius = np.sqrt(max(0, sol[3] + centre @ centre))
    if not radius_min * .7 < radius < radius_max * 1.3:
        raise ValueError('Depth surface is not an expected fruit.')
    result = least_squares(lambda v: np.linalg.norm(points-v[:3], axis=1)-v[3],
                           np.r_[centre, np.clip(radius, radius_min, radius_max)],
                           bounds=(np.r_[centre-.03,radius_min],np.r_[centre+.03,radius_max]),
                           loss='soft_l1', f_scale=.001, max_nfev=80)
    error = float(np.sqrt(np.mean(result.fun**2)))
    return result.x[:3], float(result.x[3]), error


def grade(probabilities, cfg):
    # Label order: background, leaf, ripe skin, unripe skin, visible defect.
    labels = np.argmax(probabilities, axis=1)
    fruit = labels >= 2
    if fruit.sum() < cfg['min_points']:
        return None
    labels, probs = labels[fruit], probabilities[fruit]
    damage = float(np.mean(labels == 4))
    skin = labels[labels != 4]
    if len(skin) == 0:
        return None
    ripe = float(np.mean(skin == 2))
    confidence = float(np.mean(np.max(probs, axis=1)))
    if confidence < cfg['grade_confidence']:
        return None
    if damage >= cfg['damage_fraction']:
        return 'damaged', confidence, damage, ripe
    if max(ripe, 1-ripe) < .80:
        return None
    return ('ripe' if ripe >= .80 else 'unripe'), confidence, damage, ripe


def detect(rgb, depth, intrinsics, rotation, translation, lidar, cfg, model):
    h, w = depth.shape
    if rgb.shape != (h, w, 3):
        raise ValueError('RGB/depth shape mismatch.')
    fx, fy, cx, cy = intrinsics
    if min(fx,fy) <= 0:
        raise ValueError('Invalid camera calibration.')
    v, u = np.indices((h,w))
    optical = np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],axis=-1)
    xyz = optical @ rotation.T + translation
    lo = np.array(cfg['roi'])[::2]; hi = np.array(cfg['roi'])[1::2]
    roi = np.isfinite(xyz).all(axis=-1) & (xyz>=lo).all(axis=-1) & (xyz<=hi).all(axis=-1)
    probabilities = np.zeros((h,w,5),dtype=float)
    probabilities[...,0]=1.
    if roi.any():probabilities[roi] = model.predict(rgb[roi])
    label = np.argmax(probabilities,axis=-1)
    mask = (roi & (label>=2) & (probabilities.max(axis=-1)>=cfg['pixel_confidence'])).astype('uint8')
    # Closing reconnects small defects/leaf holes, without joining separate fruit.
    merged = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count, components = cv2.connectedComponents(merged)
    detections=[]
    for component in range(1,count):
        selected=(components==component)&(mask>0)
        if selected.sum()<cfg['min_points']:continue
        points=xyz[selected]
        try:centre,radius,error=fit_sphere(points[::max(1,len(points)//1500)],cfg['radius_min'],cfg['radius_max'])
        except ValueError:continue
        if error>cfg['max_fit_error']:continue
        depth_centre=float((centre-translation) @ rotation[:,2])
        if depth_centre<=0:continue
        expected=np.pi*fx*fy*radius**2/depth_centre**2
        visibility=float(min(1.,selected.sum()/expected))
        if visibility<cfg['min_visible_fraction']:continue
        result=grade(probabilities[selected],cfg)
        if result is None:continue
        near=lidar[np.abs(np.linalg.norm(lidar-centre,axis=1)-radius)<cfg['lidar_tolerance']]
        if len(near)<cfg['min_lidar_points']:continue
        category,confidence,damage,ripe=result
        detections.append({'xyz':centre.tolist(),'radius':radius,'grade':category,
            'confidence':confidence,'visible_fraction':visibility,'fit_error':error,
            'damage_fraction':damage,'ripe_fraction':ripe,'lidar_points':len(near)})
    return sorted(detections,key=lambda d:d['xyz'][1]),mask*255


class StableDetections:
    def __init__(self, frames):
        self.frames=frames;self.previous=[];self.count=0

    def update(self, detections):
        same=len(detections)==len(self.previous) and bool(detections)
        if same:
            same=all(a['grade']==b['grade'] and np.linalg.norm(np.array(a['xyz'])-b['xyz'])<.002
                     for a,b in zip(detections,self.previous))
        self.count=self.count+1 if same else 1
        self.previous=detections
        return detections if self.count>=self.frames else []
