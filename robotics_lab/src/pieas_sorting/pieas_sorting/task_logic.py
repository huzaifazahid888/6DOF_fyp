"""Pure task acceptance rules, kept separate from ROS transport for testing."""
import math
import numpy as np


def validate_detection(value,c):
    """Reject malformed or out-of-workspace data before it becomes a robot goal."""
    if value.get('shape') not in c['bins']:raise ValueError('Unknown detected shape.')
    xyz=np.asarray(value.get('xyz',[]),dtype=float)
    size=np.asarray(value.get('size',[]),dtype=float)
    if xyz.shape!=(3,) or size.shape!=(3,) or not np.isfinite(xyz).all() or not np.isfinite(size).all():
        raise ValueError('Detection must contain finite XYZ and dimensions.')
    b,p=c['belt'],c['perception']
    if abs(xyz[0]-b['x'])>p['roi_half_x'] or abs(xyz[1]-b['stop_y'])>p['roi_half_y']:
        raise ValueError('Detection is outside the inspection station.')
    if not (0.018<min(size[:2])<0.045 and max(size[:2])<0.065 and 0.02<size[2]<0.045):
        raise ValueError('Detected part dimensions are outside supported range.')
    top=float(value.get('top_z',float('nan')))
    if not math.isfinite(top) or abs(top-(xyz[2]+size[2]/2))>.003 or abs(top-size[2]-b['top'])>.003:
        raise ValueError('Detected top, centre and belt height disagree.')
    if not math.isfinite(float(value.get('yaw',float('nan')))):raise ValueError('Invalid detection yaw.')
    if value.get('lidar_points',0)<p['minimum_lidar_points']:raise ValueError('Insufficient LiDAR support.')
    if not .8<=value.get('confidence',0)<=1.001:raise ValueError('Low shape confidence.')


class DropVerifier:
    """Require independent, stationary simulator samples inside the bin.

This is simulation scoring, not part of object detection.
"""
    def __init__(self,centre,inner,floor,height,size,shape):
        self.centre=centre;self.inner=inner;self.floor=floor;self.height=height
        self.margin=(max(size[:2]) if shape=='cylinder' else math.hypot(*size[:2]))/2
        self.previous=None;self.since=None

    def update(self,xyz,stamp):
        if xyz is None:return False
        xyz=np.asarray(xyz,dtype=float)
        if xyz.shape!=(3,) or not np.isfinite(xyz).all():return False
        inside=(abs(xyz[0]-self.centre[0])<self.inner/2-self.margin and
                abs(xyz[1]-self.centre[1])<self.inner/2-self.margin and self.floor<=xyz[2]<self.height)
        if self.previous is not None:
            previous,when=self.previous
            if stamp<=when:return False
            if np.linalg.norm(xyz-previous)/(stamp-when)>.01:inside=False
        self.previous=(xyz,stamp)
        if not inside:self.since=None;return False
        if self.since is None:self.since=stamp
        return stamp-self.since>=.4
