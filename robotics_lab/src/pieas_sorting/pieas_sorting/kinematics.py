"""URDF forward kinematics and bounded, upright suction IK.

Suction does not require a particular yaw. We solve XYZ + the tool Z direction,
then ask MoveIt to validate the candidate and plan the collision-free path.
This numerical solver does NOT perform collision checking itself.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS = [f'joint_{i}' for i in range(1, 7)]


class Kinematics:
    def __init__(self, description, rail_limit=0.16):
        root = ET.fromstring(description)
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        link = 'tool0'
        while link != 'world':
            joint = by_child[link]
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = Rotation.from_euler('xyz',
                    list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
                transform[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
            axis = joint.find('axis')
            axis = np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else None
            self.chain.append((joint.get('name'), joint.get('type'), transform, axis))
            link = joint.find('parent').get('link')
        self.chain.reverse()
        limits = [root.find(f"joint[@name='{name}']/limit") for name in JOINTS]
        # Additional factory operating envelope. These are not hardware ratings.
        self.lower = np.array([max(float(l.get('lower', '-3.14')), -rail_limit if i == 0 else -2.8)
                               for i, l in enumerate(limits)])
        self.upper = np.array([min(float(l.get('upper', '3.14')), rail_limit if i == 0 else 2.8)
                               for i, l in enumerate(limits)])

    def forward(self, q):
        transforms = dict(zip(JOINTS, q))
        result = np.eye(4)
        for name, kind, origin, axis in self.chain:
            result = result @ origin
            motion = np.eye(4)
            if kind == 'prismatic':
                motion[:3, 3] = axis * transforms[name]
            elif kind in ('revolute', 'continuous'):
                motion[:3, :3] = Rotation.from_rotvec(axis * transforms[name]).as_matrix()
            result = result @ motion
        return result

    def candidates(self, xyz, current, attempts=16):
        """Yield finite, bounded solutions; deterministic seeds make tests repeatable."""
        xyz, current = np.asarray(xyz), np.asarray(current)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError('Target XYZ must be three finite metres.')
        rng = np.random.default_rng(42)
        seeds = [np.clip(current, self.lower + 1e-6, self.upper - 1e-6)]
        seeds += [rng.uniform(self.lower, self.upper) for _ in range(attempts - 1)]
        seen = []
        for seed in seeds:
            def residual(q):
                t = self.forward(q)
                # tool0 +Z points UP in this model; the suction face points -Z.
                # Direction vector has two independent DOFs, leaving yaw free.
                return np.r_[t[:3, 3] - xyz, 0.15 * (t[:3, 2] - [0, 0, 1])]
            result = least_squares(residual, seed, bounds=(self.lower, self.upper),
                                   max_nfev=160, ftol=1e-9, xtol=1e-9, gtol=1e-9)
            t = self.forward(result.x)
            if np.linalg.norm(t[:3, 3] - xyz) > 0.0008 or t[2, 2] < np.cos(0.04):
                continue
            if any(np.linalg.norm(result.x - old) < 0.05 for old in seen):
                continue
            seen.append(result.x)
            yield result.x
