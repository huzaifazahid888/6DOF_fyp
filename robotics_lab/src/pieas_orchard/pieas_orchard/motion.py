"""Feedback settling checks shared by the task and CPU tests."""
import math


class StationaryWindow:
    """Require new feedback below velocity limits for a continuous time window."""
    def __init__(self, after, duration=.2):
        self.after=after
        self.duration=duration
        self.last=None
        self.since=None

    def update(self, stamp, velocities):
        if not math.isfinite(stamp) or stamp<=self.after:
            return False
        if self.last is not None and stamp<=self.last:
            return False
        if self.last is not None and stamp-self.last>.15:
            self.since=None
        self.last=stamp
        limits=[.002,.01,.01,.01,.01,.01]
        if len(velocities)!=6 or any(not math.isfinite(v) or abs(v)>limit for v,limit in zip(velocities,limits)):
            self.since=None
            return False
        if self.since is None:self.since=stamp
        return stamp-self.since>=self.duration


def rank_configurations(candidates,current,continuous_indices=()):
    """Prefer less joint travel; rail travel is normalized by 0.1 metre.

    This ranks IK endpoints only. MoveIt still checks collisions and plans paths.
    """
    import numpy as np
    current=np.asarray(current,dtype=float)
    def cost(candidate):
        delta=np.asarray(candidate,dtype=float)-current
        for i in continuous_indices:delta[i]=math.atan2(math.sin(delta[i]),math.cos(delta[i]))
        delta[0]/=.1
        delta[-1]*=.3  # Suction allows wrist yaw; prioritize the load-bearing arm.
        return float(delta@delta)
    return sorted(candidates,key=cost)
