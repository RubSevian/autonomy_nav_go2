"""Pose-only motion evidence. No ROS, commands, arrival or budget authority.

An empirical stationary envelope is not a probability of physical motion.
Unknown calibration, stale data and discontinuities never confirm motion.
"""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionResult:
    reason: str
    displacement_m: float = 0.
    noise_bound_m: float | None = None
    duration_s: float = 0.
    samples: int = 0

    @property
    def confirmed(self):
        return self.reason == 'MEASURED_MOTION'


class MotionEvidence:
    """Compare robust endpoint positions against a calibrated window bound.

    noise_bound_m must describe this exact estimator/window, not individual
    pose sigma or map voxel size. Endpoint averages reduce isolated jitter;
    two successive, distinct endpoint windows must exceed the bound in a
    consistent direction. This is evidence only: route progress is separate.
    """
    def __init__(self, *, window_s=1., endpoint_s=.15, max_gap_s=.25,
                 max_age_s=.25, max_speed_mps=.5, noise_bound_m=None,
                 jump_slack_m=.02, capacity=2048):
        for v in (window_s, endpoint_s, max_gap_s, max_age_s, max_speed_mps, jump_slack_m):
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('Motion limits must be positive finite numbers')
        if window_s <= 2*endpoint_s:
            raise ValueError('Endpoint windows must not overlap')
        if noise_bound_m is not None and (type(noise_bound_m) not in (int,float)
                or not math.isfinite(noise_bound_m) or noise_bound_m <= 0):
            raise ValueError('Noise bound must be calibrated and positive, or None')
        if type(capacity) is not int or capacity < 6:
            raise ValueError('Motion history capacity must be at least six')
        self.window_s, self.endpoint_s = window_s, endpoint_s
        self.max_gap_s, self.max_age_s = max_gap_s, max_age_s
        self.max_speed_mps, self.jump_slack_m = max_speed_mps, jump_slack_m
        self.noise_bound_m, self.capacity = noise_bound_m, capacity
        self.reset()

    def reset(self, reason='WARMUP'):
        self.history = deque()
        self.candidate = None
        self.result = MotionResult(reason, noise_bound_m=self.noise_bound_m)

    def observe(self, x, y, stamp_ns, age_s):
        if (type(stamp_ns) is not int or stamp_ns <= 0 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y,age_s))):
            self.reset('INVALID_SAMPLE'); return self.result
        if not 0 <= age_s <= self.max_age_s:
            self.reset('STALE_SAMPLE'); return self.result
        previous = self.history[-1] if self.history else None
        if previous:
            dt = (stamp_ns-previous[0])/1e9
            step = math.hypot(x-previous[1],y-previous[2])
            if dt == 0 and step <= 1e-9:
                # A duplicate is not another independent confirmation.
                self.result=MotionResult('DUPLICATE_SAMPLE', noise_bound_m=self.noise_bound_m)
                return self.result
            if dt <= 0:
                self.reset('TIME_OR_POSE_REVISION'); return self.result
            if dt > self.max_gap_s:
                self.reset('MEASUREMENT_GAP')
            elif step > self.max_speed_mps*dt+self.jump_slack_m:
                self.reset('POSE_JUMP'); return self.result
        self.history.append((stamp_ns,x,y))
        # Retain a single sample before the requested horizon for irregular rates.
        horizon_ns = int((self.window_s+self.endpoint_s)*1e9)
        while len(self.history)>1 and stamp_ns-self.history[1][0] >= horizon_ns:
            self.history.popleft()
        if len(self.history)>self.capacity:
            self.reset('HISTORY_CAPACITY'); return self.result
        if self.noise_bound_m is None:
            self.result = MotionResult('UNCALIBRATED',samples=len(self.history))
            return self.result
        first = self.history[0][0]
        duration = (stamp_ns-first)/1e9
        if duration < self.window_s+self.endpoint_s:
            self.result = MotionResult('WARMUP',noise_bound_m=self.noise_bound_m,
                                       duration_s=duration,samples=len(self.history))
            return self.result
        early = [s for s in self.history if s[0]-first <= self.endpoint_s*1e9]
        late = [s for s in self.history if stamp_ns-s[0] <= self.endpoint_s*1e9]
        if min(len(early),len(late))<3:
            self.candidate=None
            self.result=MotionResult('INSUFFICIENT_SAMPLES',noise_bound_m=self.noise_bound_m)
            return self.result
        dx=sum(s[1] for s in late)/len(late)-sum(s[1] for s in early)/len(early)
        dy=sum(s[2] for s in late)/len(late)-sum(s[2] for s in early)/len(early)
        distance=math.hypot(dx,dy)
        reason='WITHIN_NOISE_ENVELOPE'
        if distance>self.noise_bound_m:
            reason='CANDIDATE_MOTION'
            if self.candidate:
                t,cx,cy=self.candidate
                # Wait for a distinct endpoint window, and require positive
                # directional agreement. Repeated callbacks alone cannot confirm.
                if stamp_ns-t >= self.endpoint_s*1e9 and dx*cx+dy*cy>0:
                    reason='MEASURED_MOTION'
                elif dx*cx+dy*cy<=0:
                    self.candidate=None
            if self.candidate is None:self.candidate=(stamp_ns,dx,dy)
        else:
            self.candidate=None
        self.result=MotionResult(reason,distance,self.noise_bound_m,duration,len(self.history))
        return self.result
