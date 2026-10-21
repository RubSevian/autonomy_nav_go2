"""Bounded recovery bookkeeping; no ROS, velocity publisher or motor owner."""
import math


class LocalizationRecovery:
    def __init__(self, required=5):
        if type(required) is not int or required<1:raise ValueError('positive recovery sample count required')
        self.required=required;self.good=0;self.last_stamp=0

    def invalidate(self):
        self.good=0

    def observe(self, stamp_ns, valid):
        if not valid or type(stamp_ns) is not int or stamp_ns<=0:
            self.invalidate();return False
        if stamp_ns<self.last_stamp:
            self.last_stamp=stamp_ns;self.invalidate();return False
        if stamp_ns==self.last_stamp:return self.ready
        self.last_stamp=stamp_ns;self.good=min(self.required,self.good+1)
        return self.ready

    @property
    def ready(self):return self.good>=self.required


class PathRecovery:
    def __init__(self, timeout_s=10.):
        if not math.isfinite(timeout_s) or timeout_s<=0:raise ValueError('positive path recovery timeout required')
        self.timeout_s=timeout_s;self.since=None;self.goal_key=None

    def reset(self):self.since=None;self.goal_key=None

    def check(self, now, *, goal_key, localization_valid, authority_valid, path_valid, path_source_time):
        if not localization_valid or not authority_valid:
            self.reset();return 'INVALIDATE_GOAL'
        if self.goal_key is not None and self.goal_key!=goal_key:
            self.reset();return 'INVALIDATE_GOAL'
        if self.since is None:
            if path_valid:return 'READY'
            self.since=now;self.goal_key=goal_key
            return 'WAITING_FOR_PATH'
        if now-self.since>=self.timeout_s:return 'GOAL_UNREACHABLE'
        if path_valid and path_source_time is not None and path_source_time>=self.since:
            self.reset();return 'RECOVERED'
        return 'WAITING_FOR_PATH'
