"""Velocity request gate; does not implement locomotion modes or robot commands."""
import math
from navigation_recovery import LocalizationRecovery

ZERO = (0.0, 0.0, 0.0)

class NavigationGate:
    def __init__(self, mode='stack', command_timeout=.25, sensor_timeout=.5, limits=(.2, .1, .1), clock_required=False, recovery_samples=None):
        if mode not in ('stack', 'command_test'):
            raise ValueError('mode must be stack or command_test')
        if not all(math.isfinite(x) and x > 0 for x in (command_timeout, sensor_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        if len(limits) != 3 or any(not math.isfinite(x) or x <= 0 or x > bound for x,bound in zip(limits,(.2,.1,.1))):
            raise ValueError('Limits must fit the R3 NAV contract')
        self.limits = tuple(limits)
        self.mode = mode
        self.command_timeout = command_timeout
        self.sensor_timeout = sensor_timeout
        self.health = {}
        self.cancelled = False
        self.request = ZERO
        self.request_received = None
        self.request_header_age = 0.
        self.active = False
        self.active_received = None
        self.locomotion_ready = False
        self.locomotion_received = None
        self.goal_received = False
        self.block_reason = 'startup'
        self.clock_required = clock_required
        self.clock_valid = False
        self.clock_received = None
        self.clock_failed = False
        self.clock_seen_locked = False
        self.recovery=LocalizationRecovery(recovery_samples) if recovery_samples is not None else None
        self.localization_was_ready=False;self.goal_invalidated=False

    @staticmethod
    def fresh(now, stamp, ttl):
        return stamp is not None and 0 <= now - stamp <= ttl

    def observe(self, name, valid, now, header_age=0.):
        valid = valid and math.isfinite(header_age) and header_age >= 0
        self.health[name] = now - header_age if valid else None
        if not valid:
            self.clear_request()

    def observe_alignment(self, state, valid, now, header_age=0.):
        valid = valid and state == 'LOCKED' and math.isfinite(header_age) and 0 <= header_age <= self.sensor_timeout
        self.clock_valid = bool(valid)
        self.clock_received = now - header_age if valid else None
        if valid:
            self.clock_seen_locked = True
        elif self.clock_seen_locked or state in ('DEGRADED', 'FAULT'):
            self.clock_failed = True
            self.cancel()  # immutable session failure, resume() cannot rearm it

    def clock_ready(self, now):
        if not self.clock_required:
            return True
        if self.clock_seen_locked and not self.fresh(now, self.clock_received, self.sensor_timeout):
            self.clock_failed = True
            self.cancel()
        return not self.clock_failed and self.clock_valid and self.fresh(now, self.clock_received, self.sensor_timeout)

    def localization_ready(self, now):
        healthy=self.clock_ready(now) and (self.mode == 'command_test' or all(
            self.fresh(now, self.health.get(name), self.sensor_timeout)
            for name in ('lidar', 'imu', 'odom', 'tf')))
        if not healthy:
            if self.recovery:self.recovery.invalidate()
            if self.localization_was_ready and self.goal_received:
                self.goal_invalidated=True;self.goal_received=False;self.health['path']=None;self.clear_request()
            return False
        ready=healthy and (self.recovery is None or self.recovery.ready)
        if ready:self.localization_was_ready=True
        return ready

    def observe_pair(self, stamp_ns, valid):
        if self.recovery:return self.recovery.observe(stamp_ns,valid)
        return valid

    def goal(self):
        self.goal_received = True

    def planner_active(self, active, now):
        self.active = bool(active)
        self.active_received = now
        if not active:
            self.clear_request()

    def locomotion(self, ready, now):
        self.locomotion_ready = bool(ready)
        self.locomotion_received = now
        if not ready:
            self.clear_request()

    def clear_request(self):
        self.request = ZERO
        self.request_received = None

    def invalidate_goal(self):
        self.goal_received=False
        self.goal_invalidated=True
        self.health['path']=None
        self.clear_request()

    def cancel(self):
        self.cancelled = True
        self.goal_received = False
        self.clear_request()

    def resume(self):
        self.cancelled = False
        self.goal_received = False
        self.health['path'] = None
        self.clear_request()  # never replay a pre-cancel command

    def enabled(self, now):
        if self.cancelled:
            self.block_reason = 'cancelled'
        elif not self.localization_ready(now):
            self.block_reason = 'localization_unavailable'
        elif not self.locomotion_ready or not self.fresh(now, self.locomotion_received, self.sensor_timeout):
            self.block_reason = 'locomotion_not_active'
        elif self.mode == 'stack' and not self.goal_received:
            self.block_reason = 'new_goal_required'
        elif self.mode == 'stack' and not (self.active and self.fresh(now, self.active_received, self.sensor_timeout)):
            self.block_reason = 'planner_inactive'
        elif self.mode == 'stack' and not self.fresh(now, self.health.get('path'), self.sensor_timeout):
            self.block_reason = 'path_unavailable'
        elif self.health.get('stop') is None:
            self.block_reason = 'planner_stop'
        else:
            self.block_reason = ''
            return True
        return False

    def accept(self, value, header_age, frame, now):
        self.clear_request()
        if not self.enabled(now) or frame != 'vehicle' or not math.isfinite(header_age) or not 0 <= header_age <= self.command_timeout:
            return False
        if len(value) != 3 or not all(math.isfinite(x) for x in value):
            return False
        self.request = tuple(max(-limit, min(limit, x)) for x, limit in zip(value, self.limits))
        self.request_received = now
        self.request_header_age = header_age
        return True

    def output(self, now):
        enabled = self.enabled(now)
        if not enabled:
            self.clear_request()
        if not enabled or not self.fresh(now, self.request_received, self.command_timeout - self.request_header_age):
            return ZERO
        return self.request
