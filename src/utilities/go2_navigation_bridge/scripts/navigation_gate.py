"""Velocity request gate; does not implement locomotion modes or robot commands."""
import math

ZERO = (0.0, 0.0, 0.0)

class NavigationGate:
    def __init__(self, mode='stack', command_timeout=.25, sensor_timeout=.5):
        if mode not in ('stack', 'command_test'):
            raise ValueError('mode must be stack or command_test')
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

    @staticmethod
    def fresh(now, stamp, ttl):
        return stamp is not None and 0 <= now - stamp <= ttl

    def observe(self, name, valid, now, header_age=0.):
        valid = valid and math.isfinite(header_age) and header_age >= 0
        self.health[name] = now - header_age if valid else None
        if not valid:
            self.clear_request()

    def localization_ready(self, now):
        return self.mode == 'command_test' or all(
            self.fresh(now, self.health.get(name), self.sensor_timeout)
            for name in ('lidar', 'imu', 'odom', 'tf'))

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
        elif not self.locomotion_ready or not self.fresh(now, self.locomotion_received, .5):
            self.block_reason = 'locomotion_not_active'
        elif self.mode == 'stack' and not self.goal_received:
            self.block_reason = 'new_goal_required'
        elif self.mode == 'stack' and not (self.active and self.fresh(now, self.active_received, .5)):
            self.block_reason = 'planner_inactive'
        elif self.mode == 'stack' and not self.fresh(now, self.health.get('path'), .5):
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
        limits = (.2, .1, .1)
        self.request = tuple(max(-limit, min(limit, x)) for x, limit in zip(value, limits))
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
