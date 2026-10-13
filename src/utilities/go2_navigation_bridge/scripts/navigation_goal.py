"""Bounded navigation request bookkeeping; no locomotion FSM or ROS dependency."""
import math


class GoalTracker:
    def __init__(self, tolerance=.25, dwell_s=.5):
        if any(not math.isfinite(v) or v <= 0 for v in (tolerance, dwell_s)):
            raise ValueError('Goal tolerance and dwell must be finite and positive')
        self.tolerance = tolerance
        self.dwell_s = dwell_s
        self.revision = 0
        self.reset()

    def reset(self):
        self.target = None
        self.frame = ''
        self.stamp_ns = 0
        self.started = None
        self.finished = None
        self.state = 'IDLE'
        self.error_xy = None
        self.inside_since = None
        self.far_reason = ''

    def accept(self, target, frame, stamp_ns, now):
        target = tuple(target)
        if len(target) != 3 or not all(math.isfinite(x) for x in target):
            return False
        if stamp_ns <= self.stamp_ns:
            return False
        # Retransmission must not reset a running or completed request.
        if target == self.target and frame == self.frame:
            return False
        self.revision += 1
        self.target, self.frame, self.stamp_ns = target, frame, stamp_ns
        self.started, self.finished = now, None
        self.state, self.error_xy, self.inside_since = 'REQUESTED', None, None
        self.far_reason = ''
        return True

    def active(self):
        if self.state == 'REQUESTED':
            self.state = 'ACTIVE'

    def position(self, x, y, now, allow_local_completion=False):
        if self.target is None or not all(math.isfinite(v) for v in (x, y)):
            return
        self.error_xy = math.hypot(x-self.target[0], y-self.target[1])
        if self.state not in ('REQUESTED', 'ACTIVE'):
            return
        if not allow_local_completion:
            self.inside_since = None
            return
        if self.error_xy <= self.tolerance:
            if self.inside_since is None:
                self.inside_since = now
            if allow_local_completion and now-self.inside_since >= self.dwell_s:
                self.finish('SUCCEEDED', now)
        else:
            self.inside_since = None

    def far_status(self, values, stamp_ns, now):
        if self.target is None or stamp_ns < self.stamp_ns or self.state not in ('REQUESTED', 'ACTIVE'):
            return False
        try:
            original = tuple(float(values['goal_original_'+axis]) for axis in 'xyz')
            if any(not math.isfinite(x) for x in original) or any(abs(x-y) > 1e-4 for x,y in zip(original,self.target)):
                return False
        except (KeyError, ValueError, TypeError):
            return False
        self.far_reason = values.get('reason_code', '')
        if self.far_reason == 'GOAL_REACHED':
            # Independently check current map->vehicle position; do not trust a
            # latched diagnostic from a former planner session as proof of arrival.
            if self.error_xy is not None and self.error_xy <= self.tolerance:
                self.finish('SUCCEEDED', now)
                return True
        elif self.far_reason == 'ADJUSTED_GOAL_OUTSIDE_TOLERANCE':
            self.finish('STOPPED_ADJUSTED', now)
            return True
        return False

    def finish(self, state, now):
        self.state, self.finished = state, now

    def snapshot(self, now):
        return dict(revision=self.revision, state=self.state, frame=self.frame,
                    requested=list(self.target) if self.target else None,
                    error_xy_m=self.error_xy, tolerance_m=self.tolerance,
                    elapsed_s=None if self.started is None else (self.finished or now)-self.started,
                    far_reason=self.far_reason)
