"""Bounded navigation request bookkeeping; no locomotion FSM or ROS dependency."""
import math


class GoalTracker:
    def __init__(self, tolerance=.25, dwell_s=.5, *, far_mode=False, wait_budget_s=10., settle_position_m=.02, settle_yaw_rad=.05, settle_max_gap_s=.25):
        if any(not math.isfinite(v) or v <= 0 for v in (tolerance, dwell_s)):
            raise ValueError('Goal tolerance and dwell must be finite and positive')
        for v in (wait_budget_s, settle_position_m, settle_yaw_rad, settle_max_gap_s):
            if type(v) not in (float,int) or not math.isfinite(v) or v<=0:
                raise ValueError('Goal waiting/settling limits must be finite and positive')
        self.settle_max_gap_s=settle_max_gap_s
        self.last_measurement=None
        self.far_mode=far_mode
        self.wait_budget_s=wait_budget_s
        self.settle_position_m=settle_position_m
        self.settle_yaw_rad=settle_yaw_rad
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
        self.last_measurement=None
        self.wait_spent=0.
        self.wait_updated=None
        self.last_far_stamp=0
        self.last_pose_stamp=0
        self.settle_anchor=None
        self.hold_since=None
        self.adjusted=None
        self.correction_reason=''

    def accept(self, target, frame, stamp_ns, now):
        target = tuple(target)
        if len(target) != 3 or not all(math.isfinite(x) for x in target):
            return False
        if stamp_ns <= self.stamp_ns:
            return False
        # Retransmission must not reset a running or completed request.
        if target == self.target and frame == self.frame and self.state not in ('LOCALIZATION_LOST','GOAL_UNREACHABLE','AUTHORITY_LOST','CONTINUATION_NOT_FOUND','ARRIVAL_NOT_CONFIRMED'):
            return False
        self.revision += 1
        self.target, self.frame, self.stamp_ns = target, frame, stamp_ns
        self.started, self.finished = now, None
        self.state, self.error_xy, self.inside_since = 'REQUESTED', None, None
        self.far_reason = ''
        self.last_measurement=None
        self.wait_spent=0.;self.wait_updated=now;self.last_far_stamp=0;self.last_pose_stamp=0
        self.settle_anchor=None;self.hold_since=None;self.adjusted=None;self.correction_reason=''
        return True

    def active(self):
        if self.state == 'REQUESTED':
            self.state = 'ACTIVE'

    def advance(self, now):
        if self.wait_updated is not None and self.state in ('REPLANNING','ARRIVAL_CONFIRMING'):
            self.wait_spent += max(0.,now-self.wait_updated)
        self.wait_updated=now
        if self.state in ('REPLANNING','ARRIVAL_CONFIRMING') and self.wait_spent >= self.wait_budget_s:
            self.finish('ARRIVAL_NOT_CONFIRMED' if self.state=='ARRIVAL_CONFIRMING' else 'CONTINUATION_NOT_FOUND',now)

    @property
    def paused(self):
        return self.state in ('REPLANNING','ARRIVAL_CONFIRMING')

    def position(self, x, y, now, allow_local_completion=False, *, stamp_ns=0,
                 measurement_time=None, yaw=0., stopped_time=None):
        if self.target is None or not all(math.isfinite(v) for v in (x, y)):
            self.inside_since=None;self.settle_anchor=None
            return
        self.error_xy = math.hypot(x-self.target[0], y-self.target[1])
        if self.far_mode:
            self._far_position(x,y,yaw,now,stamp_ns,measurement_time,allow_local_completion,stopped_time)
            return
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

    def _far_position(self,x,y,yaw,now,stamp_ns,measurement_time,allowed,stopped_time):
        self.advance(now)
        if self.state not in ('REQUESTED','ACTIVE','REPLANNING','ARRIVAL_CONFIRMING'):
            return
        if not allowed or not math.isfinite(yaw) or type(stamp_ns) is not int or stamp_ns<=0:
            self.inside_since=None;self.settle_anchor=None
            return
        if stamp_ns<=self.last_pose_stamp:
            # Point-LIO may revise a pose at the same measurement stamp. A
            # duplicate cannot advance dwell, but a changed pose must break it.
            anchor=self.settle_anchor
            changed=(self.error_xy>self.tolerance or stamp_ns<self.last_pose_stamp or
                (anchor is not None and (math.hypot(x-anchor[0],y-anchor[1])>self.settle_position_m or
                abs(math.atan2(math.sin(yaw-anchor[2]),math.cos(yaw-anchor[2])))>self.settle_yaw_rad)))
            if changed:
                self.inside_since=None;self.settle_anchor=None
                if self.error_xy>self.tolerance and self.state=='ARRIVAL_CONFIRMING':
                    self.state='REPLANNING';self.hold_since=None
            return
        self.last_pose_stamp=stamp_ns
        measured=now if measurement_time is None else measurement_time
        if not math.isfinite(measured) or measured>now:
            self.inside_since=None;self.settle_anchor=None
            return
        if self.last_measurement is None or not 0<measured-self.last_measurement<=self.settle_max_gap_s:
            self.inside_since=None;self.settle_anchor=None
        self.last_measurement=measured
        if self.error_xy>self.tolerance:
            self.inside_since=None;self.settle_anchor=None
            if self.state=='ARRIVAL_CONFIRMING':
                self.state='REPLANNING';self.hold_since=None
            return
        if self.state!='ARRIVAL_CONFIRMING':
            self.state='ARRIVAL_CONFIRMING';self.hold_since=now
            self.inside_since=None;self.settle_anchor=None
        if stopped_time is None or not self.hold_since<=stopped_time<=now:
            self.inside_since=None;self.settle_anchor=None
            return
        anchor=self.settle_anchor
        moved=(anchor is None or self.inside_since is None or math.hypot(x-anchor[0],y-anchor[1])>self.settle_position_m
               or abs(math.atan2(math.sin(yaw-anchor[2]),math.cos(yaw-anchor[2])))>self.settle_yaw_rad)
        if moved:
            self.settle_anchor=(x,y,yaw);self.inside_since=measured
        elif measured-self.inside_since>=self.dwell_s:
            self.finish('SUCCEEDED',now)

    def far_status(self, values, stamp_ns, now):
        self.advance(now)
        if self.target is None or stamp_ns < self.stamp_ns or (self.far_mode and stamp_ns<=self.last_far_stamp) or self.state not in ('REQUESTED','ACTIVE','REPLANNING','ARRIVAL_CONFIRMING'):
            return False
        try:
            original = tuple(float(values['goal_original_'+axis]) for axis in 'xyz')
            if any(not math.isfinite(x) for x in original) or any(abs(x-y) > 1e-4 for x,y in zip(original,self.target)):
                return False
            if self.far_mode and int(values.get('goal_stamp_ns',0))!=self.stamp_ns:
                return False
        except (KeyError, ValueError, TypeError):
            return False
        self.last_far_stamp=stamp_ns
        self.far_reason = values.get('reason_code', '')
        self.correction_reason=values.get('goal_correction_reason','')
        try:
            adjusted=[float(values['goal_current_'+a]) for a in 'xyz']
            if all(math.isfinite(v) for v in adjusted):self.adjusted=adjusted
        except (KeyError,ValueError,TypeError):pass
        if self.far_mode:
            if self.far_reason in ('GOAL_REACHED','REQUESTED_GOAL_HOLD'):
                if self.state!='ARRIVAL_CONFIRMING':
                    self.state='REPLANNING' # Only measured original-goal arrival starts confirmation.
            elif self.far_reason in ('ADJUSTED_GOAL_OUTSIDE_TOLERANCE','ADJUSTED_GOAL_WAIT',
                    'CORRECTION_REJECTED','CORRECTION_UNSTABLE','START_OR_GOAL_INVALID','START_OR_GRAPH_UNAVAILABLE') or values.get('state')=='REPLANNING_STOP':
                if self.state!='ARRIVAL_CONFIRMING':self.state='REPLANNING'
            elif self.far_reason=='ROUTE_COMMITTED' and self.state!='ARRIVAL_CONFIRMING':
                self.state='ACTIVE'
            return False
        if self.far_reason == 'GOAL_REACHED':
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
        return dict(revision=self.revision, stamp_ns=self.stamp_ns, state=self.state, frame=self.frame,
                    requested=list(self.target) if self.target else None,
                    error_xy_m=self.error_xy, tolerance_m=self.tolerance,
                    elapsed_s=None if self.started is None else (self.finished or now)-self.started,
                    far_reason=self.far_reason, adjusted=self.adjusted, correction_reason=self.correction_reason,
                    wait_spent_s=self.wait_spent, wait_budget_s=self.wait_budget_s,
                    arrival_confirmed=self.state=='SUCCEEDED')
