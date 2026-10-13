"""Bounded, ROS-independent reconciliation of odometry and actual TF revisions.

Arrival time never renews measurement freshness. A verified vehicle pose is
cached at its original stamp so asynchronous updates cannot mix TF epochs.
"""
from bisect import bisect_left, insort
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class PairingConfig:
    max_age_s: float = .5
    history_s: float = 1.
    max_samples: int = 2048  # per stream, including equal-stamp revisions
    position_tolerance_m: float = .001
    quaternion_tolerance: float = .001

    def __post_init__(self):
        for value in (self.max_age_s, self.history_s, self.position_tolerance_m,
                      self.quaternion_tolerance):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Pairing durations/tolerances must be finite and positive')
        if self.history_s < self.max_age_s:
            raise ValueError('Pair history must cover the sensor freshness deadline')
        if type(self.max_samples) is not int or self.max_samples < 1:
            raise ValueError('Pair history sample cap must be a positive integer')


@dataclass(frozen=True, slots=True)
class PoseSample:
    stamp_ns: int
    frame: str
    child: str
    position: tuple
    quaternion: tuple

    def valid(self):
        return (type(self.stamp_ns) is int and self.stamp_ns > 0 and
                bool(self.frame) and bool(self.child) and len(self.position) == 3 and
                len(self.quaternion) == 4 and
                all(math.isfinite(v) for v in (*self.position, *self.quaternion)) and
                .5 < sum(v*v for v in self.quaternion) < 1.5)


def pose_errors(a, b):
    """Component-wise translation and sign-invariant quaternion errors."""
    position = max(abs(x-y) for x, y in zip(a.position, b.position))
    quaternion = min(max(abs(x-y) for x, y in zip(a.quaternion, b.quaternion)),
                     max(abs(x+y) for x, y in zip(a.quaternion, b.quaternion)))
    return position, quaternion


class PoseHistory:
    """Compact immutable poses; ordered stamps and bounded revision buckets."""
    def __init__(self, config):
        self.config = config
        self.stamps = []
        self.buckets = {}
        self.count = 0

    def clear(self):
        self.stamps.clear()
        self.buckets.clear()
        self.count = 0

    def prune(self, cutoff_ns):
        index = bisect_left(self.stamps, cutoff_ns)
        for stamp in self.stamps[:index]:
            self.count -= len(self.buckets.pop(stamp))
        del self.stamps[:index]

    def observe(self, pose):
        high_water = max(pose.stamp_ns, self.stamps[-1] if self.stamps else 0)
        cutoff = high_water - int(self.config.history_s * 1e9)
        self.prune(cutoff)
        if pose.stamp_ns < cutoff:
            return
        if pose.stamp_ns not in self.buckets:
            insort(self.stamps, pose.stamp_ns)
            self.buckets[pose.stamp_ns] = deque()
        self.buckets[pose.stamp_ns].append(pose)
        self.count += 1
        while self.count > self.config.max_samples:
            oldest = self.stamps[0]
            self.buckets[oldest].popleft()
            self.count -= 1
            if not self.buckets[oldest]:
                del self.buckets[oldest]
                del self.stamps[0]


class PairLookupError(Exception):
    def __init__(self, reason, detail=''):
        super().__init__(detail or reason)
        self.reason = reason


@dataclass(frozen=True, slots=True)
class LocalizationPair:
    odom: PoseSample
    tf: PoseSample
    vehicle: PoseSample
    position_error_m: float
    quaternion_error: float


class LocalizationPairing:
    def __init__(self, config):
        self.config = config
        self.odom = PoseHistory(config)
        self.tf = PoseHistory(config)
        self.selected = None
        self.last_now_ns = None
        self.reason = 'not_observed'
        self.waiting_reason = ''
        self.detail = ''
        self.fallback = False
        self.candidate_position_error_m = None
        self.candidate_quaternion_error = None

    def clear(self, reason='not_observed'):
        self.odom.clear()
        self.tf.clear()
        self.selected = None
        self.reason = reason
        self.waiting_reason = ''
        self.detail = ''
        self.fallback = False
        self.candidate_position_error_m = None
        self.candidate_quaternion_error = None

    def observe_odom(self, pose):
        if not pose.valid():
            self.clear('invalid_odometry')
            return
        self.odom.observe(pose)

    def observe_tf(self, pose):
        if not pose.valid():
            self.clear('invalid_tf')
            return
        self.tf.observe(pose)

    def _fresh(self, stamp_ns, now_ns):
        return 0 <= now_ns - stamp_ns <= int(self.config.max_age_s * 1e9)

    def select(self, now_ns, resolve_vehicle):
        """Resolve only exact stamps received on BOTH streams, newest first.

        resolve_vehicle(odom, tf) must verify the current TF-buffer revision
        and resolve the complete map->vehicle chain at this exact stamp. It
        must never return a latest-time transform or block waiting for data.
        """
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.clear('ros_clock_regressed')
            self.last_now_ns = now_ns
            return None
        self.last_now_ns = now_ns
        cutoff = now_ns - int(self.config.history_s * 1e9)
        self.odom.prune(cutoff)
        self.tf.prune(cutoff)
        self.candidate_position_error_m = None
        self.candidate_quaternion_error = None
        self.detail = ''
        fresh_odom = [s for s in self.odom.stamps if self._fresh(s, now_ns)]
        fresh_tf = {s for s in self.tf.stamps if self._fresh(s, now_ns)}
        def stream_reason(name, history):
            if not history.stamps:
                return name + '_missing'
            return name + ('_future' if history.stamps[-1] > now_ns else '_stale')
        reason = (stream_reason('odometry', self.odom) if not fresh_odom else
                  stream_reason('tf', self.tf) if not fresh_tf else 'no_common_timestamp')
        newest = max(fresh_odom[-1] if fresh_odom else 0, max(fresh_tf, default=0))
        for stamp in reversed(fresh_odom):
            if self.selected and stamp < self.selected.odom.stamp_ns:
                break  # retain the newer cached pair rather than roll back
            if stamp not in fresh_tf:
                continue
            # TF buffer retains the last received revision at a repeated stamp.
            tf = self.tf.buckets[stamp][-1]
            for odom in reversed(self.odom.buckets[stamp]):
                if (odom.frame, odom.child) != (tf.frame, tf.child):
                    reason = 'pair_frame_mismatch'
                    continue
                pe, qe = pose_errors(odom, tf)
                if self.candidate_position_error_m is None:
                    self.candidate_position_error_m, self.candidate_quaternion_error = pe, qe
                if pe >= self.config.position_tolerance_m or qe >= self.config.quaternion_tolerance:
                    reason = 'pair_pose_mismatch'
                    continue
                try:
                    vehicle = resolve_vehicle(odom, tf)
                except PairLookupError as error:
                    reason, self.detail = error.reason, str(error)
                    continue
                if (not vehicle.valid() or vehicle.stamp_ns != stamp or
                        vehicle.frame != 'map' or vehicle.child != 'vehicle'):
                    reason = 'invalid_vehicle_transform'
                    continue
                pair = LocalizationPair(odom, tf, vehicle, pe, qe)
                # A late packet must never roll back an already verified pose.
                if self.selected is None or stamp >= self.selected.odom.stamp_ns:
                    self.selected = pair
                self.reason = ''
                self.fallback = self.selected.odom.stamp_ns < newest
                self.waiting_reason = (reason if reason not in ('no_common_timestamp', '')
                                       else 'waiting_for_newer_pair') if self.fallback else ''
                return self.selected
        self.waiting_reason = reason
        if self.selected and self._fresh(self.selected.odom.stamp_ns, now_ns):
            self.reason = ''
            self.fallback = True
            return self.selected
        self.reason = 'pair_expired' if self.selected else reason
        self.fallback = False
        self.selected = None
        return None

    def snapshot(self, now_ns):
        pair = self.selected
        fresh = bool(pair and self._fresh(pair.odom.stamp_ns, now_ns))
        odom_stamp = self.odom.stamps[-1] if self.odom.stamps else None
        tf_stamp = self.tf.stamps[-1] if self.tf.stamps else None
        return dict(reason=('pair_expired' if pair and not fresh else self.reason), waiting_reason=self.waiting_reason,
                    detail=self.detail, using_previous_pair=self.fallback,
                    selected_stamp_ns=pair.odom.stamp_ns if pair else None,
                    selected_age_s=(now_ns-pair.odom.stamp_ns)/1e9 if pair else None,
                    selected_fresh=fresh,
                    position_error_m=pair.position_error_m if pair else None,
                    quaternion_error=pair.quaternion_error if pair else None,
                    candidate_position_error_m=self.candidate_position_error_m,
                    candidate_quaternion_error=self.candidate_quaternion_error,
                    odom_samples=self.odom.count, tf_samples=self.tf.count,
                    latest_odom_stamp_ns=odom_stamp, latest_tf_stamp_ns=tf_stamp,
                    latest_odom_age_s=(now_ns-odom_stamp)/1e9 if odom_stamp else None,
                    latest_tf_age_s=(now_ns-tf_stamp)/1e9 if tf_stamp else None)
