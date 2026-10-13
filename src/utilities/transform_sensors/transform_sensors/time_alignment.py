"""Bounded, ROS-independent alignment of a shared LiDAR/IMU clock.

One-way traffic cannot identify absolute transport latency.  The smallest
observed receive-minus-measurement-end offset is an estimate, with a configured
margin.  It is frozen before publication so measurement intervals are preserved.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AlignmentConfig:
    warmup_s: float = 2.0
    margin_s: float = 0.02
    max_age_s: float = 0.5
    clock_jump_s: float = 0.1
    max_scan_duration_s: float = 0.2
    min_cloud_samples: int = 5
    min_imu_samples: int = 100

    def __post_init__(self):
        for name in ('warmup_s', 'margin_s', 'max_age_s', 'clock_jump_s', 'max_scan_duration_s'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        for name in ('min_cloud_samples', 'min_imu_samples'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(name + ' must be a positive integer')
        if self.margin_s + self.max_scan_duration_s >= self.max_age_s:
            raise ValueError('margin + max scan duration must be smaller than max age')


class SensorTimeAlignment:
    """One session, one immutable offset, O(1) state; faults require restart.

    Callbacks supply receive ROS/monotonic time before processing.  Cloud spans
    cover all original points, including points later removed by self-filtering.
    A fault stops both streams, avoiding a discontinuity inside an active LIO.
    """

    def __init__(self, config=AlignmentConfig()):
        self.config = config
        self.state = 'WARMUP'
        self.reason = 'collecting_clock_samples'
        self.offset_ns = None
        self._first_mono = None
        self._last_receive = None
        self._minimum_offset = None
        self._last_source = {}
        self.seen = dict(cloud=0, imu=0)
        self.accepted = dict(cloud=0, imu=0)
        self.dropped = dict(cloud=0, imu=0)
        self.last_age_ns = dict(cloud=None, imu=None)

    def _drop(self, stream, reason, fault=False):
        self.dropped[stream] += 1
        if self.state != 'FAULT':
            self.reason = reason
            if fault:
                self.state = 'FAULT'
        return None

    def map_stamp(self, stream, source_ns, receive_ros_ns, receive_mono_ns, span_ns=0):
        """Return mapped acquisition-start ns, or None during warmup/rejection."""
        if stream not in self.seen:
            raise ValueError('Unknown sensor stream: ' + str(stream))
        if self.state == 'FAULT':
            return self._drop(stream, self.reason)
        if (any(type(v) is not int for v in (source_ns, receive_ros_ns, receive_mono_ns, span_ns))
                or source_ns <= 0 or receive_ros_ns <= 0 or receive_mono_ns < 0
                or span_ns < 0 or (stream == 'imu' and span_ns != 0)
                or span_ns > self.config.max_scan_duration_s * 1e9):
            return self._drop(stream, 'invalid_measurement_time')

        if self._last_receive is not None:
            old_ros, old_mono = self._last_receive
            if (receive_mono_ns < old_mono or
                    abs((receive_ros_ns - old_ros) - (receive_mono_ns - old_mono))
                    > self.config.clock_jump_s * 1e9):
                return self._drop(stream, 'host_clock_jump', fault=True)
        self._last_receive = (receive_ros_ns, receive_mono_ns)
        previous = self._last_source.get(stream)
        if previous is not None and source_ns <= previous:
            return self._drop(stream, 'sensor_clock_reset' if previous - source_ns > self.config.max_age_s * 1e9
                              else 'duplicate_or_out_of_order',
                              fault=previous - source_ns > self.config.max_age_s * 1e9)
        self._last_source[stream] = source_ns
        self.seen[stream] += 1

        if self.state == 'WARMUP':
            if self._first_mono is None:
                self._first_mono = receive_mono_ns
            candidate = receive_ros_ns - source_ns - span_ns
            self._minimum_offset = candidate if self._minimum_offset is None else min(self._minimum_offset, candidate)
            if (receive_mono_ns - self._first_mono < self.config.warmup_s * 1e9
                    or self.seen['cloud'] < self.config.min_cloud_samples
                    or self.seen['imu'] < self.config.min_imu_samples):
                return self._drop(stream, 'collecting_clock_samples')
            self.offset_ns = self._minimum_offset - round(self.config.margin_s * 1e9)
            self.state = 'LOCKED'

        mapped = source_ns + self.offset_ns
        age = receive_ros_ns - mapped
        self.last_age_ns[stream] = age
        if mapped <= 0 or age < span_ns:
            return self._drop(stream, 'measurement_in_future', fault=True)
        if age > self.config.max_age_s * 1e9:
            return self._drop(stream, 'measurement_stale')
        self.accepted[stream] += 1
        self.reason = ''
        return mapped

    def status(self):
        """A small snapshot, without retaining messages or per-sample history."""
        return dict(state=self.state, reason=self.reason, offset_ns=self.offset_ns,
                    seen=dict(self.seen), accepted=dict(self.accepted), dropped=dict(self.dropped),
                    last_age_s={k: v / 1e9 if v is not None else None for k, v in self.last_age_ns.items()},
                    absolute_transport_latency_known=False)
