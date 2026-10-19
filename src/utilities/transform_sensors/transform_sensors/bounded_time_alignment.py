"""Diagnostic common sensor-time map. No ROS, transport, motion, or file I/O.

One-way envelope tracking is NOT identification of clock frequency/network delay.
All epochs are integer ns. Rates are integer ppb; updates are continuous hinges
beyond every previously observed scan end. Uncertain mapping fails closed.
"""
from bisect import bisect_right
from collections import Counter, deque
from dataclasses import dataclass
from statistics import median
import math

from transform_sensors.time_alignment import AlignmentConfig, SensorTimeAlignment

def finite_imu_fields(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


NS = 1_000_000_000
I64_MAX = (1 << 63) - 1


def div_round(n, d):
    return (n + d // 2) // d if n >= 0 else -((-n + d // 2) // d)


@dataclass(frozen=True)
class BoundedConfig(AlignmentConfig):
    envelope_window_s: float = 1.0
    envelope_history: int = 8
    segment_capacity: int = 16
    max_rate_ppm: float = 1000.0
    rate_slew_ppm_s: float = 100.0
    max_uncertainty_s: float = .005
    small_future_ns: int = 2000
    max_small_future_drops: int = 3
    envelope_min_cloud_samples: int = 5
    envelope_min_imu_samples: int = 100
    max_skipped_envelope_windows: int = 1
    max_out_of_bound_rate_windows: int = 1

    def __post_init__(self):
        super().__post_init__()
        for key in ('envelope_min_cloud_samples', 'envelope_min_imu_samples', 'max_skipped_envelope_windows',
                    'max_out_of_bound_rate_windows'):
            if type(getattr(self, key)) is not int or getattr(self, key) < 1:
                raise ValueError(key)
        if self.max_skipped_envelope_windows > 2 or self.max_out_of_bound_rate_windows > 2:
            raise ValueError('at most two uncertain clock windows may be held')
        for key in ('envelope_window_s', 'max_rate_ppm', 'rate_slew_ppm_s', 'max_uncertainty_s'):
            v = getattr(self, key)
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError(key)
        for key in ('envelope_history', 'segment_capacity', 'small_future_ns', 'max_small_future_drops'):
            if type(getattr(self, key)) is not int or getattr(self, key) < 1:
                raise ValueError(key)
        # Hard diagnostic bounds prevent configuration from bypassing guards.
        if (not 4 <= self.envelope_history <= 16 or not 8 <= self.segment_capacity <= 32
                or not .5 <= self.envelope_window_s <= 2 or self.max_rate_ppm > 2000
                or self.rate_slew_ppm_s > 100 or self.small_future_ns > 2000
                or self.max_small_future_drops > 3 or self.max_uncertainty_s > .02
                or self.max_age_s != .5 or self.clock_jump_s > .1
                or self.max_scan_duration_s > .2 or self.margin_s != .02):
            raise ValueError('outside bounded diagnostic contract')


class BoundedTimeAlignment(SensorTimeAlignment):
    """Latched DEGRADED/FAULT require restart; no automatic mission rearm.

    Single small-future samples are dropped, never published/clamped. 2 us is
    an explicit quantization hypothesis covering 847 ns, not clock-error grace.
    It is verified/recorded separately from the unchanged 500 ms stale limit.
    """
    def __init__(self, config=BoundedConfig()):
        super().__init__(config)
        self._segments = deque(maxlen=config.segment_capacity)  # (source, mapped, ppb)
        self._envelopes = deque(maxlen=config.envelope_history)
        self._window_start = None
        self._window_min = None
        self._window_counts = dict(cloud=0, imu=0)
        self._window_history = deque(maxlen=config.envelope_history)
        self._high_source = 0
        self._last_mapped = {}
        self._last_arrival = {}
        self._last_source_receive = {}
        self._small_future_run = dict(cloud=0, imu=0)
        self.rate_ppb = 0
        self.uncertainty_ns = 0
        self.model_updates = 0
        self.reject_reasons = Counter()
        self.failure_reason = ''
        self.last_end_residual_ns = dict(cloud=None, imu=None)
        self._ever_locked = False
        self.skipped_envelope_windows = 0
        self.consecutive_skipped_windows = 0
        self._last_complete_envelope_mono = None
        self.consecutive_out_of_bound_rate_windows = 0
        self.last_rate_estimate_ppb = None
        self.last_bounded_fit_error_ns = None

    def _drop(self, stream, reason, fault=False):
        self.reject_reasons[stream + ':' + reason] += 1
        if fault and not self.failure_reason:
            self.failure_reason = reason
        return super()._drop(stream, reason, fault)

    def _degrade(self, reason):
        if self.state != 'FAULT' and self.state != 'DEGRADED':
            self.state = 'DEGRADED'
            self.reason = reason
            self.failure_reason = reason

    def check_health(self, now_mono_ns):
        if self._ever_locked and self.state == 'LOCKED':
            if any(now_mono_ns - self._last_arrival.get(s, -NS) > round(self.config.max_age_s * NS)
                   for s in ('imu', 'cloud')):
                self._degrade('sensor_dropout_restart_required')
        return self.state == 'LOCKED'

    def map_time(self, source_ns):
        """Evaluate frozen common map, including old segments; never mutate it."""
        if not self._segments or source_ns < self._segments[0][0]:
            raise ValueError('outside retained mapping history')
        starts = [s[0] for s in self._segments]
        anchor, mapped, ppb = self._segments[bisect_right(starts, source_ns) - 1]
        delta = source_ns - anchor
        value = mapped + delta + div_round(delta * ppb, NS)
        if not 0 < value <= I64_MAX:
            raise ValueError('mapped timestamp out of int64 range')
        return value

    def map_relative_times(self, source_ns, times_s):
        """Numpy is used only for cloud points, never epoch * float.

        Preserve field dtype/layout in caller. Source-relative times are rounded
        to integer ns; each point receives F(source+t)-F(source), so hinge
        crossings and header/IMU use exactly one model. No in-place input writes.
        """
        import numpy as np
        times = np.asarray(times_s)
        if not np.isfinite(times).all() or (times < 0).any() or (times > self.config.max_scan_duration_s).any():
            raise ValueError('invalid point relative time')
        offsets = np.rint(times.astype(np.float64) * NS).astype(np.int64)
        mapped = np.empty(offsets.shape, dtype=np.int64)
        base = self.map_time(source_ns)
        segments = list(self._segments)
        if source_ns < segments[0][0]:
            raise ValueError('cloud older than mapping history')
        starts = [s[0] for s in segments]
        first = bisect_right(starts, source_ns) - 1
        last = bisect_right(starts, source_ns + int(offsets.max())) if offsets.size else first
        for i in range(first, last):
            anchor, value, ppb = segments[i]
            lo = max(0, anchor - source_ns)
            hi = segments[i + 1][0] - source_ns if i + 1 < len(segments) else I64_MAX
            mask = (offsets >= lo) & (offsets < hi)
            # delta bounded by retained history / scan duration, avoiding overflow.
            delta = offsets[mask] + (source_ns - anchor)
            # Split before multiply to keep int64 intermediates bounded.
            n = (delta % NS) * ppb
            corr = (delta // NS) * ppb + np.where(n >= 0, (n + NS // 2) // NS, -((-n + NS // 2) // NS))
            mapped[mask] = value - base + delta + corr
        return mapped.astype(np.float64) / NS

    def _observe_envelope(self, stream, source_end, receive_ros, receive_mono):
        if self._window_start is None:
            self._window_start = receive_mono
        if receive_mono - self._window_start >= round(self.config.envelope_window_s * NS):
            required = dict(cloud=self.config.envelope_min_cloud_samples,
                            imu=self.config.envelope_min_imu_samples)
            sufficient = self._window_min is not None and all(self._window_counts[s] >= n for s, n in required.items())
            self._window_history.append(dict(start_mono_ns=self._window_start,
                                             end_mono_ns=receive_mono,
                                             counts=dict(self._window_counts),
                                             required_counts=required,
                                             sufficient=sufficient,
                                             closing_stream=stream))
            if sufficient:
                self.consecutive_skipped_windows = 0
                self._last_complete_envelope_mono = receive_mono
                self._envelopes.append(self._window_min)
                if len(self._envelopes) >= 4:
                    self._update_model()
            else:
                self.skipped_envelope_windows += 1
                self.consecutive_skipped_windows += 1
                # Keep the last continuous mapping for one incomplete window;
                # do not fit a clock update from an undersampled envelope.
                # Every packet still passes dropout/age/future/jump checks.
                if self.consecutive_skipped_windows > self.config.max_skipped_envelope_windows:
                    self._degrade('insufficient_common_envelope_samples')
            self._window_start = receive_mono
            self._window_min = None
            self._window_counts = dict(cloud=0, imu=0)
        # Half-open receive-time windows: the boundary sample belongs to the
        # new window, together with its envelope minimum. Counting it before
        # closure credits the wrong stream to the old window and then loses it.
        self._window_counts[stream] += 1
        item = (source_end, receive_ros - source_end)
        if self._window_min is None or item[1] < self._window_min[1]:
            self._window_min = item
        return item

    def _update_model(self):
        points = list(self._envelopes)
        slopes = [div_round((b[1] - a[1]) * NS, b[0] - a[0])
                  for i, a in enumerate(points) for b in points[i + 1:]
                  if b[0] - a[0] >= NS // 2]
        if not slopes:
            self._degrade('envelope_not_observable')
            return
        estimate = round(median(slopes))
        self.last_rate_estimate_ppb = estimate
        bound = round(self.config.max_rate_ppm * 1000)
        if abs(estimate) > bound:
            # Envelope slopes include variable one-way delivery time. One
            # implausible slope is not proof that the sensor clock changed.
            # Keep the mapped rate bounded, but permit only one such window,
            # and only if every retained envelope fits within the existing
            # uncertainty limit under a bounded-rate model.
            self.consecutive_out_of_bound_rate_windows += 1
            bounded = max(-bound, min(bound, estimate))
            ref_source, ref_q = points[-1]
            self.last_bounded_fit_error_ns = max(
                abs((q - ref_q) - div_round((x - ref_source) * bounded, NS))
                for x, q in points)
            if (self.consecutive_out_of_bound_rate_windows > self.config.max_out_of_bound_rate_windows
                    or self.last_bounded_fit_error_ns > round(self.config.max_uncertainty_s * NS)):
                self._degrade('estimated_rate_outside_bound')
                return
            estimate = bounded
        else:
            self.consecutive_out_of_bound_rate_windows = 0
            self.last_bounded_fit_error_ns = None
        ref_source, ref_q = points[-1]
        errors = sorted(abs((q - ref_q) - div_round((x - ref_source) * estimate, NS)) for x, q in points)
        self.uncertainty_ns = max(errors) if self.consecutive_out_of_bound_rate_windows else errors[(len(errors) - 1) * 3 // 4]
        if self.uncertainty_ns > round(self.config.max_uncertainty_s * NS):
            self._degrade('envelope_uncertainty_exceeded')
            return
        anchor = self._high_source + 1  # beyond all previous scan ends and IMUs
        mapped_anchor = self.map_time(anchor)
        target = ref_q + div_round((anchor - ref_source) * estimate, NS) - round(self.config.margin_s * NS)
        current = mapped_anchor - anchor
        # Bounded phase servo (4 s time constant), not per-packet receipt clamp.
        phase_ppb = div_round((target - current) * NS, 4 * NS)
        desired = max(-bound, min(bound, estimate + phase_ppb))
        max_change = round(self.config.rate_slew_ppm_s * self.config.envelope_window_s * 1000)
        rate = max(self.rate_ppb - max_change, min(self.rate_ppb + max_change, desired))
        if rate != self.rate_ppb:
            self._segments.append((anchor, mapped_anchor, rate))
            self.rate_ppb = rate
            self.model_updates += 1

    def map_stamp(self, stream, source_ns, receive_ros_ns, receive_mono_ns, span_ns=0):
        if stream not in self.seen:
            raise ValueError('unknown stream')
        if self.state in ('FAULT', 'DEGRADED'):
            return self._drop(stream, self.failure_reason or self.reason)
        if (any(type(v) is not int for v in (source_ns, receive_ros_ns, receive_mono_ns, span_ns))
                or not 0 < source_ns <= I64_MAX - max(0, span_ns)
                or not 0 < receive_ros_ns <= I64_MAX or receive_mono_ns < 0
                or span_ns < 0 or span_ns > round(self.config.max_scan_duration_s * NS)
                or (stream == 'imu' and span_ns)):
            return self._drop(stream, 'invalid_measurement_time')
        if self._last_receive is not None:
            ros, mono = self._last_receive
            if receive_mono_ns < mono or abs((receive_ros_ns - ros) - (receive_mono_ns - mono)) > round(self.config.clock_jump_s * NS):
                return self._drop(stream, 'host_clock_jump', fault=True)
        self._last_receive = receive_ros_ns, receive_mono_ns
        previous = self._last_source.get(stream)
        if previous is not None and source_ns <= previous:
            return self._drop(stream, 'sensor_clock_reset' if previous - source_ns > round(self.config.max_age_s * NS)
                              else 'duplicate_or_out_of_order', fault=previous - source_ns > round(self.config.max_age_s * NS))
        # A cloud is delivered after its final point. Compare that endpoint
        # against arrivals; varying scan duration must not look like a clock jump.
        # IMU span is validated as zero above, preserving its original check.
        source_end_ns = source_ns + span_ns
        pair = self._last_source_receive.get(stream)
        if pair is not None and source_end_ns - pair[0] - (receive_mono_ns - pair[1]) > round(self.config.clock_jump_s * NS):
            return self._drop(stream, 'sensor_clock_forward_jump', fault=True)
        if self._ever_locked:
            self.check_health(receive_mono_ns)
            if self.state != 'LOCKED':
                return self._drop(stream, self.failure_reason)
        self._last_source[stream] = source_ns
        self._last_source_receive[stream] = source_end_ns, receive_mono_ns
        self._last_arrival[stream] = receive_mono_ns
        self.seen[stream] += 1
        if self.state == 'WARMUP':
            if self._first_mono is None:
                self._first_mono = receive_mono_ns
            q = receive_ros_ns - source_ns - span_ns
            self._minimum_offset = q if self._minimum_offset is None else min(q, self._minimum_offset)
            if (receive_mono_ns - self._first_mono < round(self.config.warmup_s * NS)
                    or self.seen['cloud'] < self.config.min_cloud_samples or self.seen['imu'] < self.config.min_imu_samples):
                return self._drop(stream, 'collecting_clock_samples')
            self.offset_ns = self._minimum_offset - round(self.config.margin_s * NS)
            self._segments.append((source_ns - NS, source_ns - NS + self.offset_ns, 0))
            self.state = 'LOCKED'
            self._ever_locked = True
            self._last_complete_envelope_mono = receive_mono_ns
        if self.consecutive_skipped_windows:
            # Bound extrapolation while holding the last model. Source and
            # held-map rates may differ by twice the permitted rate bound.
            held_s=(receive_mono_ns-self._last_complete_envelope_mono)/NS
            uncertainty=self.uncertainty_ns/NS+held_s*2*self.config.max_rate_ppm/1e6
            if uncertainty>self.config.max_uncertainty_s:
                self._degrade('held_clock_uncertainty_exceeded')
                return self._drop(stream,self.failure_reason)
        try:
            mapped = self.map_time(source_ns)
            end = self.map_time(source_ns + span_ns)
        except ValueError:
            return self._drop(stream, 'outside_mapping_history', fault=True)
        age = receive_ros_ns - mapped
        residual = receive_ros_ns - end
        self.last_age_ns[stream] = age
        self.last_end_residual_ns[stream] = residual
        reference_age = receive_ros_ns - (source_ns + self.offset_ns)
        # A gradually increasing transport delay is indistinguishable from a
        # slower source clock. Never use adaptation to bypass the original
        # positive-age 500 ms guard, even when the moving model looks fresh.
        if max(age, reference_age) > round(self.config.max_age_s * NS):
            self._degrade('measurement_stale_restart_required')
            return self._drop(stream, self.failure_reason)
        if residual < 0:
            self._small_future_run[stream] += 1
            if residual < -self.config.small_future_ns:
                self._degrade('future_residual_exceeded')
            elif self._small_future_run[stream] >= self.config.max_small_future_drops:
                self._degrade('persistent_small_future')
            return self._drop(stream, self.failure_reason or 'small_future_drop')
        self._small_future_run[stream] = 0
        if mapped <= self._last_mapped.get(stream, 0):
            return self._drop(stream, 'mapped_order_violation', fault=True)
        self._high_source = max(self._high_source, source_ns + span_ns)
        self._observe_envelope(stream, source_ns + span_ns, receive_ros_ns, receive_mono_ns)
        if self.state != 'LOCKED':
            return self._drop(stream, self.failure_reason)
        self._last_mapped[stream] = mapped
        self.accepted[stream] += 1
        self.reason = ''
        return mapped

    def status(self):
        result = super().status()
        result.update(variant='bounded_common_v1', rate_ppb=self.rate_ppb,
                      uncertainty_ns=self.uncertainty_ns, model_updates=self.model_updates,
                      segment_count=len(self._segments), envelope_count=len(self._envelopes),
                      reject_reasons=dict(self.reject_reasons), failure_reason=self.failure_reason,
                      last_end_residual_ns=dict(self.last_end_residual_ns),
                      envelope_window_counts=dict(self._window_counts),
                      envelope_window_history=list(self._window_history),
                      skipped_envelope_windows=self.skipped_envelope_windows,
                      consecutive_skipped_windows=self.consecutive_skipped_windows,
                      consecutive_out_of_bound_rate_windows=self.consecutive_out_of_bound_rate_windows,
                      last_rate_estimate_ppb=self.last_rate_estimate_ppb,
                      last_bounded_fit_error_ns=self.last_bounded_fit_error_ns,
                      envelope_counting='half_open_receive_time_v2',
                      mapping_valid=self.state == 'LOCKED', restart_required=self.state in ('DEGRADED', 'FAULT'),
                      motion_authorized=False, clock_rate_identified=False)
        return result
