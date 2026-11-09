#!/usr/bin/env python3
"""Offline evidence for progress-detector calibration; never labels a robot stationary.

Window residuals compare the same EKF's pose and velocity, not ground truth.
No covariance-to-displacement conversion assumes independent time samples.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml
from nav_motion_analyze import rotation, vehicle_velocity


def summary(values):
    a = np.asarray(values, dtype=float)
    return dict(samples=len(a), **({} if not len(a) else {
        'median': float(np.median(a)), 'p95': float(np.quantile(a, .95)),
        'p99': float(np.quantile(a, .99)), 'max': float(np.max(a))}))


def analyze(capture, windows=(.25, .5, 1.)):
    capture = Path(capture)
    profile = yaml.safe_load((capture/'navigation_profile.yaml').read_text())
    offset = np.array([-profile['robot']['sensor_offset_x_m'],
                       -profile['robot']['sensor_offset_y_m'], 0.])
    report = dict(capture=capture.name, stationary_confirmed=False,
                  ground_truth=False, suitable_for_threshold_selection=False,
                  missing=[], invalid_velocity_rows=0, duplicate_velocity_stamps=0,
                  invalid_pose_rows=0, rejected_pose_reasons={}, unmatched_pose_rows=0, discontinuities=0)
    for name in ('odometry.csv', 'velocity.csv'):
        if not (capture/name).exists(): report['missing'].append(name)
    if report['missing']: return report
    velocities = {}
    with (capture/'velocity.csv').open() as stream:
        for row in csv.DictReader(stream):
            try:
                stamp = int(row['header_ns'])
                if stamp <= 0 or row['frame'] != 'aft_mapped': raise ValueError()
                v, _ = vehicle_velocity(
                    [float(row[k]) for k in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')],
                    [float(row[f'cov_{i}']) for i in range(36)], offset)
                if stamp in velocities:
                    report['duplicate_velocity_stamps'] += 1
                    velocities[stamp] = None
                else: velocities[stamp] = v[:3]
            except (ValueError, TypeError, KeyError, np.linalg.LinAlgError):
                report['invalid_velocity_rows'] += 1
    segments = [[]]
    previous_stamp = None
    with (capture/'odometry.csv').open() as stream:
        for row in csv.DictReader(stream):
            try:
                stamp = int(row['header_ns'])
                if (stamp <= 0 or row['finite'] != 'True' or row['frame'] != 'camera_init'
                        or row['child'] != 'aft_mapped'): raise ValueError('pose_contract')
                r = rotation([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
                p = np.array([float(row[k]) for k in ('x', 'y', 'z')]) + r @ offset
                age = (int(row['receipt_ros_ns'])-stamp)/1e9
                if not np.isfinite(p).all(): raise ValueError('nonfinite_pose')
                if not 0 <= age <= .25: raise ValueError('pose_age_over_250ms_or_future')
                if int(row['clock_pair_span_ns']) > 2_000_000: raise ValueError('clock_pair_over_2ms')
                if previous_stamp is not None and not 0 < (stamp-previous_stamp)/1e9 <= .25:
                    report['discontinuities'] += 1
                    segments.append([])
                previous_stamp = stamp
                v = velocities.get(stamp)
                if v is None:
                    report['unmatched_pose_rows'] += 1
                    segments.append([])
                    continue
                segments[-1].append((stamp, p[:2], (r @ v)[:2]))
            except (ValueError, TypeError, KeyError) as error:
                report['invalid_pose_rows'] += 1
                reason = str(error) or type(error).__name__
                reasons = report['rejected_pose_reasons']
                reasons[reason] = reasons.get(reason, 0)+1
                segments.append([])
                previous_stamp = None
    report['paired_samples'] = sum(map(len, segments))
    report['windows'] = {}
    for duration in windows:
        displacement, integral, residual = [], [], []
        for segment in segments:
            if len(segment) < 2: continue
            # Relative integer stamps preserve resolution across ROS clock epochs.
            t = np.array([(s[0]-segment[0][0])/1e9 for s in segment])
            p = np.array([s[1] for s in segment]); v = np.array([s[2] for s in segment])
            accumulated = np.vstack([np.zeros(2), np.cumsum(
                (v[1:]+v[:-1])*.5*np.diff(t)[:, None], axis=0)])
            # Nonoverlapping windows avoid pretending thousands of correlated
            # windows are thousands of independent statistical experiments.
            start = 0
            while start < len(t):
                end = int(np.searchsorted(t, t[start]+duration))
                if end >= len(t): break
                if t[end]-t[start] <= duration+.05:
                    dp = p[end]-p[start]; dv = accumulated[end]-accumulated[start]
                    displacement.append(float(np.linalg.norm(dp)))
                    integral.append(float(np.linalg.norm(dv)))
                    residual.append(float(np.linalg.norm(dp-dv)))
                start = end
        report['windows'][str(duration)] = dict(displacement_m=summary(displacement),
            integrated_velocity_m=summary(integral), consistency_residual_m=summary(residual))
    report['interpretation'] = ('Unlabelled motion: displacement is not stationary noise. '
        'Residual is internal EKF consistency, not physical accuracy. '
        'No calibrated movement threshold can be selected from this report alone.')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    args = parser.parse_args()
    print(json.dumps([analyze(p) for p in args.captures], indent=2, ensure_ascii=False))
