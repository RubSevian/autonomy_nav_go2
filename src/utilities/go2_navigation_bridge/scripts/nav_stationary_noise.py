#!/usr/bin/env python3
"""Chronological stationary replay. Reports a candidate, never edits configuration."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from nav_motion_analyze import rotation
from navigation_motion import MotionEvidence


def replay(capture, *, stationary_confirmed=False):
    if not stationary_confirmed:
        raise ValueError('Physical stationary confirmation is required, not merely zero commands')
    root=Path(capture)
    profile=yaml.safe_load((root/'navigation_profile.yaml').read_text())
    offset=np.array([-profile['robot']['sensor_offset_x_m'],
                     -profile['robot']['sensor_offset_y_m'],0.])
    samples=[];invalid=0
    with (root/'odometry.csv').open() as f:
        for row in csv.DictReader(f):
            try:
                if row['finite']!='True' or (row['frame'],row['child'])!=('camera_init','aft_mapped'):
                    raise ValueError('pose contract')
                r=rotation([float(row[k]) for k in ('qx','qy','qz','qw')])
                p=np.array([float(row[k]) for k in ('x','y','z')])+r@offset
                stamp=int(row['header_ns'])
                age=(int(row['receipt_ros_ns'])-stamp)/1e9
                if int(row['clock_pair_span_ns'])>2_000_000:raise ValueError('clock pair')
                if stamp<=0 or not np.isfinite(p).all():raise ValueError('invalid position')
                samples.append((float(p[0]),float(p[1]),stamp,age))
            except (ValueError,KeyError,TypeError):invalid+=1
    if len(samples)<100:raise ValueError('Insufficient pose samples')
    # Fixed warm-up exclusion and chronological split are analysis choices,
    # not proof that initialization has converged or noise is stationary.
    start=samples[0][2]+10_000_000_000
    end=samples[-1][2]
    if end-start<10_000_000_000:raise ValueError('Recording too short after warmup')
    split=start+int(.6*(end-start))
    train=[s for s in samples if start<=s[2]<split]
    holdout=[s for s in samples if s[2]>=split]
    estimator=MotionEvidence(noise_bound_m=1e-12)
    distances=[]
    for s in train:
        result=estimator.observe(*s)
        if result.samples>=6 and result.duration_s>=estimator.window_s+estimator.endpoint_s:
            distances.append(result.displacement_m)
    if not distances or max(distances)<=0:raise ValueError('No usable training displacement')
    bound=1.25*max(distances)
    estimator=MotionEvidence(noise_bound_m=bound)
    reasons=Counter();validation_max=0.
    for s in holdout:
        result=estimator.observe(*s);reasons[result.reason]+=1
        validation_max=max(validation_max,result.displacement_m)
    return dict(capture=root.name,pose_sha256=hashlib.sha256((root/'odometry.csv').read_bytes()).hexdigest(),
        physical_stationary_source='operator confirmation',ground_truth=False,
        warmup_excluded_s=10.,training_fraction_of_time=.6,invalid_rows=invalid,
        window_s=estimator.window_s,endpoint_s=estimator.endpoint_s,
        training_windows=len(distances),training_max_displacement_m=max(distances),
        candidate_bound_m=bound,candidate_rule='1.25 * training maximum; provisional engineering margin',
        holdout_samples=len(holdout),holdout_max_displacement_m=validation_max,
        holdout_reasons=dict(reasons),
        stationary_replay_passed=(reasons['MEASURED_MOTION']==0 and
                                  reasons['WITHIN_NOISE_ENVELOPE']>0),
        deployable=False,limitation='One recording; overlapping windows are correlated. '
        'No false-alarm probability or sensitivity to real short movements established. '
        'A finite window cannot distinguish arbitrarily slow drift from motion.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--stationary-confirmed',action='store_true')
    args=parser.parse_args()
    print(json.dumps(replay(args.capture,stationary_confirmed=args.stationary_confirmed),
                     indent=2,ensure_ascii=False))
