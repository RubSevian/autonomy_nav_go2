#!/usr/bin/env python3
"""Chronological stationary replay. Reports a candidate, never edits configuration."""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from nav_motion_analyze import rotation
from navigation_motion import MotionEvidence


def long_term_segments(samples):
    """Describe localization over continuous valid intervals, without claiming truth.

    Coordinates are relative to the median of the first second, not successive
    differences. Gaps, stale data and clock reversals must not join two stands.
    """
    segments=[]; current=[]
    for sample in samples:
        x,y,stamp,age=sample
        valid=np.isfinite([x,y,age]).all() and 0 <= age <= .25
        if current and (not valid or not 0 < stamp-current[-1][2] <= 250_000_000):
            segments.append(current); current=[]
        if valid:current.append(sample)
    if current:segments.append(current)
    result=[]
    for segment in segments:
        t=(np.array([s[2] for s in segment],dtype=np.int64)-segment[0][2])/1e9
        if t[-1]<10:continue
        xy=np.array([s[:2] for s in segment])
        origin=np.median(xy[t<=1.],axis=0)
        final=np.median(xy[t>=t[-1]-1.],axis=0)
        radius=np.linalg.norm(xy-origin,axis=1)
        bins=[]
        for start in range(0,int(t[-1]),5):
            chunk=xy[(t>=start)&(t<start+5)]
            if len(chunk):bins.append(dict(time_s=start,xy_from_start_m=(np.median(chunk,axis=0)-origin).tolist()))
        result.append(dict(start_stamp_ns=segment[0][2],duration_s=float(t[-1]),
            samples=len(segment),endpoint_shift_m=float(np.linalg.norm(final-origin)),
            max_distance_from_start_m=float(max(radius)),
            p95_distance_from_start_m=float(np.percentile(radius,95)),
            xy_span_m=np.ptp(xy,axis=0).tolist(),five_second_medians=bins))
    return result


def replay(capture, *, stationary_confirmed=False, require_stable_rl=False):
    if not stationary_confirmed:
        raise ValueError('Physical stationary confirmation is required, not merely zero commands')
    root=Path(capture)
    profile=yaml.safe_load((root/'navigation_profile.yaml').read_text())
    offset=np.array([-profile['robot']['sensor_offset_x_m'],
                     -profile['robot']['sensor_offset_y_m'],0.])
    rl=[]
    status_path=root/'status.jsonl'
    if status_path.exists():
        with status_path.open() as stream:
            for line in stream:
                try:
                    record=json.loads(line)
                    if record.get('topic')!='/go2/locomotion_status':continue
                    data=record['data']
                    if isinstance(data,str):data=yaml.load(data,Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))
                    observed=data.get('observed_ns')
                    if type(observed) is not int or observed<=0:continue
                    rl.append((observed,data))
                except (ValueError,TypeError,KeyError,yaml.YAMLError):continue
    rl.sort(key=lambda r:r[0]);times=[r[0] for r in rl]
    stable_since=[];since=None;previous=None
    for observed,d in rl:
        key=(d.get('state'),d.get('phase'),d.get('output_enabled'))
        zero=d.get('motion_command')==[0,0,0]
        steady=(key in (('STANDBY','DISARMED',False),('ACTIVE','RL_ZERO',True)) and zero)
        if not steady:since=None
        elif previous is None or key!=previous[1] or observed-previous[0]>750_000_000:since=observed
        stable_since.append(since);previous=(observed,key)
    rejected_rl=0
    samples=[];invalid=0;measurement_mono={}
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
                if require_stable_rl:
                    measured=int(row.get('receipt_mono_ns',0))-int(row['receipt_ros_ns'])+stamp
                    index=bisect.bisect_right(times,measured)-1
                    steady=(index>=0 and index+1<len(rl) and
                            times[index+1]-times[index]<=750_000_000 and
                            stable_since[index] is not None and
                            stable_since[index+1]==stable_since[index] and
                            measured-stable_since[index]>=5_000_000_000)
                    # Offline bracketing of stable states; this is not a live
                    # 250-ms freshness assertion or an independent motion sensor.
                    if not steady:rejected_rl+=1;continue
                samples.append((float(p[0]),float(p[1]),stamp,age))
                if row.get('receipt_mono_ns'):
                    measurement_mono[stamp]=int(row['receipt_mono_ns'])-int(row['receipt_ros_ns'])+stamp
            except (ValueError,KeyError,TypeError):invalid+=1
    if len(samples)<100:
        return dict(capture=root.name,deployable=False,stationary_replay_passed=False,
                    error='INSUFFICIENT_STEADY_SAMPLES',usable_samples=len(samples),rl_rejected_rows=rejected_rl)
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
    reasons=Counter();validation_max=0.;confirmed=[]
    for s in holdout:
        result=estimator.observe(*s);reasons[result.reason]+=1
        validation_max=max(validation_max,result.displacement_m)
        if result.confirmed:confirmed.append(s[2])
    correlation=[]
    for stamp in confirmed:
        measured=measurement_mono.get(stamp)
        index=bisect.bisect_right(times,measured)-1 if measured is not None else -1
        item=dict(pose_stamp_ns=stamp,rl_status='UNAVAILABLE_OR_STALE')
        if index>=0 and 0<=measured-times[index]<=250_000_000:
            d=rl[index][1]
            item.update(rl_status='FRESH',phase=d.get('phase'),state=d.get('state'),
                output_enabled=d.get('output_enabled'),motion_command=d.get('motion_command'),
                requested_navigation=d.get('navigation_requested_command'),
                age_s=(measured-times[index])/1e9)
        correlation.append(item)
    return dict(capture=root.name,pose_sha256=hashlib.sha256((root/'odometry.csv').read_bytes()).hexdigest(),
        physical_stationary_source='operator confirmation',ground_truth=False,
        require_stable_rl=require_stable_rl,rl_rejected_rows=rejected_rl,
        rl_selection='offline bracketing of unchanged zero-command steady phase, gap <= 0.75s, settled >= 5s',
        warmup_excluded_s=10.,training_fraction_of_time=.6,invalid_rows=invalid,
        long_term_segments=long_term_segments([s for s in samples if s[2]>=start]),
        window_s=estimator.window_s,endpoint_s=estimator.endpoint_s,
        training_windows=len(distances),training_max_displacement_m=max(distances),
        candidate_bound_m=bound,candidate_rule='1.25 * training maximum; provisional engineering margin',
        holdout_samples=len(holdout),holdout_max_displacement_m=validation_max,
        holdout_reasons=dict(reasons),
        false_motion_rl_correlation=correlation[:256],
        correlation_truncated=len(correlation)>256,
        stationary_replay_passed=(reasons['MEASURED_MOTION']==0 and
                                  reasons['WITHIN_NOISE_ENVELOPE']>0),
        deployable=False,limitation='One recording; overlapping windows are correlated. '
        'No false-alarm probability or sensitivity to real short movements established. '
        'A finite window cannot distinguish arbitrarily slow drift from motion.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--stationary-confirmed',action='store_true')
    parser.add_argument('--require-stable-rl',action='store_true')
    args=parser.parse_args()
    print(json.dumps(replay(args.capture,stationary_confirmed=args.stationary_confirmed,require_stable_rl=args.require_stable_rl),
                     indent=2,ensure_ascii=False))
