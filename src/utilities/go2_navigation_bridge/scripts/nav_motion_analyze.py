#!/usr/bin/env python3
"""Offline velocity/mission-base-motion analysis. Never starts ROS or sends goals."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml
from nav_capture_common import atomic

HOME_RESULTS = {'GRASP_PRESUMED_SUCCESS', 'TARGET_NOT_GRASPED', 'OBJECT_NOT_FOUND',
                'GRASP_UNREACHABLE', 'MISSION_COMPLETED_UNVERIFIED'}


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = float(q @ q)
    if not np.isfinite(q).all() or not .5 < norm < 1.5:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / math.sqrt(norm)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def vehicle_velocity(value, covariance, sensor_to_vehicle):
    """Fixed reference-point change, with angular/linear cross covariance."""
    transform = np.eye(6)
    transform[:3, 3:] = -skew(sensor_to_vehicle)
    value, covariance = np.asarray(value), np.asarray(covariance).reshape(6, 6)
    if not np.isfinite(value).all() or not np.isfinite(covariance).all():
        raise ValueError('nonfinite velocity')
    if not np.allclose(covariance, covariance.T, atol=1e-9, rtol=1e-6):
        raise ValueError('asymmetric covariance')
    covariance = (covariance + covariance.T)/2
    if np.linalg.eigvalsh(covariance).min() < -1e-9:
        raise ValueError('indefinite covariance')
    return transform @ value, transform @ covariance @ transform.T


def json_lines(path):
    if not path.exists():
        return
    with path.open() as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue  # A crash may leave a partial final record.


def mission_records(status, journal=None):
    result = []
    for row in status:
        if row.get('topic') == '/go2/mission/status':
            data = row.get('data', {})
            try:
                result.append(json.loads(data) if isinstance(data, str) else data)
            except ValueError:
                continue
    if journal:
        for row in json_lines(journal):
            if row.get('event') == 'mission_state':
                result.append(dict(row, observed_ns=row['monotonic_ns']))
    return sorted(result, key=lambda d: d.get('observed_ns', 0))


def manipulation_windows(records):
    windows = {}
    for d in records:
        key = (d.get('session_id'), d.get('goal_revision'))
        window = d.get('base_motion_window') or {}
        start = window.get('scan_started_ns')
        if start:
            key = (d.get('session_id'), start)
            value = windows.setdefault(key, dict(session_id=key[0], start_ns=start, end_ns=None,
                                                 boundary_source='mission_FSM', outcome=None))
            value['end_ns'] = window.get('home_completed_ns') or value['end_ns']
            if d.get('state') in HOME_RESULTS:
                value['outcome'] = d['state']
        elif d.get('state') == 'ARM_SCAN_POSE':
            windows.setdefault(key, dict(session_id=key[0], start_ns=d['observed_ns'], end_ns=None,
                                          boundary_source='legacy_journal_event', outcome=None))
        elif key in windows and d.get('state') in HOME_RESULTS:
            windows[key].update(end_ns=d['observed_ns'], outcome=d['state'])
    return list(windows.values())


def summary(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return dict(samples=0, mean=None, median=None, p95=None, max=None)
    return dict(samples=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def interval_metrics(samples, start_ns, end_ns, max_gap_s=.5):
    selected = [s for s in samples if start_ns <= s['t'] <= end_ns]
    if len(selected) < 2:
        return dict(status='INSUFFICIENT_SAMPLES', samples=len(selected))
    p = np.array([s['p'] for s in selected]); yaw = np.unwrap([s['yaw'] for s in selected])
    steps = np.diff([s['header'] for s in selected])/1e9
    errors = []
    if np.any(steps <= 0): errors.append('NONINCREASING_MEASUREMENT_TIME')
    if np.any(steps > max_gap_s): errors.append('ODOMETRY_GAP')
    if selected[0]['t']-start_ns > max_gap_s*1e9 or end_ns-selected[-1]['t'] > max_gap_s*1e9:
        errors.append('BOUNDARY_NOT_COVERED')
    if len({(s['frame'], s['child']) for s in selected}) != 1: errors.append('FRAME_CHANGED')
    if any(s.get('invalid_time') for s in selected): errors.append('INVALID_CLOCK_PAIR_OR_AGE')
    if np.any(np.linalg.norm(np.diff(p, axis=0), axis=1) > .25): errors.append('POSE_DISCONTINUITY')
    delta = p[-1]-p[0]
    # Means reduce endpoint jitter, but are reported separately from full-window displacement.
    first = p[np.array([s['t']-selected[0]['t'] <= 1e9 for s in selected])].mean(axis=0)
    last = p[np.array([selected[-1]['t']-s['t'] <= 1e9 for s in selected])].mean(axis=0)
    return dict(status='INVALID_INTERVAL' if errors else 'RELATIVE_ODOMETRY_ONLY', issues=errors,
        samples=len(selected), duration_s=(selected[-1]['t']-selected[0]['t'])/1e9,
        max_gap_s=float(steps.max()), delta_xyz_m=delta.tolist(), displacement_xy_m=float(np.linalg.norm(delta[:2])),
        displacement_xyz_m=float(np.linalg.norm(delta)), endpoint_mean_window_s=1.,
        mean_endpoint_delta_xyz_m=(last-first).tolist(),
        max_excursion_xy_m=float(np.linalg.norm(p[:, :2]-p[0, :2], axis=1).max()),
        max_excursion_xyz_m=float(np.linalg.norm(p-p[0], axis=1).max()),
        yaw_change_deg=float(np.degrees(yaw[-1]-yaw[0])),
        yaw_range_deg=float(np.degrees(yaw.max()-yaw.min())))


def analyze(capture, *, journal=None, output=None, plot=False):
    output = output or capture
    output.mkdir(parents=True, exist_ok=True)
    profile = yaml.safe_load((capture/'navigation_profile.yaml').read_text())
    max_age_s = profile.get('bridge', {}).get('sensor_timeout_s', .5)
    offset = np.array([-profile['robot']['sensor_offset_x_m'], -profile['robot']['sensor_offset_y_m'], 0.])
    status = list(json_lines(capture/'status.jsonl'))
    missions = mission_records(status, journal)
    clock_offset = None
    if journal:
        clock_pairs = [d['wall_ns']-d['monotonic_ns'] for d in json_lines(journal)
                       if 'wall_ns' in d and 'monotonic_ns' in d]
        if clock_pairs: clock_offset = int(np.median(clock_pairs))
    samples = []; invalid_rows = 0; time_methods = set()
    with (capture/'odometry.csv').open() as stream:
        for row in csv.DictReader(stream):
            try:
                if row['finite'] != 'True': raise ValueError('invalid pose')
                r = rotation([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
                p = np.array([float(row[k]) for k in ('x', 'y', 'z')]) + r @ offset
                if not np.isfinite(p).all(): raise ValueError('nonfinite pose')
                header = int(row['header_ns']); receipt = int(row['receipt_mono_ns'])
                invalid_time = False
                if row.get('receipt_ros_ns'):
                    age = int(row['receipt_ros_ns'])-header
                    t = receipt-age; method = 'paired_ROS_monotonic'
                    invalid_time = not 0 <= age <= max_age_s*1e9 or int(row.get('clock_pair_span_ns', 0)) > 2_000_000
                elif clock_offset is not None:
                    t = header-clock_offset; method = 'legacy_journal_clock_approximation'
                else:
                    t = receipt; method = 'legacy_receipt_time_approximation'
                if header <= 0: raise ValueError('invalid stamp')
                samples.append(dict(t=t, header=header, p=p, r=r, yaw=math.atan2(r[1,0], r[0,0]),
                                    frame=row.get('frame'), child=row.get('child'), invalid_time=invalid_time))
                time_methods.add(method)
            except (ValueError, KeyError): invalid_rows += 1
    # Do not reorder odometry: doing so would conceal resets and old queued samples.
    velocity = {}; velocity_invalid = 0; velocity_duplicate = 0
    if (capture/'velocity.csv').exists():
        with (capture/'velocity.csv').open() as stream:
            for row in csv.DictReader(stream):
                try:
                    if row['frame'] != 'aft_mapped': raise ValueError('velocity frame')
                    header = int(row['header_ns'])
                    if header in velocity: velocity_duplicate += 1; velocity[header] = None; continue
                    value, cov = vehicle_velocity([float(row[k]) for k in ('vx','vy','vz','wx','wy','wz')],
                        [float(row[f'cov_{i}']) for i in range(36)], offset)
                    velocity[header] = (value, cov)
                except (ValueError, KeyError, np.linalg.LinAlgError): velocity_invalid += 1
    commands = sorted([d for d in status if d.get('topic') == '/cmd_vel' and d.get('frame') == 'vehicle'],
                      key=lambda d: d['mono_ns'])
    command_times = [d['mono_ns'] for d in commands]
    mission_times = [d['observed_ns'] for d in missions]
    rows = []
    for i, s in enumerate(samples):
        j = bisect.bisect_right(mission_times, s['t'])-1
        mission = missions[j] if j >= 0 else {}
        current_phase = mission.get('state', 'UNCLASSIFIED')
        # Status freshness matters for live records; journal states are persistent transitions.
        if not journal and j >= 0 and s['t']-mission_times[j] > 600_000_000:
            current_phase = 'STATUS_STALE'
        row = dict(header_ns=s['header'], measurement_mono_ns=s['t'], phase=current_phase,
                   grasp_assessment=mission.get('grasp_assessment','UNKNOWN'),
                   vehicle_x_m=float(s['p'][0]), vehicle_y_m=float(s['p'][1]), vehicle_z_m=float(s['p'][2]),
                   vx_mps=None, vy_mps=None, vz_mps=None, speed_xy_mps=None,
                   sigma_vx_mps=None, sigma_vy_mps=None, sigma_vz_mps=None,
                   fd_speed_xy_mps=None, command_vx_mps=None, command_vy_mps=None, tracking_error_xy_mps=None)
        observation = velocity.get(s['header'])
        if observation is not None and not s['invalid_time']:
            v, cov = observation
            for axis, name in enumerate(('vx','vy','vz')):
                row[name+'_mps'] = float(v[axis]); row['sigma_'+name+'_mps'] = math.sqrt(max(0., cov[axis,axis]))
            row['speed_xy_mps'] = math.hypot(v[0],v[1])
            c = bisect.bisect_right(command_times, s['t'])-1
            if c >= 0 and s['t']-command_times[c] <= 250_000_000:
                command = commands[c]['command']
                if len(command)==3 and all(math.isfinite(x) for x in command):
                    row['command_vx_mps'],row['command_vy_mps'] = command[:2]
                    row['tracking_error_xy_mps'] = math.hypot(v[0]-command[0],v[1]-command[1])
        # Derivative is a separate windowed diagnostic, never substituted for EKF velocity.
        k = i
        while k > 0 and 0 < s['header']-samples[k-1]['header'] <= 300_000_000:
            if samples[k]['header']-samples[k-1]['header'] <= 0: break
            k -= 1
        dt = (s['header']-samples[k]['header'])/1e9
        if .2 <= dt <= .3 and all(0 < samples[n]['header']-samples[n-1]['header'] <= 150_000_000
                                  for n in range(k+1,i+1)):
            row['fd_speed_xy_mps'] = float(np.linalg.norm((s['p']-samples[k]['p'])[:2])/dt)
        rows.append(row)
    windows = manipulation_windows(missions)
    for window in windows:
        end = window['end_ns']
        window['metrics'] = (interval_metrics(samples,window['start_ns'],end,max_gap_s=max_age_s) if end else
                             dict(status='HOME_NOT_COMPLETED', samples=0))
        selected = [r for r in rows if end and window['start_ns'] <= r['measurement_mono_ns'] <= end]
        window['estimated_speed_xy_mps'] = summary([r['speed_xy_mps'] for r in selected if r['speed_xy_mps'] is not None])
        window['model_sigma_mps'] = {axis:summary([r['sigma_'+axis+'_mps'] for r in selected
                                                  if r['sigma_'+axis+'_mps'] is not None]) for axis in ('vx','vy','vz')}
    navigation = {}
    for name, phases in [('before_grasp', {'NAVIGATING'}), ('return_after_home', {'RETURN_NAVIGATING'})]:
        selected = [r for r in rows if r['phase'] in phases]
        errors = [r['tracking_error_xy_mps'] for r in selected if r['tracking_error_xy_mps'] is not None]
        navigation[name] = dict(estimated_speed_xy_mps=summary([r['speed_xy_mps'] for r in selected if r['speed_xy_mps'] is not None]),
            observed_grasp_assessments=sorted({r['grasp_assessment'] for r in selected}),
            command_tracking_error_xy_mps=summary(errors),
            command_tracking_rmse_xy_mps=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            finite_difference_speed_xy_mps=summary([r['fd_speed_xy_mps'] for r in selected if r['fd_speed_xy_mps'] is not None]))
    owners = [d for d in status if d.get('topic') == 'source_owners']
    identities = {tuple(sorted(d['data'].get('/state_estimation',[]))) for d in owners}
    nav_sessions = set()
    for d in status:
        if d.get('topic') != '/go2/navigation/status': continue
        try:
            data = json.loads(d['data']) if isinstance(d['data'],str) else d['data']
            if data.get('session_id'): nav_sessions.add(data['session_id'])
        except (ValueError,KeyError): pass
    continuity_issues = []
    nonempty = {x for x in identities if x}
    if len(nonempty)>1 or any(len(x)>1 for x in identities): continuity_issues.append('ODOMETRY_OWNER_CHANGED_OR_MULTIPLE')
    velocity_owners = {tuple(sorted(d['data'].get('/point_lio/velocity',[]))) for d in owners}
    if len({x for x in velocity_owners if x})>1 or any(len(x)>1 for x in velocity_owners):
        continuity_issues.append('VELOCITY_OWNER_CHANGED_OR_MULTIPLE')
    if len(nav_sessions)>1: continuity_issues.append('NAVIGATION_SESSION_CHANGED')
    if any(b['header']<a['header'] for a,b in zip(samples,samples[1:])): continuity_issues.append('ODOMETRY_TIME_RESET')
    if invalid_rows: continuity_issues.append('INVALID_POSE_ROWS')
    if continuity_issues:
        for window in windows:
            window['metrics']['status'] = 'INVALID_INTERVAL'
            window['metrics'].setdefault('issues', []).extend(continuity_issues)
    for metrics in navigation.values():
        metrics['validity'] = 'INVALID_CONTINUITY' if continuity_issues else 'NO_EXTERNAL_GROUND_TRUTH'
    capture_info = json.loads((capture/'capture.json').read_text()) if (capture/'capture.json').exists() else {}
    result = dict(schema='base_motion_v1', reference_point='vehicle', sensor_to_vehicle_m=offset.tolist(),
        recording_complete=capture_info.get('complete'), recording_stop_reason=capture_info.get('stop_reason'),
        quality_max_age_and_gap_s=max_age_s, velocity_frame='vehicle', finite_difference_plane='world_XY',
        time_alignment=sorted(time_methods), samples=len(samples), invalid_pose_rows=invalid_rows,
        velocity_invalid_rows=velocity_invalid, velocity_duplicate_stamps=velocity_duplicate,
        velocity_status='EKF_RECORDED' if any(r['speed_xy_mps'] is not None for r in rows) else 'NOT_RECORDED',
        continuity_issues=continuity_issues, source_owner_history_recorded=bool(owners),
        manipulation_windows=windows, navigation_comparison=navigation,
        return_to_start=next((d.get('return_to_start') for d in reversed(missions) if d.get('return_to_start')),None),
        external_ground_truth=False, absolute_velocity_error='NOT_MEASURED',
        interpretation='Position change includes physical base motion and localization drift. EKF covariance is model uncertainty; command tracking error is not sensor error. Fixed extrinsic uncertainty is not included.')
    atomic(output/'base_motion_results.json',result)
    if rows:
        with (output/'base_motion.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    if plot and rows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes=plt.subplots(3,1,figsize=(11,10),layout='constrained')
        t=(np.array([s['t'] for s in samples])-samples[0]['t'])/1e9
        p=np.array([s['p'] for s in samples]);axes[0].plot(p[:,0],p[:,1]);axes[0].set_aspect('equal',adjustable='datalim')
        axes[0].set(xlabel='vehicle X, m',ylabel='vehicle Y, m',title='Point-LIO relative trajectory (no external ground truth)')
        for w in windows:
            if not w['end_ns']:continue
            selected=np.array([w['start_ns']<=s['t']<=w['end_ns'] for s in samples])
            if not selected.any():continue
            displacement=np.linalg.norm(p[selected,:2]-p[selected,:2][0],axis=1)
            axes[1].plot(t[selected],displacement,label='Scan pose to completed HOME')
        axes[1].set(xlabel='Measurement time from recording start, s',ylabel='XY displacement, m');axes[1].legend()
        for key,label in [('speed_xy_mps','EKF velocity'),('fd_speed_xy_mps','Pose difference, 0.2–0.3 s window')]:
            values=[float('nan') if r[key] is None else r[key] for r in rows]
            if any(math.isfinite(v) for v in values):axes[2].plot(t,values,label=label)
        axes[2].set(xlabel='Measurement time from recording start, s',ylabel='XY speed, m/s');axes[2].legend()
        for ax in axes:ax.grid(alpha=.25)
        fig.savefig(output/'base_motion.png',dpi=160);plt.close(fig)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path)
    parser.add_argument('--mission-journal',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--plot',action='store_true');args=parser.parse_args()
    result=analyze(args.capture,journal=args.mission_journal,output=args.output,plot=args.plot)
    print(json.dumps({k:result[k] for k in ('samples','velocity_status','continuity_issues')},ensure_ascii=False))


if __name__=='__main__':main()
