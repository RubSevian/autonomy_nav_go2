#!/usr/bin/env python3
"""Offline relative drift analysis. Requires actual operator marks, no GT claims."""
import argparse,csv,json,math,statistics
from pathlib import Path
from nav_capture_common import atomic

def metrics(rows):
 if len(rows)<2:return {'status':'INSUFFICIENT_SAMPLES','samples':len(rows)}
 first=rows[0];last=rows[-1];seconds=(last['t']-first['t'])/1e9
 xyz=lambda r:[r[k] for k in ['x','y','z']]
 yaw=[rows[0]['yaw']]
 for r in rows[1:]:yaw.append(yaw[-1]+math.remainder(r['yaw']-yaw[-1],2*math.pi))
 return {'samples':len(rows),'duration_s':seconds,'displacement_xyz_m':math.dist(xyz(first),xyz(last)),'displacement_xy_m':math.hypot(last['x']-first['x'],last['y']-first['y']),'max_distance_from_first_m':max(math.dist(xyz(first),xyz(r)) for r in rows),'yaw_change_deg':math.degrees(yaw[-1]-yaw[0]),'mean_odom_displacement_rate_m_per_min':math.dist(xyz(first),xyz(last))*60/seconds if seconds>0 else None}

def automatic_bookends(rows,config):
 if len(rows)<2:return {'valid':False,'reason':'insufficient_odometry'}
 span=round(config['stationary_window_s']*1e9)
 first=[r for r in rows if r['t']<=rows[0]['t']+span]
 last=[r for r in rows if r['t']>=rows[-1]['t']-span]
 def stable(window):
  report=metrics(window)
  if len(window)<2:return dict(report,valid=False)
  yaw=[window[0]['yaw']]
  for r in window[1:]:yaw.append(yaw[-1]+math.remainder(r['yaw']-yaw[-1],2*math.pi))
  spread=math.degrees(max(yaw)-min(yaw));report['yaw_range_deg']=spread
  report['valid']=(report['duration_s']>=config['minimum_stationary_s'] and
    report['max_distance_from_first_m']<=config['stationary_max_radius_m'] and spread<=config['stationary_max_yaw_range_deg'])
  return report
 checks={'start':stable(first),'end':stable(last),'method':'first_last_odometry_windows','physical_stops_confirmed':False}
 checks['valid']=checks['start']['valid'] and checks['end']['valid'] and first[-1]['t']<last[0]['t']
 if checks['valid']:
  for r in rows:r['phase']='START_STATIONARY' if r['t']<=first[-1]['t'] else 'RETURN_STATIONARY' if r['t']>=last[0]['t'] else 'MOVING'
 return checks

def main():
 p=argparse.ArgumentParser();p.add_argument('capture',type=Path);a=p.parse_args();rows=[];segments=[]
 with (a.capture/'odometry.csv').open() as f:
  for r in csv.DictReader(f):
   if r['finite']!='True':continue
   rows.append({'t':int(r['receipt_mono_ns']),'header':int(r['header_ns']),'phase':r['phase'],**{k:float(r[k]) for k in ['x','y','z','yaw']}})
 capture=json.loads((a.capture/'capture.json').read_text());config=capture.get('recording_config',{})
 automatic=bool(config.get('auto_bookends',False)) and not any(r['phase'] in ['START_STATIONARY','RETURN_STATIONARY'] for r in rows)
 checks=automatic_bookends(rows,config) if automatic else None
 minimum=config.get('minimum_stationary_s',2.0);average_ns=round(config.get('closure_average_s',2.0)*1e9)
 for r in rows:
  if not segments or segments[-1]['phase']!=r['phase']:segments.append({'phase':r['phase'],'rows':[]})
  segments[-1]['rows'].append(r)
 stationary=[{'phase':s['phase'],**metrics(s['rows'])} for s in segments if s['phase'] in ['START_STATIONARY','RETURN_STATIONARY']]
 starts=[s['rows'] for s in segments if s['phase']=='START_STATIONARY'];ends=[s['rows'] for s in segments if s['phase']=='RETURN_STATIONARY'];ret=None
 if starts and ends and starts[0][-1]['t']-starts[0][0]['t']>=minimum*1e9 and ends[-1][-1]['t']-ends[-1][0]['t']>=minimum*1e9:
  start=starts[0];end=ends[-1]
  first=[r for r in start if r['t']>=start[-1]['t']-average_ns];last=[r for r in end if r['t']>=end[-1]['t']-average_ns]
  av=lambda rs,k:statistics.mean(r[k] for r in rs)
  p0=[av(first,k) for k in ['x','y','z']];p1=[av(last,k) for k in ['x','y','z']]
  yawmean=lambda rs:math.atan2(av([{'s':math.sin(r['yaw'])} for r in rs],'s'),av([{'c':math.cos(r['yaw'])} for r in rs],'c'))
  ret={'initial_mean_xyz':p0,'returned_mean_xyz':p1,'relative_closure_xyz_m':math.dist(p0,p1),'relative_closure_xy_m':math.hypot(p1[0]-p0[0],p1[1]-p0[1]),'relative_closure_yaw_deg':math.degrees(math.remainder(yawmean(last)-yawmean(first),2*math.pi)),'note':'Operator-marked return; actual return-point/orientation error is inseparable from odometry drift without external reference.'}
 if automatic and ret is not None:ret['note']='Approximate first/last stable odometry windows; operator return and physical stops are unconfirmed. No external ground truth.'
 header_backwards=sum(b['header']<a['header'] for a,b in zip(rows,rows[1:]));result={'status':('ESTIMATED_RELATIVE_AUTO_BOOKENDS' if automatic else 'MEASURED_RELATIVE_ONLY') if stationary and ret is not None and capture['complete'] and not header_backwards else 'INCOMPLETE_OR_UNMARKED','phase_source':'automatic_odometry_bookends' if automatic else 'operator_marks','automatic_stationarity_checks':checks,'trajectory':metrics(rows),'stationary_segments':stationary,'return_to_mark':ret,'nonfinite_or_invalid_rows_excluded':capture['poses']-len(rows),'header_backwards':header_backwards,'absolute_localization_accuracy':'NOT_MEASURED_NO_GROUND_TRUTH'}
 atomic(a.capture/'drift_results.json',result)
 note='Начало и конец выбраны автоматически по стабильности odometry; физическая остановка и точность возврата не подтверждены.' if automatic else 'Остановки и возврат отмечает оператор.'
 (a.capture/'DRIFT_REPORT.md').write_text('# Point-LIO: относительная оценка\n\n```json\n'+json.dumps(result,indent=2,ensure_ascii=False)+'\n```\n\n'+note+' Это не внешняя ground truth.\n')
 print(json.dumps(result,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
