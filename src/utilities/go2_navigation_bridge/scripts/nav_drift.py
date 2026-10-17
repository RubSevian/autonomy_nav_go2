#!/usr/bin/env python3
"""Subscriber-only trajectory capture with explicit operator phase marks."""
import argparse,csv,json,math,select,signal,sys,time,shutil,subprocess
from datetime import datetime
from pathlib import Path
from nav_capture_common import atomic,diagnostic_statuses,default_recording_profile,recording_config

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path);p.add_argument('--output',type=Path);p.add_argument('--seconds',type=int);a,ros_args=p.parse_known_args()
 if ros_args and ros_args[0]!='--ros-args':p.error('Unknown recorder arguments: '+str(ros_args))
 a.config=a.config or default_recording_profile();config=recording_config(a.config)
 a.seconds=config['duration_s'] if a.seconds is None else a.seconds
 if not 10<=a.seconds<=config['max_duration_s']:p.error('Duration outside YAML bounds')
 if a.output is None:a.output=Path(config['output_root'])/(config['output_prefix']+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 config=dict(config,duration_s=a.seconds);shutil.copyfile(a.config,out/'navigation_profile.yaml')
 import yaml
 (out/'recording_config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
 import rclpy
 from rclpy.node import Node
 from rclpy.parameter import Parameter
 from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
 from rclpy.signals import SignalHandlerOptions
 from nav_msgs.msg import Odometry,Path as NavPath
 from geometry_msgs.msg import TwistStamped
 from diagnostic_msgs.msg import DiagnosticArray
 from sensor_msgs.msg import PointCloud2,Imu
 from std_msgs.msg import String
 rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
 n=Node('manual_drift_recorder',namespace='/diagnostics',enable_rosout=False,start_parameter_services=False,parameter_overrides=[Parameter('start_type_description_service',value=False)])
 n.destroy_publisher(n._parameter_event_publisher);n._parameter_event_publisher=None
 rows=[];events=[];streams={};phase='unmarked';stop=False;stop_reason='duration_elapsed';capture_error=None;statusfile=(out/'status.jsonl').open('w')
 def interrupted(sig,frame):
  nonlocal stop,stop_reason
  stop=True;stop_reason=signal.Signals(sig).name
 signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
 def count(topic,m,info=None):
  now=time.monotonic_ns();d=streams.setdefault(topic,{'count':0,'first_ns':now,'last_ns':now,'max_gap_ns':0})
  d['count']+=1;d['max_gap_ns']=max(d['max_gap_ns'],now-d['last_ns']);d['last_ns']=now
 def odom(m,info):
  nonlocal stop,stop_reason
  now=time.monotonic_ns();count('odom',m);p=m.pose.pose.position;q=m.pose.pose.orientation
  if len(rows)>=config['pose_capacity']:stop=True;stop_reason='pose_capacity_exhausted';events.append({'mono_ns':now,'error':stop_reason});return
  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,yaw]
  rows.append([now,m.header.stamp.sec*10**9+m.header.stamp.nanosec,phase,m.header.frame_id,m.child_frame_id,*values,all(math.isfinite(v) for v in values) and .5<sum(v*v for v in [q.x,q.y,q.z,q.w])<1.5 and bool(m.header.frame_id),info['source_timestamp'],info['received_timestamp']])
 def state(topic,m):statusfile.write(json.dumps({'mono_ns':time.monotonic_ns(),'topic':topic,'data':m.data})+'\n')
 subs=[n.create_subscription(Odometry,'/state_estimation',odom,qos_profile_sensor_data)]
 for typ,topic in [(PointCloud2,'/utlidar/cloud'),(PointCloud2,'/registered_scan'),(Imu,'/utlidar/imu')]:subs.append(n.create_subscription(typ,topic,lambda m,t=topic:count(t,m),qos_profile_sensor_data))
 for topic in ['/go2/locomotion_status','/utlidar/time_alignment','/go2/navigation/status',config['point_lio_timing_topic']]:
  subs.append(n.create_subscription(String,topic,lambda m,t=topic:state(t,m),10))
 def diagnostic(topic,m):
  count(topic,m)
  statusfile.write(json.dumps({'mono_ns':time.monotonic_ns(),'topic':topic,'statuses':diagnostic_statuses(m)})+'\n')
 for topic in ['/far/planner_status','/local_planner/status','/path_follower/status']:
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/far/planner_status' else 10
  subs.append(n.create_subscription(DiagnosticArray,topic,lambda m,t=topic:diagnostic(t,m),qos))
 def path_record(topic,m):
  count(topic,m)
  statusfile.write(json.dumps({'mono_ns':time.monotonic_ns(),'topic':topic,'poses':len(m.poses),'header_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec})+'\n')
 for topic in ['/path','/point_lio/path']:
  subs.append(n.create_subscription(NavPath,topic,lambda m,t=topic:path_record(t,m),qos_profile_sensor_data))
 def candidate(m):
  count('/navigation/cmd_vel',m)
  statusfile.write(json.dumps({'mono_ns':time.monotonic_ns(),'topic':'/navigation/cmd_vel','command':[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]})+'\n')
 subs.append(n.create_subscription(TwistStamped,'/navigation/cmd_vel',candidate,10))
 print(f'Recording: {out}; duration={a.seconds}s; interactive_marks={config["interactive_marks"]}',flush=True)
 if config['interactive_marks']:print('Optional marks: 1=start stationary, 2=moving, 3=return stationary, q=finish.',flush=True)
 begin=time.monotonic()
 try:
  while not stop and time.monotonic()-begin<a.seconds:
   rclpy.spin_once(n,timeout_sec=config['spin_timeout_s'])
   if config['interactive_marks'] and sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
    value=sys.stdin.readline().strip();name={'1':'START_STATIONARY','2':'MOVING','3':'RETURN_STATIONARY'}.get(value)
    if value=='q':stop=True;stop_reason='operator_q'
    if name:phase=name;events.append({'mono_ns':time.monotonic_ns(),'phase':name});print(name,flush=True)
 except Exception as error:
  capture_error=type(error).__name__+': '+str(error);stop_reason='callback_or_recorder_error'
  events.append({'mono_ns':time.monotonic_ns(),'error':capture_error})
  raise
 finally:
  with (out/'odometry.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['receipt_mono_ns','header_ns','phase','frame','child','x','y','z','qx','qy','qz','qw','yaw','finite','dds_source_ns','dds_received_ns']);w.writerows(rows)
  atomic(out/'marks.json',events);atomic(out/'streams.json',streams)
  atomic(out/'capture.json',{'duration_s':time.monotonic()-begin,'planned_duration_s':a.seconds,'poses':len(rows),'capacity':config['pose_capacity'],'complete':capture_error is None and len(rows)<config['pose_capacity'],'stop_reason':stop_reason,'error':capture_error,'recording_config':config,'config_path':str(a.config.resolve()),'own_publishers':n.get_publisher_names_and_types_by_node(n.get_name(),n.get_namespace()),'own_clients':n.get_client_names_and_types_by_node(n.get_name(),n.get_namespace()),'queue_drops':'NOT_OBSERVABLE','ground_truth':False})
  statusfile.close();n.destroy_node();rclpy.shutdown()
 if config['analyze_on_finish']:
  subprocess.run([sys.executable,str(Path(__file__).with_name('nav_drift_analyze.py')),str(out)],check=True)
 print('Saved recording: '+str(out),flush=True)
 return 0
if __name__=='__main__':raise SystemExit(main())
