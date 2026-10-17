#!/usr/bin/env python3
"""Start installed FAR launch in an isolated domain, with NO physical sensor input."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import sys
import math
if os.environ.get('ROS_DOMAIN_ID') != '228' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Synthetic test requires localhost ROS_DOMAIN_ID=228')
import rclpy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters

root=Path(__file__).resolve().parents[6]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_profile import load_profile, component_parameters
profile_path=Path(__file__).resolve().parents[1]/'config/real_navigation.yaml'
profile=load_profile(profile_path)
rclpy.init();node=rclpy.create_node('full_launch_smoke')
statuses=[]
node.create_subscription(String,'/go2/navigation/status',lambda m:statuses.append(json.loads(m.data)),1)
process=None
with tempfile.TemporaryDirectory(prefix='nav-calibration-fixture-') as directory:
    calibration=Path(directory)/'calibration.yaml'
    calibration.write_text(''.join(name+': 0.0\n' for name in ('acc_bias_x','acc_bias_y','acc_bias_z','ang_bias_x','ang_bias_y','ang_bias_z','ang_z2x_proj','ang_z2y_proj')))
    log=(root/'runtime/nav_full_launch_smoke.log').open('w')
    try:
        process=subprocess.Popen(['ros2','launch','go2_navigation_bridge','real_navigation.launch.py','stage:=far','rviz:=false',
                                  'calibration_path:='+str(calibration),'profile_path:='+str(profile_path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        expected={'go2_navigation_bridge','localPlanner','pathFollower','far_planner','terrainAnalysis','terrainAnalysisExt','laserMapping','transform_everything','graph_decoder'}
        deadline=time.monotonic()+25.
        while time.monotonic()<deadline:
            assert process.poll() is None,'launch exited'
            rclpy.spin_once(node,timeout_sec=.1)
            names={name for name,namespace in node.get_node_names_and_namespaces()}
            if expected<=names and statuses:break
        assert expected<=names,(expected-names)
        assert statuses and not statuses[-1]['navigation_ready'] and not statuses[-1]['command_enabled']
        assert not node.get_publishers_info_by_topic('/lowcmd')
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        def parameters(name,keys):
            client=node.create_client(GetParameters,'/'+name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=5.)
            req=GetParameters.Request();req.names=keys;future=client.call_async(req)
            rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
            assert future.done() and future.result() is not None
            values=future.result().values;node.destroy_client(client);return values
        follower=parameters('pathFollower',['sendSportCommand','maxYawRate','followerMotionModel','stopDisThre'])
        assert not follower[0].bool_value and abs(follower[1].double_value-5.729577951)<1e-6
        assert follower[2].string_value=='holonomic' and abs(follower[3].double_value-.225)<1e-6
        geometry=parameters('localPlanner',['vehicleLength','vehicleWidth','minPathScale'])
        assert [v.double_value for v in geometry]==[.62,.4,.75]
        far=parameters('far_planner',['sensor_offset_x','vehicle_height'])
        assert abs(far[0].double_value-.3)<1e-6 and abs(far[1].double_value-.34)<1e-6
        # Every central value is visible in the REAL running node, not just launch text.
        expected_parameters=component_parameters(profile)
        expected_parameters['localPlanner'].pop('pathFolder',None)
        expected_parameters['transform_everything']=profile['sensors']
        def value_of(v):
            return {1:lambda:v.bool_value,2:lambda:v.integer_value,3:lambda:v.double_value,
                    4:lambda:v.string_value,7:lambda:list(v.integer_array_value),
                    8:lambda:list(v.double_array_value)}.get(v.type,lambda:None)()
        for name,settings in expected_parameters.items():
            observed=parameters(name,list(settings))
            for key,wanted,actual in zip(settings,settings.values(),observed):
                actual=value_of(actual)
                assert actual==wanted or type(actual) in (int,float) and type(wanted) in (int,float) and math.isclose(actual,wanted,rel_tol=1e-7,abs_tol=1e-9),(name,key,wanted,actual)
        for topic,depth in [('/utlidar/transformed_cloud',2),('/utlidar/transformed_raw_imu',512)]:
            info=[entry for entry in node.get_subscriptions_info_by_topic(topic) if entry.node_name=='laserMapping']
            assert len(info)==1 and info[0].qos_profile.depth==depth,(topic,info)
        timing=[]
        node.create_subscription(String,'/point_lio/timing',lambda m:timing.append(json.loads(m.data)),1)
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and not timing:rclpy.spin_once(node,timeout_sec=.05)
        assert timing and timing[-1]['odom_published']==0 and timing[-1]['last_odom_age_s']==-1
        for topic,consumer in [('/utlidar/cloud','transform_everything'),('/utlidar/imu','transform_everything'),
                               ('/utlidar/transformed_cloud','laserMapping'),('/utlidar/transformed_raw_imu','laserMapping'),
                               ('/registered_scan','terrainAnalysis'),('/terrain_map_ext','far_planner'),
                               ('/terrain_map','localPlanner'),('/navigation/cmd_vel','go2_navigation_bridge')]:
            assert consumer in {info.node_name for info in node.get_subscriptions_info_by_topic(topic)},(topic,consumer)
        print('PASS installed full FAR launch: real profile wiring, no sensors=>no velocity, bounded Point-LIO QoS/runtime timing, no LowCmd/Sport publisher')
        print('PASS all central component parameters observed in native nodes; LiDAR/IMU/terrain/planner subscriptions match')
    finally:
        if process and process.poll() is None:process.send_signal(signal.SIGINT)
        if process:
            try:process.wait(timeout=10.)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()
        text=(root/'runtime/nav_full_launch_smoke.log').read_text()
        assert 'Traceback (most recent call last)' not in text and 'terminate called' not in text and 'process has died' not in text,text[-4000:]
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
