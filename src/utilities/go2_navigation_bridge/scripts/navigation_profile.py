"""Validated source profile, independent of ROS and physical robot ownership."""
import math
import copy
from pathlib import Path
import yaml


def load_profile(path):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(data, dict):
        raise ValueError('Navigation profile must be a mapping')
    fields = {
        'calibration': ('rotation_speed_rps',),
        'robot': ('length_m', 'width_m', 'sensor_offset_x_m', 'sensor_offset_y_m'),
        'local': ('search_radius_m', 'path_scale', 'max_speed_mps', 'max_yaw_rate_rps',
                  'max_accel_mps2', 'goal_tolerance_m', 'stop_distance_m', 'slow_distance_m'),
        'bridge': ('vx_limit', 'vy_limit', 'wz_limit', 'command_timeout_s', 'sensor_timeout_s',
                   'goal_dwell_s', 'tick_period_s', 'status_period_s', 'tf_check_period_s',
                   'graph_check_period_s', 'far_status_timeout_s', 'cancel_timeout_s', 'cancel_retry_s',
                   'tf_pair_history_s', 'tf_pair_position_tolerance_m', 'tf_pair_quaternion_tolerance'),
    }
    for section, names in fields.items():
        for name in names:
            value = data[section][name]
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{section}.{name} must be finite')
            if 'offset' not in name and value <= 0:
                raise ValueError(f'{section}.{name} must be positive')
    if data['calibration']['rotation_speed_rps'] > 1.396:
        raise ValueError('Calibration speed exceeds the original 1.396 rad/s scenario')
    local, robot, bridge = data['local'], data['robot'], data['bridge']
    if bridge['tf_pair_history_s'] < bridge['sensor_timeout_s']:
        raise ValueError('TF pair history must cover sensor_timeout_s')
    if type(bridge['tf_pair_max_samples']) is not int or bridge['tf_pair_max_samples'] < 1:
        raise ValueError('TF pair sample cap must be a positive integer')
    if local['follower_motion_model'] not in ('holonomic', 'unicycle'):
        raise ValueError('Unknown follower motion model')
    for name in ('narrow_passage', 'orientation_aware', 'narrow_recovered_selection'):
        if type(local[name]) is not bool:
            raise ValueError(name + ' must be a boolean')
    if local['path_scale'] != .75:
        raise ValueError('This correspondence library profile requires path_scale=0.75')
    if local['search_radius_m'] * local['path_scale'] < math.hypot(robot['length_m'], robot['width_m']) / 2:
        raise ValueError('Collision radius smaller than robot circumscribed radius')
    if not local['stop_distance_m'] < local['goal_tolerance_m'] <= local['slow_distance_m']:
        raise ValueError('Require stop_distance < goal_tolerance <= slow_distance')
    # Existing R3 NAV contract remains the authority; profile must not promise
    # commands the leg controller will silently clamp to different values.
    if any(bridge[key] > limit for key, limit in zip(('vx_limit', 'vy_limit', 'wz_limit'), (.2, .1, .1))):
        raise ValueError('Bridge limits exceed existing R3 NAV contract')
    if bridge['command_timeout_s'] > .25:
        raise ValueError('Command timeout exceeds existing R3 NAV contract (0.25 s)')
    if local['max_speed_mps'] > bridge['vx_limit'] or local['max_yaw_rate_rps'] > bridge['wz_limit']:
        raise ValueError('Follower limits exceed bridge limits')
    sensors, nodes = data['sensors'], data['nodes']
    sensor_topics = ('raw_cloud_topic', 'raw_imu_topic', 'cloud_topic', 'raw_imu_output_topic',
                     'legacy_imu_output_topic', 'time_status_topic')
    for key in sensor_topics:
        topic = sensors[key]
        if not isinstance(topic, str) or not topic.startswith('/') or any(c.isspace() for c in topic):
            raise ValueError('Invalid sensor topic: ' + key)
    if len({sensors[key] for key in sensor_topics}) != len(sensor_topics):
        raise ValueError('Sensor inputs/outputs must be distinct')
    for key in ('time_warmup_s', 'time_margin_s', 'time_max_age_s', 'time_clock_jump_s',
                'time_max_scan_duration_s', 'time_status_period_s'):
        value = sensors[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('Sensor time parameter must be finite and positive: ' + key)
    for key in ('time_min_cloud_samples', 'time_min_imu_samples'):
        if type(sensors[key]) is not int or sensors[key] < 1:
            raise ValueError('Sensor time sample count must be a positive integer: ' + key)
    if sensors['time_margin_s'] + sensors['time_max_scan_duration_s'] >= sensors['time_max_age_s']:
        raise ValueError('Sensor time margin + max scan duration must be smaller than max age')
    if sensors['time_max_age_s'] > bridge['sensor_timeout_s']:
        raise ValueError('Sensor relay max age must not exceed bridge sensor timeout')
    for key in ('cam_offset_m', 'cloud_pitch_rad', 'imu_pitch_rad', 'imu_yaw_rad', 'imu_tilt_deg'):
        if type(sensors[key]) not in (int, float) or not math.isfinite(sensors[key]):
            raise ValueError('Invalid sensor transform: ' + key)
    if sensors['imu_tilt_deg'] != 15.1:
        raise ValueError('IMU tilt must match the existing 15.1 degree calibration routine')
    for key in ('self_filter_min_m', 'self_filter_max_m'):
        v = sensors[key]
        if not isinstance(v, list) or len(v) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in v):
            raise ValueError('Sensor filter bounds must contain three finite numbers')
    if any(a >= b for a,b in zip(sensors['self_filter_min_m'], sensors['self_filter_max_m'])):
        raise ValueError('Sensor filter min must be smaller than max')
    for name in ('localPlanner', 'pathFollower', 'terrainAnalysis', 'terrainAnalysisExt', 'laserMapping', 'far_planner', 'graph_decoder'):
        if not isinstance(nodes.get(name), dict):
            raise ValueError('Missing node parameter mapping: ' + name)
        for key,value in nodes[name].items():
            sequence = value if isinstance(value, list) else [value]
            if any(type(v) not in (int, float, str, bool) or type(v) in (int, float) and not math.isfinite(v) for v in sequence):
                raise ValueError(f'Invalid parameter {name}.{key}')
    follower, point, far = nodes['pathFollower'], nodes['laserMapping'], nodes['far_planner']
    init_samples = point.get('mapping.imu_init_samples')
    if type(init_samples) is not int or init_samples < 1:
        raise ValueError('mapping.imu_init_samples must be a positive integer')
    for key,minimum,maximum in [('queues.lidar_subscription_depth',1,2000),
                                ('queues.imu_subscription_depth',1,4000),
                                ('queues.lidar_buffer_capacity',2,200),
                                ('queues.imu_buffer_capacity',2,4000)]:
        if type(point.get(key)) is not int or not minimum <= point[key] <= maximum:
            raise ValueError('Invalid Point-LIO queue: ' + key)
    for key in ('timing.status_period_s','timing.warn_age_s'):
        if type(point.get(key)) not in (int,float) or not math.isfinite(point[key]) or point[key]<=0:
            raise ValueError('Invalid Point-LIO timing: ' + key)
    topic=point.get('timing.status_topic')
    if not isinstance(topic,str) or not topic.startswith('/') or any(c.isspace() for c in topic):
        raise ValueError('Invalid Point-LIO timing topic')
    if data['recording'].get('point_lio_timing_topic')!=topic:
        raise ValueError('Recorder and Point-LIO timing topics must match')
    if type(far.get('opencv_threads')) is not int or not 1 <= far['opencv_threads'] <= 16:
        raise ValueError('FAR opencv_threads must be an integer in 1..16')
    if follower['sendSportCommand'] is not False or follower['is_real_robot'] is not False or follower['allowStaticPath'] is not False:
        raise ValueError('RL navigation requires Sport output disabled and continuously refreshed paths')
    if point['preprocess.lidar_type'] != 5 or point['mapping.imu_en'] is not True or point['publish.scan_publish_en'] is not True:
        raise ValueError('Real stack requires Unitree handler, IMU and registered scans')
    for key,length in (('mapping.extrinsic_T',3), ('mapping.extrinsic_R',9), ('mapping.gravity',3), ('mapping.gravity_init',3)):
        if not isinstance(point[key],list) or len(point[key]) != length or any(type(v) not in (int,float) for v in point[key]):
            raise ValueError('Invalid Point-LIO vector: ' + key)
    for key in ('main_run_freq', 'vehicle_height', 'robot_dim', 'voxel_dim'):
        if type(far[key]) not in (int,float) or far[key] <= 0:
            raise ValueError('FAR parameter must be positive: ' + key)
    if far['robot_dim']/2 + far['voxel_dim'] < math.hypot(robot['length_m'],robot['width_m'])/2:
        raise ValueError('FAR clearance smaller than robot envelope')
    if any(nodes[name][key] != 'map' for name,key in (('terrainAnalysis','worldFrame'), ('terrainAnalysisExt','worldFrame'), ('far_planner','world_frame'), ('graph_decoder','world_frame'))):
        raise ValueError('Current real stack requires map world frame')
    # Reject duplicates instead of silently ignoring an operator's edit.
    component_parameters(data)
    return data


def _derived_parameters(data):
    robot, local, sensors = data['robot'], data['local'], data['sensors']
    offsets = dict(sensorOffsetX=float(robot['sensor_offset_x_m']), sensorOffsetY=float(robot['sensor_offset_y_m']))
    speed = dict(maxSpeed=float(local['max_speed_mps']), autonomySpeed=float(local['max_speed_mps']))
    return {
        'localPlanner': dict(offsets, **speed, vehicleLength=float(robot['length_m']), vehicleWidth=float(robot['width_m']),
                             narrowFootprintLength=float(robot['length_m']), narrowFootprintWidth=float(robot['width_m']),
                             pathScale=float(local['path_scale']), minPathScale=float(local['path_scale']),
                             searchRadius=float(local['search_radius_m']), goalCloseDis=float(local['goal_tolerance_m']),
                             enableNarrowPassageMode=local['narrow_passage'], enableOrientationAwareCheck=local['orientation_aware'],
                             enableNarrowRecoveredSelection=local['narrow_recovered_selection']),
        'pathFollower': dict(offsets, **speed, maxYawRate=math.degrees(local['max_yaw_rate_rps']),
                             maxAccel=float(local['max_accel_mps2']), stopDisThre=float(local['stop_distance_m']),
                             slowDwnDisThre=float(local['slow_distance_m']), goalCloseDis=float(local['goal_tolerance_m']),
                             followerMotionModel=local['follower_motion_model'], enableNarrowPassageMode=local['narrow_recovered_selection']),
        'far_planner': {'sensor_offset_x':offsets['sensorOffsetX'], 'sensor_offset_y':offsets['sensorOffsetY'],
                        'g_planner/converge_distance':float(local['goal_tolerance_m'])},
        'laserMapping': {'common.lid_topic':sensors['cloud_topic'], 'common.imu_topic':sensors['raw_imu_output_topic']},
    }


def component_parameters(data):
    """Resolve shared settings once; no I/O, ROS or additional config files."""
    result = copy.deepcopy(data['nodes'])
    for name,parameters in _derived_parameters(data).items():
        overlap = parameters.keys() & result[name].keys()
        if overlap:
            raise ValueError(f'{name}: derived parameters must not be duplicated: {sorted(overlap)}')
        result[name].update(parameters)
    for parameters in result.values():
        if parameters.get('use_sim_time', False) is not False:
            raise ValueError('Real navigation requires use_sim_time=false')
        parameters['use_sim_time'] = False
    return result
