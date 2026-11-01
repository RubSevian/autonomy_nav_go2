"""Passive 1 Hz thermal log. Ages describe transport/common frames, not each sensor."""
import csv
import json
import math
import time

FIELDS = ('sample_mono_ns', 'sample_ros_ns', 'mission_state', 'mission_session_id',
          'mission_status_age_s', 'source', 'motor_index', 'motor_id', 'sensor',
          'temperature_c', 'available', 'receipt_age_s', 'common_frame_age_s',
          'source_tick', 'owner_session_id', 'source_session_id', 'error')


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def thermal_rows(now, ros, mission, legs, arm):
    """Missing/invalid readings remain empty; old readings retain their increasing age."""
    state, session, mission_ns = mission
    base = dict(sample_mono_ns=now, sample_ros_ns=ros, mission_state=state,
                mission_session_id=session,
                mission_status_age_s=(now-mission_ns)/1e9 if mission_ns else '')
    for source, sample, count, sensors in (
            ('go2_lowstate', legs, 12, ('motor',)),
            ('rars_owner', arm, 7, ('mos', 'rotor'))):
        sample = sample or {}
        receipt = sample.get('receipt_ns', 0)
        frame = sample.get('frame_ns', 0)
        for index in range(count):
            valid = sample.get('valid', [False]*count)[index]
            for sensor in sensors:
                value = sample.get(sensor, [None]*count)[index]
                available = bool(valid and finite(value))
                yield dict(base, source=source, motor_index=index,
                           motor_id=sample.get('ids', ['']*count)[index], sensor=sensor,
                           temperature_c=value if available else '', available=available,
                           receipt_age_s=(now-receipt)/1e9 if receipt else '',
                           common_frame_age_s=(now-frame)/1e9 if frame else '',
                           source_tick=sample.get('tick', ''),
                           owner_session_id=sample.get('session', ''),
                           source_session_id=sample.get('source_session', ''),
                           error=sample.get('error', '' if sample else 'NO_DATA'))


class TemperatureCapture:
    def __init__(self, node, directory):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.file = (directory/'temperatures.csv').open('w', buffering=1)
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.raw_legs = self.raw_arm = None
        self.legs = self.arm = None
        self.mission = ('UNKNOWN', '', 0)
        self.next_sample = 0
        self.samples = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Existing native observer downsamples measured LowState to 1 Hz.
        self.subscriptions = [
            node.create_subscription(String, '/go2/diagnostics/temperatures', self.receive_legs, qos),
            node.create_subscription(String, '/rars01/diagnostics/temperatures', self.receive_arm, qos)]

    def receive_legs(self, message):
        self.raw_legs = (message.data, time.monotonic_ns())

    def receive_arm(self, message):
        self.raw_arm = (message.data, time.monotonic_ns())

    def sample(self, now, ros):
        if now < self.next_sample:
            return
        self.next_sample = now + 1_000_000_000
        if self.raw_legs:
            raw, receipt = self.raw_legs
            self.raw_legs = None
            try:
                value = json.loads(raw)
                values = value['temperature_c']
                frame = value['frame_received_ns']
                stamp = value['observed_ns']
                tick = value['robot_tick']
                session = value['source_session_id']
                if (not isinstance(values, list) or len(values) != 12
                        or not all(finite(v) for v in values)
                        or type(frame) is not int or not 0 <= frame <= stamp
                        or type(stamp) is not int or not 0 < stamp <= receipt <= now
                        or type(tick) is not int or not 0 <= tick <= 0xffffffff
                        or not isinstance(session, str) or not session
                        or type(value['valid']) is not bool):
                    raise ValueError('Invalid native thermal snapshot')
                self.legs = dict(receipt_ns=receipt, frame_ns=frame, tick=tick,
                                 motor=values, valid=[value['valid'] and frame > 0]*12,
                                 ids=list(range(12)), source_session=session)
            except Exception as error:
                self.legs = {'error': type(error).__name__}
        if self.raw_arm:
            import yaml
            raw, receipt = self.raw_arm
            self.raw_arm = None
            try:
                value = yaml.safe_load(raw)
                for key in ('valid', 'motor_id', 'mos_c', 'rotor_c'):
                    if not isinstance(value[key], list) or len(value[key]) != 7:
                        raise ValueError('Expected seven motor values: '+key)
                frame = int(value['frame_received_ns'])
                if frame < 0 or frame > now:
                    raise ValueError('Invalid common frame clock')
                self.arm = dict(receipt_ns=receipt, frame_ns=frame,
                                valid=[v is True and value['connected'] is True and frame > 0
                                       for v in value['valid']], ids=value['motor_id'],
                                mos=value['mos_c'], rotor=value['rotor_c'],
                                session=value['owner_session_id'])
            except Exception as error:
                self.arm = {'error': type(error).__name__}
        self.writer.writerows(thermal_rows(now, ros, self.mission, self.legs, self.arm))
        self.samples += 1

    def close(self):
        self.file.close()
