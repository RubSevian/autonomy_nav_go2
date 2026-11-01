import csv
from pathlib import Path
import sys
from types import SimpleNamespace
import time
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from temperature_capture import TemperatureCapture, thermal_rows


def test_missing_and_old_data_are_distinguishable():
    missing = list(thermal_rows(9_000_000_000, 10, ('SEARCH', 'm1', 8_000_000_000), None, None))
    assert len(missing) == 26
    assert all(r['temperature_c'] == '' and not r['available'] and r['error'] == 'NO_DATA' for r in missing)
    legs = dict(receipt_ns=1_000_000_000, motor=[30]*12, valid=[True]*12)
    rows = list(thermal_rows(9_000_000_000, 10, ('HOME', 'm1', 8_000_000_000), legs, None))
    assert rows[0]['receipt_age_s'] == 8
    assert rows[0]['mission_status_age_s'] == 1
    assert rows[0]['common_frame_age_s'] == ''  # LowState has no sensor timestamp.
    assert rows[0]['temperature_c'] == 30


def test_invalid_and_nonfinite_values_never_become_zero():
    arm = dict(mos=[float('nan')]*7, rotor=[45]*7, valid=[False]+[True]*6)
    rows = list(thermal_rows(10, 10, ('UNKNOWN', '', 0), None, arm))[12:]
    assert all(r['temperature_c'] == '' for r in rows[::2])
    assert rows[1]['temperature_c'] == ''
    assert rows[3]['temperature_c'] == 45


def test_native_temperature_sampling_preserves_frame_age_and_rejects_invalid_data(tmp_path):
    import json
    pytest.importorskip('rclpy')
    class Node:
        def create_subscription(self, *args, **kwargs):
            assert args[1] != '/lowstate'
            return (args, kwargs)
    capture = TemperatureCapture(Node(), tmp_path)
    now = time.monotonic_ns()
    frame = now-100_000_000
    message = dict(observed_ns=now, frame_received_ns=frame, robot_tick=123,
                   source_session_id='collector1', valid=True, temperature_c=list(range(31,43)))
    capture.receive_legs(SimpleNamespace(data=json.dumps(message)))
    import yaml
    arm = dict(frame_received_ns=frame, connected=True, valid=[True]*7,
               motor_id=list(range(1, 8)), mos_c=[35]*7, rotor_c=[41]*7, owner_session_id='owner1')
    capture.receive_arm(SimpleNamespace(data=yaml.safe_dump(arm)))
    capture.sample(time.monotonic_ns(), 123)
    # A new periodic publication repeats the same source frame, not a new measurement.
    message['observed_ns']=time.monotonic_ns()
    capture.receive_legs(SimpleNamespace(data=json.dumps(message)))
    capture.sample(now+2_000_000_000, 124)
    assert capture.legs['frame_ns'] == frame
    capture.receive_arm(SimpleNamespace(data='valid: [true]'))
    capture.sample(now+4_000_000_000, 125)
    capture.close()
    rows = list(csv.DictReader((tmp_path/'temperatures.csv').open()))
    assert len(rows) == 78
    assert rows[0]['temperature_c'] == '31'
    assert rows[12]['temperature_c'] == '35'
    assert rows[13]['temperature_c'] == '41'
    assert float(rows[26]['common_frame_age_s']) > 2
    assert rows[0]['source_session_id'] == 'collector1'
    assert rows[64]['temperature_c'] == '' and rows[64]['error']


def test_future_native_frame_and_missing_source_never_become_valid(tmp_path):
    import json
    pytest.importorskip('rclpy')
    class Node:
        def create_subscription(self, *args, **kwargs):return None
    capture=TemperatureCapture(Node(),tmp_path)
    now=time.monotonic_ns()
    message=dict(observed_ns=now,frame_received_ns=now+1,robot_tick=1,
                 source_session_id='s',valid=True,temperature_c=[30]*12)
    capture.receive_legs(SimpleNamespace(data=json.dumps(message)))
    capture.sample(time.monotonic_ns(),1)
    assert capture.legs['error'] == 'ValueError'
    capture.close()
