from pathlib import Path
import sys
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from input_timing import InputTiming


def test_timing_distinguishes_header_age_callback_work_and_silence():
    now=[10_000_000_000];d=InputTiming(lambda:now[0],lambda:now[0])
    msg=NS(header=NS(stamp=NS(sec=9,nanosec=800_000_000)))
    with d.callback('odom',msg):now[0]+=30_000_000
    now[0]+=70_000_000
    row=d.snapshot()['odom']
    assert row['header_to_callback_s']==.2 and row['work_s']==.03 and row['silence_s']==.1
    assert msg.header.stamp.sec==9
