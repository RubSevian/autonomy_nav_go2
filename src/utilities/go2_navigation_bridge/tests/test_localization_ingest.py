"""Queue backlog reproduction using real pairing logic; no robot or DDS."""
from collections import deque
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from localization_ingest import drain_pending
from localization_pairing import LocalizationPairing,PairingConfig,PoseSample


class Handle:
    def __init__(self,rows):self.rows=deque(rows)
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def take_message(self,*args):return (self.rows.popleft(),None) if self.rows else None


def subscription(rows):return NS(handle=Handle(rows),msg_type=object,raw=False)


def test_backlog_requires_draining_both_streams_and_keeps_original_stamps():
    now=10_000_000_000
    poses=[PoseSample(now-2_000_000_000+i*10_000_000,'camera_init','aft_mapped',
                     (0.,0.,0.),(0.,0.,0.,1.)) for i in range(20)]
    poses.append(replace(poses[-1],stamp_ns=now-100_000_000))
    p=LocalizationPairing(PairingConfig(max_age_s=.5))
    odom=subscription(poses);tf=subscription(poses)
    resolve=lambda o,t:replace(t,frame='map',child='vehicle')
    drain_pending(odom,p.observe_odom,1);drain_pending(tf,p.observe_tf,1)
    assert p.select(now,resolve) is None
    assert drain_pending(odom,p.observe_odom)==20
    assert p.select(now,resolve) is None  # never synthesize TF from odometry
    assert drain_pending(tf,p.observe_tf)==20
    assert p.select(now,resolve).odom.stamp_ns==now-100_000_000
    assert p.select(now+600_000_000,resolve) is None  # no lease renewal


def test_backlog_work_is_bounded_and_order_preserved():
    sub=subscription(range(100));seen=[]
    assert drain_pending(sub,seen.append)==32
    assert seen==list(range(32)) and len(sub.handle.rows)==68
