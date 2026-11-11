import sys
import time
import threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from graph_snapshot import GraphSnapshot, TOPICS

def test_stalled_discovery_expires_without_blocking_reader():
    entered=threading.Event();release=threading.Event();now=[10.]
    def query(topic):
        if now[0]>10.:
            entered.set();release.wait(2)
        return [topic]
    cache=GraphSnapshot(query,.01,clock=lambda:now[0])
    try:
        deadline=time.monotonic()+1
        while not cache.get(TOPICS[0]) and time.monotonic()<deadline:time.sleep(.001)
        assert cache.get(TOPICS[0])==(TOPICS[0],)
        now[0]=11.
        assert entered.wait(1)
        began=time.monotonic()
        assert cache.get(TOPICS[0])==()
        assert time.monotonic()-began < .05
    finally:
        release.set();cache.close()

def test_query_error_revokes_snapshot():
    def query(topic):raise RuntimeError('discovery unavailable')
    cache=GraphSnapshot(query,.01)
    try:assert cache.get(TOPICS[0])==()
    finally:cache.close()
