"""DDS discovery outside the control tick; stale snapshots fail closed."""
import threading
import time

TOPICS = ('/go2/navigation/planner_heartbeat', '/navigation_active', '/go2/mission/health')

class GraphSnapshot:
    def __init__(self, query, period, clock=time.monotonic):
        self.query, self.period, self.clock = query, period, clock
        self.snapshot = (None, {})
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='nav_graph', daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            started = self.clock()
            try:
                values = {topic: tuple(self.query(topic)) for topic in TOPICS}
            except Exception:
                values = {}
            # Start time charges all query latency against freshness.
            self.snapshot = (started, values)
            self.stop.wait(self.period)

    def get(self, topic):
        stamp, values = self.snapshot
        if stamp is None or not 0 <= self.clock()-stamp <= min(.5, 2.5*self.period):
            return ()
        return values.get(topic, ())

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        if self.thread.is_alive():
            raise RuntimeError('DDS graph query did not stop; retain node until process exit')
