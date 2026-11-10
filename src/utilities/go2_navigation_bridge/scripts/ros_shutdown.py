"""Bounded DDS cleanup for subscriber-only recording processes, never actuators."""
import threading


def close_with_timeout(close, timeout_s=5.):
    outcome=[]
    def run():
        try:close();outcome.append(True)
        except Exception:outcome.append(False)
    worker=threading.Thread(target=run,daemon=True,name='passive-dds-cleanup')
    worker.start();worker.join(timeout_s)
    return bool(outcome and outcome[0] and not worker.is_alive())
