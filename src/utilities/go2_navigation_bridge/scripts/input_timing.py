"""Local callback timings only; no freshness decisions or clock correction."""
from contextlib import contextmanager
from functools import wraps
import time


class InputTiming:
    def __init__(self, ros_ns, mono_ns=time.monotonic_ns):
        self.ros_ns=ros_ns;self.mono_ns=mono_ns;self.streams={}

    @contextmanager
    def callback(self, name, message):
        start=self.mono_ns();wall=self.ros_ns()
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        row=self.streams.setdefault(name,dict(count=0,max_gap_s=0.,max_work_s=0.,
            max_header_age_s=0.,stamp_regressions=0))
        gap=(start-row.get('callback_start_ns',start))/1e9
        if stamp<row.get('header_ns',stamp):row['stamp_regressions']+=1
        row.update(count=row['count']+1,callback_start_ns=start,callback_start_ros_ns=wall,
                   header_ns=stamp,header_to_callback_s=(wall-stamp)/1e9,gap_s=gap,
                   max_gap_s=max(row['max_gap_s'],gap))
        row['max_header_age_s']=max(row['max_header_age_s'],row['header_to_callback_s'])
        try:yield
        finally:
            duration=(self.mono_ns()-start)/1e9
            row.update(work_s=duration,max_work_s=max(row['max_work_s'],duration))

    def snapshot(self):
        now=self.mono_ns()
        return {name:dict(row,silence_s=(now-row['callback_start_ns'])/1e9)
                for name,row in self.streams.items()}


def timed_input(name):
    def decorate(function):
        @wraps(function)
        def measured(self, message):
            with self.input_timing.callback(name,message):
                return function(self,message)
        return measured
    return decorate
