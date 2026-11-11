"""Bounded queue drain for the bridge's single-threaded ROS executor."""

import time

def drain_pending(subscription, callback, limit=32, budget_s=.002):
    if type(limit) is not int or not 1<=limit<=32:
        raise ValueError('localization batch must be in [1,32]')
    deadline=time.monotonic()+budget_s
    count=0
    for _ in range(limit):
        if time.monotonic() >= deadline:break
        with subscription.handle:
            item=subscription.handle.take_message(subscription.msg_type,subscription.raw)
        if item is None:break
        callback(item[0]);count+=1
    return count
