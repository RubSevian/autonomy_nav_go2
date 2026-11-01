"""Bounded queue drain for the bridge's single-threaded ROS executor."""


def drain_pending(subscription, callback, limit=32):
    if type(limit) is not int or not 1<=limit<=32:
        raise ValueError('localization batch must be in [1,32]')
    count=0
    for _ in range(limit):
        with subscription.handle:
            item=subscription.handle.take_message(subscription.msg_type,subscription.raw)
        if item is None:break
        callback(item[0]);count+=1
    return count
