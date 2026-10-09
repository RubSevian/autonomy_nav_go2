"""Cheap message-layout checks; full point processing belongs to Point-LIO."""
from collections import deque


class OdometryHistory:
    """Bounded samples for asynchronous odometry/TF delivery on separate topics."""
    def __init__(self):
        self._samples = deque(maxlen=32)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def observe(self, msg):
        self._samples.append(msg)

    def clear(self):
        self._samples.clear()

    def at_or_before(self, stamp_ns):
        return max((msg for msg in self._samples if self.stamp(msg) <= stamp_ns),
                   key=self.stamp, default=None)

def valid_cloud_layout(msg):
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    fields = {field.name: field for field in msg.fields}
    if len(fields) != len(msg.fields):
        return False
    if not {'x', 'y', 'z', 'intensity'}.issubset(fields) or not ({'time', 't'} & fields.keys()):
        return False
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        return False
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        return False
    for field in fields.values():
        size = sizes.get(field.datatype, 0)
        if not size or field.count <= 0 or field.offset < 0 or field.offset + size * field.count > msg.point_step:
            return False
    return all(fields[name].datatype in (7, 8) and fields[name].count == 1 for name in ('x', 'y', 'z'))
