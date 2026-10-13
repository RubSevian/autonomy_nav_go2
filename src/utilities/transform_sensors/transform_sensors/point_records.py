"""Owned structured point records with deterministic preservation of padding."""
import numpy as np


class PointRecords:
    def __init__(self, points):
        if points.dtype.names is None or not points.flags.c_contiguous:
            raise ValueError('Contiguous structured point records required')
        self._dtype = points.dtype
        self._bytes = points.view(np.uint8).reshape(-1, points.dtype.itemsize).copy()
        self.points = self._bytes.view(self._dtype).reshape(-1)

    def select(self, mask):
        # Index bytes, not the structured dtype: NumPy field transfer loops can
        # leave unnamed bytes uninitialized. The returned view owns its buffer.
        return self._bytes[mask].view(self._dtype).reshape(-1)
