# Copyright (c) 2026 RubSevian
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

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
