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

from pathlib import Path
import sys
import unittest

import numpy as np
from transform_sensors.point_records import PointRecords

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class PointRecordsTest(unittest.TestCase):
    def test_padding_fields_order_and_owned_storage_survive_selection(self):
        dtype = np.dtype(
            {
                'names': ['x', 'intensity', 'ring', 'time'],
                'formats': ['<f4', '<f4', '<u2', '<f4'],
                'offsets': [0, 16, 20, 24],
                'itemsize': 32,
            }
        )
        original = np.arange(96, dtype=np.uint8).reshape(3, 32)
        points = original.view(dtype).reshape(-1)
        points['x'] = [1.0, 2.0, 3.0]
        points['time'] = [0.01, 0.02, 0.03]
        before = original.copy()
        records = PointRecords(points)
        records.points['x'] += 5.0
        self.assertTrue(np.array_equal(original, before))
        selected = records.select(np.array([True, False, True]))
        self.assertEqual(selected.dtype, dtype)
        self.assertEqual(selected['x'].tolist(), [6.0, 8.0])
        covered = np.zeros(32, dtype=bool)
        for field, offset in dtype.fields.values():
            covered[offset: offset + field.itemsize] = True
        output = selected.view(np.uint8).reshape(2, 32)
        self.assertTrue(np.array_equal(output[:, ~covered], before[[0, 2]][:, ~covered]))
        for name in ('intensity', 'ring', 'time'):
            self.assertEqual(selected[name].tobytes(), points[[0, 2]][name].tobytes())
        records.points['x'] = 100.0
        self.assertEqual(selected['x'].tolist(), [6.0, 8.0])

    def test_empty_selection_and_invalid_layout(self):
        points = np.zeros(2, dtype=[('x', 'f4'), ('time', 'f4')])
        records = PointRecords(points)
        self.assertEqual(records.select(np.array([False, False])).size, 0)
        with self.assertRaises(ValueError):
            PointRecords(np.zeros(3))


if __name__ == '__main__':
    unittest.main()
