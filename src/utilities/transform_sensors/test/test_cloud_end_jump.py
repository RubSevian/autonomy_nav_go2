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

import unittest

from test_bounded_alignment import locked
from test_bounded_alignment import M
from test_bounded_alignment import NS
from test_bounded_alignment import R
from test_bounded_alignment import S


class CloudEndTests(unittest.TestCase):
    def test_real_cloud_forward_jump_still_faults(self):
        c = locked()
        self.assertIsNone(c.map_stamp('cloud', S + 5 * NS, R + 3 * NS, M + 3 * NS, 64_000_000))
        self.assertEqual(c.failure_reason, 'sensor_clock_forward_jump')

    def test_cloud_clock_jump_not_hidden_by_shorter_span(self):
        c = locked()
        self.assertIsNone(c.map_stamp('cloud', S + 5 * NS, R + 3 * NS, M + 3 * NS, 0))
        self.assertEqual(c.state, 'FAULT')

    def test_cloud_source_reset_still_faults(self):
        c = locked()
        self.assertIsNone(c.map_stamp('cloud', S - NS, R + 3 * NS, M + 3 * NS, 64_000_000))
        self.assertEqual(c.failure_reason, 'sensor_clock_reset')

    def test_cloud_start_order_check_remains(self):
        c = locked()
        source = c._last_source['cloud']
        self.assertIsNone(c.map_stamp('cloud', source, R + 3 * NS, M + 3 * NS, 190_000_000))
        self.assertEqual(c.reason, 'duplicate_or_out_of_order')
        self.assertEqual(c.state, 'LOCKED')


if __name__ == '__main__':
    unittest.main()
