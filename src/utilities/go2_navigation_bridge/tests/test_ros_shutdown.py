import sys
import threading
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ros_shutdown import close_with_timeout

class CleanupTests(unittest.TestCase):
    def test_clean_failed_and_hung_cleanup_are_distinct(self):
        self.assertTrue(close_with_timeout(lambda:None,.1))
        def fail():raise RuntimeError('DDS')
        self.assertFalse(close_with_timeout(fail,.1))
        release=threading.Event();started=time.monotonic()
        try:
            self.assertFalse(close_with_timeout(release.wait,.03))
            self.assertLess(time.monotonic()-started,.3)
        finally:release.set()
