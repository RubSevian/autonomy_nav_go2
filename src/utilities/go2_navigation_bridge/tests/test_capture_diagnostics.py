"""Reproduce the Jazzy diagnostic byte-field failure without ROS init/DDS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from nav_capture_common import diagnostic_statuses
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue

class CaptureDiagnosticsTests(unittest.TestCase):
    def test_actual_jazzy_diagnostics_are_json_serializable(self):
        message=DiagnosticArray()
        for level in (DiagnosticStatus.OK,DiagnosticStatus.WARN,DiagnosticStatus.ERROR,DiagnosticStatus.STALE):
            status=DiagnosticStatus();status.level=level;status.name='pathFollower';status.message='probe'
            status.values=[KeyValue(key='path_fresh',value='true')];message.status.append(status)
        rows=json.loads(json.dumps(diagnostic_statuses(message)))
        self.assertEqual([row['level'] for row in rows],[0,1,2,3])
        self.assertEqual(rows[0]['values'],{'path_fresh':'true'})

if __name__=='__main__':unittest.main()
