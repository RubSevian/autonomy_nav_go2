"""Known relative closure and angular wrap; no ROS or robot."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/nav_drift_analyze.py'

class DriftAnalysisTests(unittest.TestCase):
    def run_case(self, marked=True, complete=True, short=False, automatic=False, unstable=False):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);rows=[]
            for phase,x,y,yaw in [('START_STATIONARY',0.,0.,179.),('MOVING',1.,1.,180.),('RETURN_STATIONARY',.1,.05,-179.)]:
                for i in range(3 if short else 12 if automatic else 6):
                    t=len(rows)*500_000_000
                    rows.append(dict(receipt_mono_ns=t,header_ns=t,phase=phase if marked else 'unmarked',x=x+(i*.1 if unstable and phase=='RETURN_STATIONARY' else 0),y=y,z=0.,yaw=math.radians(yaw),finite=True))
            with (out/'odometry.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
            capture=dict(complete=complete,poses=len(rows))
            if automatic:capture['recording_config']=dict(auto_bookends=True,stationary_window_s=5.,minimum_stationary_s=2.,closure_average_s=2.,stationary_max_radius_m=.05,stationary_max_yaw_range_deg=2.)
            (out/'capture.json').write_text(json.dumps(capture))
            subprocess.run([sys.executable,str(SCRIPT),str(out)],check=True,capture_output=True)
            self.assertTrue((out/'DRIFT_REPORT.md').is_file())
            return json.loads((out/'drift_results.json').read_text())

    def test_known_closure_and_yaw_wrap(self):
        result=self.run_case()
        self.assertEqual(result['status'],'MEASURED_RELATIVE_ONLY')
        self.assertAlmostEqual(result['return_to_mark']['relative_closure_xy_m'],math.hypot(.1,.05))
        self.assertAlmostEqual(result['return_to_mark']['relative_closure_yaw_deg'],2.)
        self.assertTrue(all(s['displacement_xyz_m']==0. for s in result['stationary_segments']))

    def test_incomplete_unmarked_and_short_never_claim_measurement(self):
        for kwargs in [dict(marked=False),dict(complete=False),dict(short=True)]:
            with self.subTest(kwargs=kwargs):
                self.assertEqual(self.run_case(**kwargs)['status'],'INCOMPLETE_OR_UNMARKED')

    def test_automatic_bookends_are_estimates_without_keyboard_marks(self):
        result=self.run_case(marked=False,automatic=True)
        self.assertEqual(result['status'],'ESTIMATED_RELATIVE_AUTO_BOOKENDS')
        self.assertFalse(result['automatic_stationarity_checks']['physical_stops_confirmed'])
        self.assertAlmostEqual(result['return_to_mark']['relative_closure_xy_m'],math.hypot(.1,.05))
        self.assertAlmostEqual(result['return_to_mark']['relative_closure_yaw_deg'],2.)

    def test_automatic_moving_end_short_or_incomplete_never_claim_closure(self):
        for kwargs in [dict(unstable=True),dict(short=True),dict(complete=False)]:
            with self.subTest(kwargs=kwargs):
                self.assertEqual(self.run_case(marked=False,automatic=True,**kwargs)['status'],'INCOMPLETE_OR_UNMARKED')

if __name__=='__main__':unittest.main()
