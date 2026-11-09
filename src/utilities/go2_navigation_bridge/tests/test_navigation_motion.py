"""Motion evidence examples; constants here are synthetic, not robot calibration."""
from pathlib import Path
import sys
import csv
import json
import math
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_motion import MotionEvidence
from nav_stationary_noise import replay as stationary_replay, long_term_segments


class MotionTests(unittest.TestCase):
    def test_long_term_distinguishes_return_from_accumulated_shift(self):
        def points(fn):
            return [(fn(i*.05),0.,1_000_000_000+i*50_000_000,0.) for i in range(401)]
        drift=long_term_segments(points(lambda t:t*.005))[0]
        oscillation=long_term_segments(points(lambda t:.02*math.sin(t*math.pi/5)))[0]
        self.assertGreater(drift['endpoint_shift_m'],.09)
        self.assertLess(oscillation['endpoint_shift_m'],.013)
        self.assertGreater(oscillation['max_distance_from_start_m'],.02)
        # Separate brief fragments cannot be mistaken for a long stand.
        split=points(lambda t:0.)
        split=[(x,y,stamp+(20_000_000_000 if i>=180 else 0),.3 if i>=350 else age)
               for i,(x,y,stamp,age) in enumerate(split)]
        self.assertEqual(long_term_segments(split),[])

    def replay(self, evaluator, points, start=1_000_000_000):
        return [evaluator.observe(x,y,start+i*50_000_000,0.) for i,(x,y) in enumerate(points)]

    def test_uncalibrated_never_confirms_even_large_real_motion(self):
        results=self.replay(MotionEvidence(),[(i*.01,0.) for i in range(100)])
        self.assertTrue(all(not r.confirmed for r in results))
        self.assertEqual(results[-1].reason,'UNCALIBRATED')

    def test_small_translation_detected_without_ten_centimetres(self):
        m=MotionEvidence(noise_bound_m=.008)
        results=self.replay(m,[(i*.001,0.) for i in range(60)])
        self.assertTrue(any(r.confirmed for r in results))
        self.assertLess(results[-1].displacement_m,.03)

    def test_jitter_and_isolated_outlier_do_not_confirm(self):
        points=[(.002*(-1)**i,.001*(-1)**(i//2)) for i in range(120)]
        points[50]=(.01,0.)
        results=self.replay(MotionEvidence(noise_bound_m=.008),points)
        self.assertFalse(any(r.confirmed for r in results))

    def test_duplicate_cannot_complete_confirmation(self):
        m=MotionEvidence(noise_bound_m=.008)
        for i in range(100):
            r=m.observe(i*.001,0.,1_000_000_000+i*50_000_000,0.)
            if r.reason=='CANDIDATE_MOTION':break
        self.assertEqual(r.reason,'CANDIDATE_MOTION')
        for _ in range(20):
            self.assertEqual(m.observe(i*.001,0.,1_000_000_000+i*50_000_000,0.).reason,
                             'DUPLICATE_SAMPLE')

    def test_jump_stale_revision_and_gap_clear_history(self):
        for cause in ('jump','stale','revision','gap'):
            m=MotionEvidence(noise_bound_m=.008)
            self.replay(m,[(i*.001,0.) for i in range(30)])
            self.assertTrue(m.result.confirmed)
            args={'jump':(1.,0.,2_500_000_000,0.),
                  'stale':(.03,0.,2_500_000_000,.3),
                  'revision':(.04,0.,2_450_000_000,0.),
                  'gap':(.03,0.,3_000_000_000,0.)}[cause]
            r=m.observe(*args)
            self.assertFalse(r.confirmed)
            self.assertLessEqual(len(m.history),1)

    def test_turn_in_place_has_no_translation(self):
        self.assertFalse(any(r.confirmed for r in self.replay(
            MotionEvidence(noise_bound_m=.008),[(0.,0.)]*100)))

    def test_small_motion_inside_noise_is_unknown_not_proven_stationary(self):
        results=self.replay(MotionEvidence(noise_bound_m=.03),[(i*.001,0.) for i in range(60)])
        self.assertEqual(results[-1].reason,'WITHIN_NOISE_ENVELOPE')
        self.assertFalse(results[-1].confirmed)

    def test_bounded_memory_and_bad_configuration(self):
        m=MotionEvidence(noise_bound_m=.008,capacity=6)
        results=self.replay(m,[(0.,0.)]*100)
        self.assertIn('HISTORY_CAPACITY',[r.reason for r in results])
        self.assertLessEqual(len(m.history),6)
        for kwargs in ({'noise_bound_m':0.},{'noise_bound_m':float('nan')},
                       {'noise_bound_m':True},{'endpoint_s':1.},{'capacity':True}):
            with self.assertRaises(ValueError):MotionEvidence(**kwargs)

    def test_stationary_holdout_failure_is_not_hidden_by_retuning(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'navigation_profile.yaml').write_text(
                'robot:\n  sensor_offset_x_m: 0.0\n  sensor_offset_y_m: 0.0\n')
            with (p/'odometry.csv').open('w') as stream:
                w=csv.writer(stream)
                w.writerow(['header_ns','receipt_ros_ns','clock_pair_span_ns','finite',
                            'frame','child','x','y','z','qx','qy','qz','qw'])
                for i in range(1000):
                    stamp=1_000_000_000+i*50_000_000
                    # Only the held-out part has slow apparent motion. A correct
                    # report must reject calibration, not fit to that same part.
                    x=.001*math.sin(i*.2)+(max(0,i-750)*.001)
                    w.writerow([stamp,stamp,1,True,'camera_init','aft_mapped',x,0,0,0,0,0,1])
            with self.assertRaises(ValueError):stationary_replay(p)
            r=stationary_replay(p,stationary_confirmed=True)
            self.assertFalse(r['stationary_replay_passed'])
            self.assertGreater(r['holdout_reasons']['MEASURED_MOTION'],0)
            self.assertFalse(r['deployable'])
            self.assertLess(r['candidate_bound_m'],.01)
            # Zero commanded velocity during laydown is NOT a stationary label.
            with (p/'odometry.csv').open() as f:rows=list(csv.DictReader(f))
            with (p/'odometry.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0])+['receipt_mono_ns'])
                writer.writeheader()
                for row in rows:
                    row['receipt_mono_ns']=row['header_ns'];writer.writerow(row)
            with (p/'status.jsonl').open('w') as f:
                for i in range(0,1000,10):
                    data=dict(observed_ns=1_000_000_000+i*50_000_000,
                              state='ACTIVE' if i<700 else 'CONTROLLED_STOP',
                              phase='RL_ZERO' if i<700 else 'LIE_DOWN',
                              output_enabled=True,motion_command=[0,0,0])
                    f.write(json.dumps(dict(topic='/go2/locomotion_status',data=data))+'\n')
            selected=stationary_replay(p,stationary_confirmed=True,require_stable_rl=True)
            self.assertTrue(selected['stationary_replay_passed'])
            self.assertGreater(selected['rl_rejected_rows'],250)


if __name__=='__main__':unittest.main()
