"""Deterministic unit stress only. No ROS, robot, CUDA or Unitree API."""
import importlib.util
import json
from pathlib import Path
import sys
import time
import unittest
import numpy as np

from transform_sensors.bounded_time_alignment import BoundedConfig, BoundedTimeAlignment, NS, finite_imu_fields
from transform_sensors.point_records import PointRecords
from transform_sensors.time_alignment import SensorTimeAlignment
NAV = Path(__file__).resolve().parents[2] / 'go2_navigation_bridge/scripts/navigation_gate.py'
spec=importlib.util.spec_from_file_location('production_navigation_gate', NAV)
nav=importlib.util.module_from_spec(spec);spec.loader.exec_module(nav)
S = 1730721639 * NS
R = 1791732363 * NS
M = 1000 * NS
RESULTS = {}


def events(seconds, ppm=0., jitter=False):
    result = []
    for stream, hz, span in [('imu', 250, 0), ('cloud', 15, 64_000_000)]:
        for i in range(round(seconds * hz)):
            t = i / hz
            source = S + round(t * (1 + ppm / 1e6) * NS)
            delay = 2_000_000 + (i % 13) * 150_000 if jitter else 2_000_000
            if jitter and i % 137 == 0:
                delay += 80_000_000
            arrival = round(t * NS) + round(span / (1 + ppm / 1e6)) + delay
            result.append((arrival, stream, source, span))
    result.sort()
    return result


def feed(clock, rows, check=True):
    previous = {};accepted=0;max_span_change=0
    for arrival, stream, source, span in rows:
        mapped = clock.map_stamp(stream, source, R + arrival, M + arrival, span)
        if mapped is not None:
            if check:
                assert mapped > previous.get(stream, 0)
                assert R + arrival - mapped <= 500_000_000
                assert clock.map_time(source + span) <= R + arrival
                if span:
                    max_span_change=max(max_span_change,abs(clock.map_time(source+span)-mapped-span))
            previous[stream]=mapped;accepted+=1
    return accepted,max_span_change


def locked():
    c=BoundedTimeAlignment();feed(c,events(3));assert c.state=='LOCKED';return c


class BoundedTests(unittest.TestCase):
    def test_847ns_boundary_reconstructed_not_exact_replay(self):
        c=locked();source=S+3*NS+100_000_000;receive=c.map_time(source)-847
        self.assertIsNone(c.map_stamp('imu',source,receive,M+(receive-R)))
        self.assertEqual(c.state,'LOCKED');self.assertEqual(c.reason,'small_future_drop')
        self.assertEqual(c.last_end_residual_ns['imu'],-847)
        baseline=SensorTimeAlignment()
        for arrival,stream,stamp,span in events(3):
            baseline.map_stamp(stream,stamp,R+arrival,M+arrival,span)
        self.assertIsNone(baseline.map_stamp('imu',source,receive,M+receive-R))
        self.assertEqual(baseline.state,'FAULT')
        RESULTS['boundary_847ns']={'kind':'RECONSTRUCTED_BOUNDARY_NOT_EXACT_REPLAY','residual_ns':-847,
                                   'old_state':baseline.state,'new_state':c.state,'new_decision':c.reason}
        source+=4_000_000
        self.assertIsNotNone(c.map_stamp('imu',source,R+3*NS+106_000_000,M+3*NS+106_000_000))

    def test_persistent_small_future_latches_and_never_clamps(self):
        c=locked()
        for i in range(3):
            source=S+3*NS+100_000_000+i*4_000_000;receive=c.map_time(source)-847
            self.assertIsNone(c.map_stamp('imu',source,receive,M+receive-R))
        self.assertEqual(c.state,'DEGRADED');self.assertEqual(c.failure_reason,'persistent_small_future')
        self.assertIsNone(c.map_stamp('imu',S+4*NS,R+4*NS,M+4*NS))

    def test_20min_positive_negative_and_small_drift(self):
        for ppm in (309.,-309.,21.7,0.):
            with self.subTest(ppm=ppm):
                c=BoundedTimeAlignment();rows=events(1200,ppm,jitter=True);begin=time.process_time_ns()
                count,span_delta=feed(c,rows)
                cpu=time.process_time_ns()-begin
                RESULTS[str(ppm)]={'kind':'SYNTHETIC_UNIT_STRESS','seconds':1200,'calls':len(rows),'accepted':count,
                                  'state':c.state,'failure_reason':c.failure_reason,'rate_ppb':c.rate_ppb,
                                  'updates':c.model_updates,'segments':len(c._segments),'envelopes':len(c._envelopes),
                                  'CPU_ns':cpu,'CPU_ns_per_call':cpu/len(rows),'max_cloud_span_change_ns':span_delta,
                                  'rejects':dict(c.reject_reasons)}
                self.assertEqual(c.state,'LOCKED',c.status())
                self.assertLessEqual(len(c._segments),16);self.assertLessEqual(len(c._envelopes),8)
                self.assertGreater(count,310000)
                self.assertLessEqual(abs(c.rate_ppb),1000000)

    def test_large_future_is_safe_degraded_not_published(self):
        for residual in (-2001,-1_000_000,-50_000_000):
            c=locked();source=S+3*NS+100_000_000;receive=c.map_time(source)+residual
            self.assertIsNone(c.map_stamp('imu',source,receive,M+receive-R))
            self.assertEqual(c.state,'DEGRADED')

    def test_source_reset_and_forward_jump_fault(self):
        for source in (S-NS,S+5*NS):
            c=locked();self.assertIsNone(c.map_stamp('imu',source,R+3*NS,M+3*NS));self.assertEqual(c.state,'FAULT')

    def test_host_jumps_and_sequential_clock_read_noise(self):
        for jump in (-NS,NS):
            c=locked();self.assertIsNone(c.map_stamp('imu',S+3*NS,R+3*NS+jump,M+3*NS));self.assertEqual(c.state,'FAULT')
        c=locked()
        for i in range(50):
            t=3*NS+i*4_000_000
            self.assertIsNotNone(c.map_stamp('imu',S+t,R+t+2_000_000,M+t+2_000_000+(i%7)*2000))
        self.assertEqual(c.state,'LOCKED')

    def test_outage_restart_and_stale_cannot_become_fresh(self):
        for gap in (.5,.6,2.):
            c=locked();c.check_health(M+3*NS+round(gap*NS));self.assertEqual(c.state,'DEGRADED')
            self.assertIsNone(c.map_stamp('imu',S+4*NS,R+4*NS,M+4*NS))
        c=locked();source=S+3*NS;self.assertIsNone(c.map_stamp('imu',source,R+3*NS+600_000_000,M+3*NS+600_000_000))
        self.assertNotEqual(c.state,'LOCKED')

    def test_qos_duplicates_and_reordering_do_not_modify_model(self):
        c=locked();old=c._last_source['imu'];segments=list(c._segments)
        for source in (old,old-1_000_000):
            self.assertIsNone(c.map_stamp('imu',source,R+3*NS,M+3*NS))
        self.assertEqual(c.state,'LOCKED');self.assertEqual(list(c._segments),segments)

    def test_invalid_nan_span_and_i64(self):
        for source,span in [(0,0),(float('nan'),0),(S,-1),(S,200_000_001),(S,float('nan')),(1<<63,0)]:
            c=BoundedTimeAlignment();self.assertIsNone(c.map_stamp('cloud',source,R,M,span));self.assertEqual(c.seen['cloud'],0)
        c=locked()
        for values in ([float('nan')],[-.1],[.201]):
            with self.assertRaises(ValueError):c.map_relative_times(S+3*NS,values)
        for kw in ({'small_future_ns':2001},{'max_age_s':.6},{'max_rate_ppm':2001},{'segment_capacity':100}):
            with self.assertRaises(ValueError):BoundedConfig(**kw)

    def test_common_map_hinges_point_layout_and_epoch_precision(self):
        c=locked();anchor=S+4*NS;value=c.map_time(anchor);c._segments.append((anchor,value,-309000))
        self.assertEqual(c.map_time(anchor),value)
        self.assertLessEqual(c.map_time(anchor-1),c.map_time(anchor))
        header=anchor-20_000_000;times=np.array([0.,.01,.02,.03,.064,.2],dtype=np.float64)
        mapped=c.map_relative_times(header,times)
        for t,v in zip(times,mapped):
            point=header+round(t*NS)
            self.assertEqual(round(v*NS)+c.map_time(header),c.map_time(point))
        self.assertLessEqual(abs(round(mapped[-1]*NS)-200_000_000),200000)
        self.assertEqual(c.map_time(anchor+1)-c.map_time(anchor),1)
        dtype=np.dtype({'names':['x','y','z','intensity','time','ring'],'formats':['f4']*5+['u2'],
                        'offsets':[0,4,8,16,20,24],'itemsize':32})
        raw=np.arange(6*32,dtype=np.uint8).reshape(6,32).copy();pts=raw.view(dtype).reshape(-1);pts['time']=times
        before=raw.copy();records=PointRecords(pts);records.points['time']=mapped
        mask=np.ones(32,dtype=bool);mask[20:24]=False
        self.assertTrue(np.array_equal(records.points.view('u1').reshape(-1,32)[:,mask],before[:,mask]))
        self.assertTrue(np.array_equal(raw,before));self.assertEqual(records.points.dtype,dtype)

    def test_no_scan_retroactive_change_or_imu_coverage_loss(self):
        c=BoundedTimeAlignment();history=[]
        for arrival,stream,source,span in events(30,309):
            result=c.map_stamp(stream,source,R+arrival,M+arrival,span)
            if result is not None:
                for x,v in history[-20:]:self.assertEqual(c.map_time(x),v)
                if stream=='cloud':
                    end=c.map_time(source+span);history.append((source+span,end))
                    self.assertGreaterEqual(end,result)
                    self.assertEqual(c.map_time(source+span),end) # same clock as IMU at scan end
        self.assertEqual(c.state,'LOCKED')

    def test_valid_max_scan_span_and_invalid_imu_fields(self):
        c=locked();source=S+3*NS+100_000_000
        result=c.map_stamp('cloud',source,R+3*NS+302_000_000,M+3*NS+302_000_000,200_000_000)
        self.assertIsNotNone(result)
        self.assertEqual(c.map_time(source+200_000_000)-result,200_000_000)
        self.assertTrue(finite_imu_fields((0.,1.,-1.)))
        for value in (float('nan'),float('inf'),None,'1'):
            self.assertFalse(finite_imu_fields((0.,value)))

    def test_unobservable_rate_or_large_uncertainty_forbids_output(self):
        c=locked();c._high_source=S+10*NS
        c._envelopes.extend([(S+i*NS,R-S+i*10_000_000) for i in range(8)])
        c._update_model();self.assertEqual(c.state,'DEGRADED')
        self.assertEqual(c.failure_reason,'estimated_rate_outside_bound')
        c=locked();c._high_source=S+10*NS
        c._envelopes.extend([(S+i*NS,R-S+(10_000_000 if i%2 else 0)) for i in range(8)])
        c._update_model();self.assertEqual(c.state,'DEGRADED')
        self.assertEqual(c.failure_reason,'envelope_uncertainty_exceeded')

    def test_one_bounded_transport_slope_window_recovers_but_repeat_latches(self):
        cfg=BoundedConfig(max_uncertainty_s=.01)
        c=BoundedTimeAlignment(cfg);feed(c,events(3));c._high_source=S+10*NS
        c._envelopes.clear()
        c._envelopes.extend((S+i*NS,R-S+i*1_500_000) for i in range(6))
        c._update_model()
        self.assertEqual(c.state,'LOCKED')
        self.assertEqual(c.last_rate_estimate_ppb,1_500_000)
        self.assertLessEqual(c.last_bounded_fit_error_ns,10_000_000)
        self.assertLessEqual(abs(c.rate_ppb),1_000_000)
        c._envelopes.clear()
        c._envelopes.extend((S+i*NS,R-S) for i in range(6))
        c._update_model()
        self.assertEqual(c.state,'LOCKED')
        self.assertEqual(c.consecutive_out_of_bound_rate_windows,0)
        c._envelopes.clear()
        c._envelopes.extend((S+i*NS,R-S+i*1_500_000) for i in range(6))
        c._update_model();self.assertEqual(c.state,'LOCKED')
        c._update_model()
        self.assertEqual(c.state,'DEGRADED')
        self.assertEqual(c.failure_reason,'estimated_rate_outside_bound')

    def test_out_of_bound_rate_needs_bounded_uncertainty(self):
        c=BoundedTimeAlignment(BoundedConfig(max_uncertainty_s=.01))
        feed(c,events(3));c._high_source=S+10*NS
        c._envelopes.extend((S+i*NS,R-S+i*4_000_000) for i in range(8))
        c._update_model()
        self.assertEqual(c.state,'DEGRADED')
        self.assertGreater(c.last_bounded_fit_error_ns,10_000_000)
        with self.assertRaises(ValueError):
            BoundedConfig(max_out_of_bound_rate_windows=3)

    def test_configured_2000ppm_accepts_observed_1149ppm_but_keeps_other_guards(self):
        c=BoundedTimeAlignment(BoundedConfig(max_rate_ppm=2000.0,max_uncertainty_s=.01))
        feed(c,events(3));c._high_source=S+10*NS
        c._envelopes.clear()
        c._envelopes.extend((S+i*NS,R-S+i*1_149_433) for i in range(8))
        c._update_model()
        self.assertEqual(c.state,'LOCKED')
        self.assertEqual(c.last_rate_estimate_ppb,1_149_433)
        self.assertEqual(c.consecutive_out_of_bound_rate_windows,0)
        self.assertLessEqual(abs(c.rate_ppb),2_000_000)
        self.assertEqual(c.config.max_age_s,.5)
        self.assertEqual(c.config.max_uncertainty_s,.01)
        self.assertEqual(c.config.rate_slew_ppm_s,100.0)
        with self.assertRaises(ValueError):BoundedConfig(max_rate_ppm=2000.1)

    def test_configured_10ms_uncertainty_boundary(self):
        for residual,expected in ((5_245_479,'LOCKED'),(10_000_000,'LOCKED'),(10_000_001,'DEGRADED')):
            c=BoundedTimeAlignment(BoundedConfig(max_uncertainty_s=.01))
            feed(c,events(3));c._high_source=S+10*NS
            c._envelopes.extend([(S+i*NS,R-S+(residual if i%2 else 0)) for i in range(8)])
            c._update_model()
            self.assertEqual(c.uncertainty_ns,residual)
            self.assertEqual(c.state,expected)
        self.assertEqual(BoundedConfig(max_uncertainty_s=.010001).max_uncertainty_s,.010001)

    def test_configured_20ms_uncertainty_boundary_keeps_age_guard(self):
        for residual,expected in ((10_668_714,'LOCKED'),(20_000_000,'LOCKED'),(20_000_001,'DEGRADED')):
            c=BoundedTimeAlignment(BoundedConfig(max_uncertainty_s=.02))
            feed(c,events(3));c._high_source=S+10*NS
            c._envelopes.extend([(S+i*NS,R-S+(residual if i%2 else 0)) for i in range(8)])
            c._update_model()
            self.assertEqual(c.uncertainty_ns,residual)
            self.assertEqual(c.state,expected)
            self.assertEqual(c.config.max_age_s,.5)
        with self.assertRaises(ValueError):BoundedConfig(max_uncertainty_s=.020001)

    def test_single_positive_delay_outlier_does_not_set_rate(self):
        c=BoundedTimeAlignment();feed(c,events(25,0,jitter=True))
        self.assertEqual(c.state,'LOCKED');self.assertLess(abs(c.rate_ppb),100000)

    def test_gate_invalid_mapping_zero_and_no_self_rearm(self):
        g=nav.NavigationGate(mode='command_test',clock_required=True)
        g.observe_alignment('LOCKED',True,1.);g.locomotion(True,1.);g.observe('stop',True,1.)
        self.assertTrue(g.accept((.1,0,0),0.,'vehicle',1.));self.assertEqual(g.output(1.),(.1,0,0))
        g.observe_alignment('DEGRADED',False,1.01);self.assertEqual(g.output(1.01),nav.ZERO)
        g.observe_alignment('LOCKED',True,1.02);g.resume();g.locomotion(True,1.02)
        self.assertFalse(g.accept((.1,0,0),0.,'vehicle',1.02));self.assertEqual(g.output(1.02),nav.ZERO)
        self.assertTrue(g.clock_failed)

    def test_adaptation_cannot_rejuvenate_reference_stale_packet(self):
        c=locked();source=S+3*NS+100_000_000
        anchor=source-1_000_000;c._segments.append((anchor,c.map_time(anchor)+600_000_000,0))
        # Deliberately corrupted model fixture: fresh moving model cannot bypass
        # the unchanged reference age. No such discontinuity exists in updates.
        receive=source+c.offset_ns+610_000_000
        c._last_arrival={'imu':M+receive-R,'cloud':M+receive-R}
        self.assertIsNone(c.map_stamp('imu',source,receive,M+receive-R))
        self.assertEqual(c.failure_reason,'measurement_stale_restart_required')

    def test_gate_clock_status_ttl_and_invalid_age(self):
        for age in (-.001,.501,float('nan')):
            g=nav.NavigationGate(mode='command_test',clock_required=True);g.observe_alignment('LOCKED',True,1.,age)
            self.assertFalse(g.clock_ready(1.))
        g=nav.NavigationGate(mode='command_test',clock_required=True);g.observe_alignment('LOCKED',True,1.)
        self.assertFalse(g.clock_ready(1.501));self.assertTrue(g.clock_failed)


if __name__ == '__main__':
    unittest.main()
