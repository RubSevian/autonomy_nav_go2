"""Route replan, detour, repeated attempt and bounded-memory contracts."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_progress import RouteProgress, project


class ProgressTests(unittest.TestCase):
    def tracker(self,route=((0.,0.),(3.,0.)),**kwargs):
        p=RouteProgress(noise_bound_m=.008,**kwargs)
        self.assertTrue(p.set_route(route,1));return p

    def move(self,p,points,start=1_000_000_000,allowed=True):
        return [p.observe(x,y,start+i*50_000_000,0.,allowed) for i,(x,y) in enumerate(points)]

    def test_small_forward_motion_and_detour(self):
        for route,points in ((((0.,0.),(3.,0.)),[(.001*i,0.) for i in range(60)]),
                             (((0.,0.),(0.,3.)),[(0.,.001*i) for i in range(60)])):
            p=self.tracker(route)
            self.assertTrue(any(self.move(p,points)))
            self.assertLess(p.routes[0]['credit'],.1)

    def test_cropped_or_republished_route_keeps_credit_and_motion_window(self):
        p=self.tracker();self.move(p,[(.002*i,0.) for i in range(60)])
        credit=p.routes[0]['credit'];history=len(p.motion.history)
        self.assertTrue(p.set_route(((.05,0.),(3.,0.)),2))
        self.assertEqual(p.route,0);self.assertEqual(p.routes[0]['credit'],credit)
        self.assertEqual(len(p.motion.history),history)

    def test_shortcut_is_new_route_and_old_route_keeps_its_history(self):
        route=((0.,0.),(0.,1.),(1.,1.))
        p=self.tracker(route);p.routes[0]['credit']=.5
        p.set_route(((0.,0.),(1.,1.)),2)
        self.assertEqual(p.route,1)
        p.set_route(route,3)
        self.assertEqual(p.route,0);self.assertEqual(p.routes[0]['credit'],.5)

    def test_return_along_same_route_cannot_earn_credit_twice(self):
        p=self.tracker();self.move(p,[(.002*i,0.) for i in range(100)])
        high=p.routes[0]['credit']
        self.move(p,[(.198-.002*i,0.) for i in range(100)],6_000_000_000)
        credited=self.move(p,[(.001*i,0.) for i in range(100)],11_000_000_000)
        self.assertFalse(any(credited));self.assertEqual(p.routes[0]['credit'],high)

    def test_repeated_stops_without_new_progress_stop_but_new_progress_allows_detour(self):
        p=self.tracker()
        for _ in range(2):
            p.last_position=(.5,0.);p.stop_travel=1.;p.stop(False);p.stop(True)
            self.assertIsNone(p.failure)
        p.last_position=(.5,0.);p.stop_travel=1.;p.stop(False);p.stop(True)
        self.assertEqual(p.failure,'REPEATED_ATTEMPT')
        p=self.tracker()
        for i in range(10):
            p.credit_count=i;p.last_position=(.5,0.);p.stop_travel=1.;p.stop(False);p.stop(True)
        self.assertIsNone(p.failure)

    def test_repeated_wait_notifications_without_motion_are_not_attempts(self):
        p=self.tracker();p.last_position=(.5,0.)
        for _ in range(10):p.stop(False);p.stop(True)
        self.assertIsNone(p.failure);self.assertEqual(len(p.checkpoints),1)

    def test_unknown_bound_authority_loss_and_invalid_projection_never_credit(self):
        p=RouteProgress();p.set_route(((0,0),(3,0)),1)
        self.assertFalse(any(self.move(p,[(.002*i,0.) for i in range(100)])))
        p=self.tracker()
        self.assertFalse(any(self.move(p,[(.002*i,0.) for i in range(100)],allowed=False)))
        self.assertFalse(any(self.move(p,[(.002*i,1.) for i in range(100)],6_000_000_000)))
        self.assertIsNone(project(((0.,0.),(1.,1.),(0.,1.),(1.,0.)),(.5,.5)))

    def test_history_is_bounded_and_never_evicted_to_renew_attempts(self):
        p=self.tracker(route_capacity=2)
        p.set_route(((0.,0.),(0.,3.)),2)
        p.set_route(((0.,0.),(-3.,0.)),3)
        self.assertEqual(p.failure,'PROGRESS_HISTORY_EXHAUSTED')
        self.assertEqual(len(p.routes),2)
        p.reset();self.assertIsNone(p.failure);self.assertEqual(p.routes,[])

    def test_invalid_geometry_does_not_authorize_old_route(self):
        p=self.tracker()
        self.assertFalse(p.set_route(((0.,0.),(float('nan'),0.)),2))
        self.assertIsNone(p.route)

    def test_hairpin_projection_switch_is_not_metres_of_progress(self):
        p=self.tracker(((0.,0.),(1.,0.),(1.,.04),(0.,.04)))
        self.move(p,[(.2+i*.001,.019) for i in range(40)])
        self.assertFalse(p.observe(.24,.021,3_000_000_000,0.,True))
        self.assertEqual(p.reason,'ROUTE_COORDINATE_JUMP')


if __name__=='__main__':unittest.main()
