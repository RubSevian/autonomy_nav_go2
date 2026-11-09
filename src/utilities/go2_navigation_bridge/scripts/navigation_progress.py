"""Bounded route progress and repeated-attempt history, independent of ROS.

Planner revisions never count as travel. Cached canonical polylines keep their
credit when an identical/cropped route returns. Ambiguous projections fail
closed. Repetition is an operational limit, not proof of unreachability.
"""
import math
from dataclasses import asdict
from navigation_motion import MotionEvidence


def distance(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1])


def project(points,p):
    candidates=[];offset=0.
    for i,(a,b) in enumerate(zip(points,points[1:])):
        length=distance(a,b)
        if length<=1e-8:continue
        ux,uy=(b[0]-a[0])/length,(b[1]-a[1])/length
        along=max(0.,min(length,(p[0]-a[0])*ux+(p[1]-a[1])*uy))
        error=distance(p,(a[0]+along*ux,a[1]+along*uy))
        candidates.append((error,offset+along,(ux,uy),i))
        offset+=length
    if not candidates:return None
    candidates.sort(key=lambda c:c[0])
    best=candidates[0]
    # Crossing/self-overlapping routes have multiple plausible arc coordinates.
    if any(abs(c[0]-best[0])<1e-6 and abs(c[1]-best[1])>1e-4 for c in candidates[1:]):
        return None
    return best


class RouteProgress:
    def __init__(self, *, noise_bound_m=None, window_s=1., endpoint_s=.15,
                 route_tolerance_m=.08, repeat_radius_m=.20,
                 repeat_excursion_m=.6, repeat_limit=3, route_capacity=32,
                 history_capacity=256):
        for v in (route_tolerance_m,repeat_radius_m,repeat_excursion_m):
            if type(v) not in (int,float) or not math.isfinite(v) or v<=0:raise ValueError('Invalid route limit')
        if repeat_excursion_m<=2*repeat_radius_m:raise ValueError('Excursion must leave the repeated region')
        for v in (repeat_limit,route_capacity,history_capacity):
            if type(v) is not int or v<2:raise ValueError('Invalid history limit')
        self.motion=MotionEvidence(noise_bound_m=noise_bound_m,window_s=window_s,endpoint_s=endpoint_s)
        self.route_tolerance=route_tolerance_m
        self.repeat_radius,self.excursion=repeat_radius_m,repeat_excursion_m
        self.repeat_limit,self.route_capacity,self.history_capacity=repeat_limit,route_capacity,history_capacity
        self.reset()

    def reset(self):
        self.routes=[];self.route=None;self.route_stamp=0
        self.motion.reset();self.reason='NO_ROUTE';self.failure=None
        self.last_position=None;self.last_stamp=0;self.last_route_sample=None
        self.last_allowed=False;self.travel=0.;self.stop_travel=0.
        self.checkpoints=[];self.credit_count=0;self.repeat_count=0
        self.waiting=False;self.last_stop=None;self.route_s=None

    def invalidate_route(self,reason='NO_ROUTE'):
        self.route=None;self.route_stamp=0;self.last_route_sample=None
        self.motion.reset();self.reason=reason

    def _compatible(self,old,new):
        # Every new vertex AND segment midpoint must lie on the stored route
        # in forward order. This prevents a shortcut from matching a detour.
        if distance(old[-1],new[-1])>self.route_tolerance:return False
        probes=[]
        for a,b in zip(new,new[1:]):probes.extend((a,((a[0]+b[0])/2,(a[1]+b[1])/2)))
        probes.append(new[-1]);last=-math.inf
        for p in probes:
            q=project(old,p)
            if q is None or q[0]>self.route_tolerance or q[1]<last-1e-6:return False
            last=q[1]
        return True

    def set_route(self,points,stamp):
        if type(stamp) is not int or stamp<=self.route_stamp:return False
        try:
            if not 2<=len(points)<=256:raise ValueError()
            route=tuple(tuple(float(v) for v in p) for p in points)
            if any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in route):raise ValueError()
            if sum(distance(a,b) for a,b in zip(route,route[1:]))<=1e-6:raise ValueError()
        except (ValueError,TypeError,OverflowError):
            self.invalidate_route('INVALID_ROUTE');return False
        index=next((i for i,r in enumerate(self.routes) if self._compatible(r['points'],route)),None)
        if index is None:
            if len(self.routes)>=self.route_capacity:
                self.failure='PROGRESS_HISTORY_EXHAUSTED';return False
            index=len(self.routes);self.routes.append(dict(points=route,credit=None))
        if index!=self.route:
            self.motion.reset();self.last_route_sample=None;self.last_allowed=False
        self.route=index;self.route_stamp=stamp
        self.reason='ROUTE_AVAILABLE'
        return True

    def _checkpoint(self,p,tangent,kind):
        if self.route is None:return
        for entry in self.checkpoints:
            if (entry['route']==self.route and entry['kind']==kind and
                    distance(entry['p'],p)<=self.repeat_radius and
                    sum(a*b for a,b in zip(entry['direction'],tangent))>=.8):
                if entry['credit']!=self.credit_count:
                    entry['count']=1;entry['credit']=self.credit_count
                else:entry['count']+=1
                self.repeat_count=entry['count']
                if entry['count']>=self.repeat_limit:self.failure='REPEATED_ATTEMPT'
                return
        if len(self.checkpoints)>=self.history_capacity:
            self.failure='PROGRESS_HISTORY_EXHAUSTED';return
        self.checkpoints.append(dict(route=self.route,kind=kind,p=p,direction=tangent,
                                     credit=self.credit_count,count=1))

    def observe(self,x,y,stamp,age,allowed,direction=None):
        p=(x,y)
        result=self.motion.observe(x,y,stamp,age)
        self.reason=result.reason
        if result.reason in ('INVALID_SAMPLE','STALE_SAMPLE','TIME_OR_POSE_REVISION','POSE_JUMP','HISTORY_CAPACITY'):
            self.last_position=None;self.last_route_sample=None;self.last_allowed=False
            self.travel=0.;self.stop_travel=0.
            return False
        if result.reason=='DUPLICATE_SAMPLE':return False
        if self.route is None:
            self.reason='NO_ROUTE';return False
        r=self.routes[self.route];q=project(r['points'],p)
        if q is None or q[0]>self.route_tolerance:
            self.reason='OFF_ROUTE_OR_AMBIGUOUS';self.last_route_sample=None
            self.motion.reset();self.last_allowed=False;return False
        _,s,tangent,_=q;self.route_s=s
        if direction is not None and sum(a*b for a,b in zip(direction,tangent))<=0:
            allowed=False
        previous=self.last_route_sample
        self.last_route_sample=(stamp,s)
        if not allowed:
            self.motion.reset('MOTION_NOT_AUTHORIZED');self.reason='MOTION_NOT_AUTHORIZED'
            self.last_allowed=False;self.last_position=None
            # Motion observed while paused cannot earn credit on resumption.
            r['credit']=s if r['credit'] is None else max(r['credit'],s)
            return False
        if not self.last_allowed or previous is None or stamp-previous[0]>250_000_000:
            self.motion.reset();self.motion.observe(x,y,stamp,age)
            r['credit']=s if r['credit'] is None else max(r['credit'],s)
            self.last_allowed=True;self.last_position=p;return False
        if self.last_position is not None and abs(s-previous[1])>2*distance(p,self.last_position)+.02:
            # Nearby arms of a hairpin must not turn millimetres of XY jitter
            # into metres of apparent progress along the route.
            self.reason='ROUTE_COORDINATE_JUMP';self.motion.reset()
            self.last_route_sample=None;self.last_allowed=False;return False
        self.last_allowed=True
        if result.confirmed and self.last_position is not None:
            step=distance(p,self.last_position)
            self.travel+=step;self.stop_travel+=step
        self.last_position=p
        bound=self.motion.noise_bound_m
        credit=(result.confirmed and bound is not None and s-r['credit']>bound and s>previous[1])
        if credit:
            r['credit']=s;self.credit_count+=1;self.reason='NEW_ROUTE_PROGRESS'
        elif result.confirmed:self.reason='NO_NEW_ROUTE_PROGRESS'
        if result.confirmed and self.travel>=self.excursion:
            self._checkpoint(p,tangent,'travel');self.travel=0.
        return credit

    def stop(self,waiting):
        if waiting and not self.waiting and self.route is not None and self.last_position is not None:
            q=project(self.routes[self.route]['points'],self.last_position)
            if q is not None and (self.last_stop is None or self.stop_travel>=(self.motion.noise_bound_m or math.inf)):
                self._checkpoint(self.last_position,q[2],'stop')
                self.last_stop=self.last_position;self.stop_travel=0.
        self.waiting=waiting

    def snapshot(self):
        return dict(reason=self.reason,failure=self.failure,motion=asdict(self.motion.result),
                    calibrated=self.motion.noise_bound_m is not None,route_index=self.route,
                    route_s_m=self.route_s,route_count=len(self.routes),
                    progress_count=self.credit_count,repeat_count=self.repeat_count,
                    repeat_limit=self.repeat_limit,history_entries=len(self.checkpoints))
