"""Single-source mission health lease; cannot renew a stale observation."""
import json
import math

class MissionHealth:
    def __init__(self, ttl):
        if not math.isfinite(ttl) or ttl <= 0:
            raise ValueError('mission health TTL must be positive')
        self.ttl, self.gid, self.session = ttl, None, ''
        self.stamp, self.ready, self.nav = 0, False, ''
        self.owner_since = 0
        self.base_locked = False  # latched for this NAV process; stale input cannot unlock
        self.return_request_ns = 0
        self.return_target = None
        self.lock_owner = None

    def graph(self, gids, now_ns):
        gid = bytes(gids[0]) if len(gids) == 1 else None
        if gid != self.gid:
            self.gid, self.session, self.stamp, self.ready = gid, '', 0, False
            self.owner_since = now_ns

    def observe(self, text, gid, now_ns, nav_session):
        try:
            d = json.loads(text)
            stamp, session = d['observed_ns'], d['session_id']
            if (self.gid is None or bytes(gid) != self.gid or type(stamp) is not int
                    or not isinstance(session, str) or not session
                    or stamp <= max(self.stamp, self.owner_since)
                    or not 0 <= (now_ns-stamp)/1e9 <= self.ttl
                    or d['navigation_session_id'] != nav_session):
                self.ready = False
                return
            if self.session and self.session != session:
                self.ready = False  # a publisher cannot switch session in place
                return
            self.session, self.stamp, self.nav = session, stamp, nav_session
            self.ready = d.get('ready') is True and d.get('physical') is True
            phase=d.get('phase')
            # One explicit post-HOME return lease from the SAME mission owner.
            # Ordinary base_locked=false, stale health and a new publisher cannot unlock.
            request = d.get('return_request_ns', 0)
            home = d.get('home_completed_ns')
            target = d.get('return_target')
            if (self.base_locked and not self.return_request_ns and self.ready
                    and self.lock_owner == (self.gid, self.session)
                    and phase == 'RETURN_NAV_PREPARE' and d.get('arm_home_ready') is True
                    and type(request) is int and type(home) is int and 0 < home < request <= stamp
                    and 0 <= (now_ns-request)/1e9 <= 5.
                    and isinstance(target, list) and len(target) == 3
                    and all(type(v) in (float, int) and math.isfinite(v) for v in target)):
                self.return_request_ns, self.return_target = request, tuple(target)
                self.base_locked = False
            if self.return_request_ns and (request != self.return_request_ns or phase not in
                    ('RETURN_NAV_PREPARE','RETURN_NAVIGATING','RETURN_NAV_SETTLE')):
                self.base_locked = True
            # A recoverable WAIT_GOAL status must never establish the arm lock.
            # Existing locks remain latched, including while stale input arrives.
            if d.get('physical') is True and d.get('base_locked') is True and phase not in ('IDLE','WAIT_GOAL','NAVIGATING','WAIT_PATH_RECOVERY'):
                if self.lock_owner is None:
                    self.lock_owner = (self.gid, self.session)
                self.base_locked=True
        except (ValueError, TypeError, KeyError):
            self.ready = False

    def valid(self, now_ns, nav_session):
        return (self.gid is not None and self.ready and self.nav == nav_session
                and 0 <= (now_ns-self.stamp)/1e9 <= self.ttl)
