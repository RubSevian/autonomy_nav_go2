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
            if d.get('physical') is True and d.get('base_locked') is True:self.base_locked=True
        except (ValueError, TypeError, KeyError):
            self.ready = False

    def valid(self, now_ns, nav_session):
        return (self.gid is not None and self.ready and self.nav == nav_session
                and 0 <= (now_ns-self.stamp)/1e9 <= self.ttl)
