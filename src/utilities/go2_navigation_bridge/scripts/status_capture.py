"""Bounded latest-status sampling; semantic changes bypass the file rate limit."""
import json
import math

_EVENT_KEYS = frozenset(('state', 'phase', 'reason', 'mode', 'status', 'code',
    'fault', 'fault_latched', 'error', 'level', 'name', 'message', 'ready', 'enabled',
    'base_locked', 'selected_fresh', 'using_previous_pair',
    'session_id', 'owner_session_id', 'controller_session_id', 'navigation_session_id',
    'goal_revision', 'revision', 'cancel_latched', 'command_enabled',
    'full_attempt', 'grasp_assessment', 'blockers', 'warnings', 'phase_warnings'))


def event_signature(value):
    """Ignore ages, poses and counters; preserve nested goal/TF/error transitions."""
    result = []
    def stable(item):
        if isinstance(item, dict):return tuple(sorted((k,stable(v)) for k,v in item.items()))
        if isinstance(item, list):return tuple(stable(v) for v in item)
        if isinstance(item, float) and not math.isfinite(item):return ('nonfinite',repr(item))
        return item
    def visit(item, path):
        if isinstance(item, dict):
            for key, child in item.items():
                location = (*path, key)
                if (key in _EVENT_KEYS or key.endswith(('_reason', '_error', '_ready'))):
                    result.append((location, stable(child)))
                elif isinstance(child, (dict, list)):
                    visit(child, location)
        elif isinstance(item, list):
            for index, child in enumerate(item):
                if isinstance(child, dict):visit(child, (*path, index))
    visit(value, ())
    return frozenset(result)


class StatusCapture:
    def __init__(self, output, period_s):
        if not math.isfinite(period_s) or period_s <= 0:
            raise ValueError('status period must be positive')
        self.output = output
        self.period_ns = round(period_s*1e9)
        self.signatures = {}
        self.last_write = {}
        self.pending = {}  # At most one latest row per fixed subscribed topic.
        self.received = self.written = self.event_rows = self.replaced = 0

    def record(self, row, signature):
        topic = row['topic'];now = row['mono_ns'];self.received += 1
        changed = topic not in self.signatures or self.signatures[topic] != signature
        self.signatures[topic] = signature
        if changed or now-self.last_write[topic] >= self.period_ns:
            self.pending.pop(topic, None)
            self._write(row, 'event' if changed else 'periodic', now)
        else:
            if topic in self.pending:self.replaced += 1
            self.pending[topic] = row

    def _write(self, row, kind, now):
        self.output.write(json.dumps(dict(row, recording_kind=kind))+'\n')
        self.last_write[row['topic']] = now
        self.written += 1
        self.event_rows += kind == 'event'

    def flush(self, now, force=False):
        for topic, row in list(self.pending.items()):
            if force or now-self.last_write[topic] >= self.period_ns:
                del self.pending[topic]
                self._write(row, 'final' if force else 'periodic', now)

    def summary(self):
        return dict(period_s=self.period_ns/1e9, received=self.received, written=self.written,
                    event_rows=self.event_rows, replaced_periodic_rows=self.replaced,
                    pending=len(self.pending))
