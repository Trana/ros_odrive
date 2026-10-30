"""Test-harness guards only. No ROS, CAN sends, or production configuration.

Limits are bounded supported-test criteria, not a certified safety mechanism.
Observation timestamps must describe kernel reception, not socket dequeue time.
"""
from collections import deque


class DiagnosticAbort(RuntimeError):
    pass


def check_idle_heartbeats(nodes, states, now, *, allow_expired_watchdog=False):
    """Only the known idle watchdog flag may precede explicit rearming.

    Does not clear faults/feed watchdogs. Active checks must still require zero.
    """
    if set(states) != set(nodes):
        raise DiagnosticAbort('missing idle heartbeat')
    permitted = (0, 0x01000000) if allow_expired_watchdog else (0,)
    for node, (stamp, state, error) in states.items():
        if not 0 <= now - stamp < 2.5 or state != 1 or error not in permitted:
            raise DiagnosticAbort('invalid idle heartbeat: ' + str(node))


class DiagnosticGuard:
    def __init__(self, nodes, baseline, *, max_overruns=0, burst_count=2,
                 burst_window_s=1.0, feedback_age_s=0.05):
        if max_overruns not in (0, 1, 2) or burst_count != 2:
            raise ValueError('only strict or bounded 1/2-overrun diagnostics allowed')
        if burst_window_s != 1.0 or feedback_age_s != 0.05:
            raise ValueError('diagnostic time limits are fixed; review changes separately')
        self.nodes = set(nodes)
        if not self.nodes:
            raise ValueError('nodes required')
        self.baseline = dict(baseline)
        self.previous = dict(baseline)
        self.max_overruns = max_overruns
        self.burst_count = burst_count
        self.burst_window_s = burst_window_s
        self.feedback_age_s = feedback_age_s
        self.events = deque()
        self.feedback = {}
        self.last_check = None
        self.reason = None

    def fail(self, reason):
        self.reason = reason
        raise DiagnosticAbort(reason)

    def observe(self, node, stamp):
        if node in self.nodes:
            previous = self.feedback.get(node)
            if previous is not None and stamp < previous:
                self.fail('feedback timestamp regression')
            self.feedback[node] = stamp

    def check(self, now, counters):
        if self.reason:
            raise DiagnosticAbort(self.reason)
        if self.last_check is not None and now < self.last_check:
            self.fail('monitor clock regression')
        # Failing to run the observer promptly is itself a diagnostic failure.
        if self.last_check is not None and now - self.last_check >= self.feedback_age_s:
            self.fail('monitor scheduling gap')
        for key in self.baseline:
            if key not in counters or counters[key] < self.previous[key]:
                self.fail('missing/reset counter: ' + key)
        delta = {k: counters[k] - self.previous[k] for k in self.baseline}
        for key in ('tx_errors', 'rx_dropped', 'tx_dropped'):
            if delta[key]:
                self.fail('non-overrun error/drop: ' + key)
        # Never excuse an unexplained RX error as a permitted overrun.
        if delta['rx_errors'] != delta['rx_over_errors']:
            self.fail('RX error/overrun mismatch')
        total = counters['rx_over_errors'] - self.baseline['rx_over_errors']
        if total > self.max_overruns:
            self.fail('total overrun budget exceeded')
        # Attribute deltas to the whole sampled interval conservatively. Retain
        # one extra sample interval at the rolling window boundary.
        while self.events and self.events[0][0] < now - self.burst_window_s - self.feedback_age_s:
            self.events.popleft()
        if delta['rx_over_errors']:
            self.events.append((now, delta['rx_over_errors']))
        if sum(count for _, count in self.events) >= self.burst_count:
            self.fail('overrun burst limit reached')
        for node in self.nodes:
            stamp = self.feedback.get(node)
            if stamp is None or not 0 <= now - stamp < self.feedback_age_s:
                self.fail('missing/stale/future encoder feedback: ' + str(node))
        self.previous = dict(counters)
        self.last_check = now
        return total
