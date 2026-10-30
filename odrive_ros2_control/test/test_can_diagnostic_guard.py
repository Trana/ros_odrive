import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from can_diagnostic_guard import DiagnosticAbort, DiagnosticGuard, check_idle_heartbeats


def counters(n=0):
    return dict(rx_errors=n, rx_over_errors=n, rx_dropped=0, tx_errors=0,
                tx_dropped=0, rx_packets=100, tx_packets=0)


class GuardTests(unittest.TestCase):
    def test_idle_watchdog_is_explicit_and_exact(self):
        for error in [0, 0x01000000]:
            check_idle_heartbeats([11], {11: (1, 1, error)}, 1.1, allow_expired_watchdog=True)
        for error, state in [(0x01000000, 1), (0x01000001, 1), (1, 1), (0, 8)]:
            with self.assertRaises(DiagnosticAbort):
                check_idle_heartbeats([11], {11: (1, state, error)}, 1.1)
        with self.assertRaises(DiagnosticAbort):
            check_idle_heartbeats([11], {11: (1, 1, 0x01000001)}, 1.1, allow_expired_watchdog=True)

    def test_idle_missing_stale_and_future(self):
        for states in [{}, {11: (0, 1, 0)}, {11: (4, 1, 0)}]:
            with self.assertRaises(DiagnosticAbort):
                check_idle_heartbeats([11], states, 3, allow_expired_watchdog=True)

    def guard(self, allowance=2):
        return DiagnosticGuard([11, 12], counters(), max_overruns=allowance)

    def tick(self, guard, t, n=0):
        for node in [11, 12]:
            guard.observe(node, t)
        return guard.check(t, counters(n))

    def test_strict_default(self):
        with self.assertRaisesRegex(DiagnosticAbort, 'total'):
            self.tick(self.guard(0), 0, 1)

    def test_isolated_overruns_then_budget(self):
        g = self.guard()
        for step in range(301):
            t = step / 100
            self.tick(g, t, int(t >= .1) + int(t >= 1.5))
        with self.assertRaisesRegex(DiagnosticAbort, 'total'):
            self.tick(g, 3.01, 3)

    def test_burst(self):
        g = self.guard()
        self.tick(g, 0, 1)
        with self.assertRaisesRegex(DiagnosticAbort, 'burst'):
            self.tick(g, .01, 2)

    def test_two_in_one_sample(self):
        with self.assertRaisesRegex(DiagnosticAbort, 'burst'):
            self.tick(self.guard(), 0, 2)

    def test_stale_axis_and_latching(self):
        g = self.guard()
        self.tick(g, 0)
        g.observe(11, .04)
        g.check(.04, counters())
        g.observe(11, .05)
        with self.assertRaisesRegex(DiagnosticAbort, 'encoder'):
            g.check(.05, counters())
        with self.assertRaises(DiagnosticAbort):
            self.tick(g, .06)

    def test_observer_stall(self):
        g = self.guard()
        self.tick(g, 0)
        with self.assertRaisesRegex(DiagnosticAbort, 'scheduling'):
            self.tick(g, .05)

    def test_missing_and_future(self):
        for stamp in [None, 1]:
            g = self.guard()
            if stamp is not None:
                g.observe(11, stamp)
            with self.assertRaises(DiagnosticAbort):
                g.check(0, counters())

    def test_other_errors_never_allowed(self):
        for key in ['tx_errors', 'rx_dropped', 'tx_dropped', 'rx_errors']:
            g = self.guard()
            self.tick(g, 0)
            c = counters()
            c[key] += 1
            with self.assertRaises(DiagnosticAbort):
                g.check(.01, c)

    def test_reset_missing_clock(self):
        for kind in ['reset', 'missing', 'clock']:
            g = self.guard()
            self.tick(g, .1)
            c = counters()
            if kind == 'reset': c['rx_packets'] = 0
            if kind == 'missing': del c['rx_over_errors']
            with self.assertRaises(DiagnosticAbort):
                g.check(0 if kind == 'clock' else .11, c)

    def test_timestamp_regression(self):
        g = self.guard()
        g.observe(11, 1)
        with self.assertRaises(DiagnosticAbort):
            g.observe(11, .9)

    def test_invalid_configuration(self):
        for value in [-1, 3, 100]:
            with self.assertRaises(ValueError):
                self.guard(value)


if __name__ == '__main__':
    unittest.main()
