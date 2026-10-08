"""Pure benchmark regression tests: preserve failure timing and honest missing-clock statistics."""
import unittest

from benchmark import failure_periods, percentiles, process_usage


class BenchmarkStatisticsTest(unittest.TestCase):
    """Exercise closed/open failure runs and actual Linux process counters without starting ROS."""

    def test_failure_runs_include_all_failure_statuses_and_open_tail(self):
        results = [{'received_monotonic_s': t, 'status': status} for t, status in
                   [(10.1, 7), (10.2, 10), (10.7, 2), (11.0, 3), (11.2, 8)]]
        periods = failure_periods(results, 10, 12)
        self.assertEqual(len(periods), 2)
        self.assertAlmostEqual(periods[0]['duration_s'], 0.6)
        self.assertFalse(periods[0]['open_at_end'])
        self.assertEqual(periods[1]['duration_s'], 1)
        self.assertTrue(periods[1]['open_at_end'])

    def test_missing_clock_values_are_not_fabricated_zeros(self):
        self.assertEqual(percentiles([None, float('nan')]), [])
        self.assertEqual(percentiles([None, 80, 300])[-1], 300)

    def test_actual_process_counters_are_available(self):
        import os
        usage = process_usage(os.getpid())
        self.assertGreater(usage['rss_kib'], 0)
        self.assertGreaterEqual(usage['cpu_s'], 0)


if __name__ == '__main__':
    unittest.main()
