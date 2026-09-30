"""Unit and integration tests for M8A Benchmark Infrastructure."""

import json
import os
import tempfile
import unittest

from amr_fleet_core.benchmark_manager import (
    ExperimentMetadata,
    get_git_commit,
    StandardMetrics,
    StatisticalAggregator,
    TerminationReason,
)
from amr_fleet_core.task_generator import (
    DEFAULT_OBSTACLE_BOUNDS,
    TaskGenerator,
    TaskGeneratorConfig,
)
from amr_fleet_core.workload import WorkloadManager


def get_workload_path(filename: str) -> str:
    """Resolve workload file path relative to repository root."""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(test_dir)))
    candidate = os.path.join(repo_root, 'config', 'workloads', filename)
    if os.path.isfile(candidate):
        return candidate
    # Fallback to local cwd
    local_cand = os.path.join('config', 'workloads', filename)
    if os.path.isfile(local_cand):
        return local_cand
    return candidate


class TestM8AWorkloadScaling(unittest.TestCase):
    """Test deterministic workload generation and scaling across sizes."""

    def test_workload_file_loading_and_counts(self):
        """Verify all 4 deterministic workload files load exact task counts."""
        sizes = [15, 30, 50, 100]
        for sz in sizes:
            path = get_workload_path(f'workload_{sz}_tasks.yaml')
            self.assertTrue(os.path.isfile(path), f'Missing: {path}')
            tasks = WorkloadManager.load_from_yaml(path)
            self.assertEqual(
                len(tasks), sz, f'Expected {sz}, got {len(tasks)}'
            )

            ids = [t.task_id for t in tasks]
            self.assertEqual(len(ids), len(set(ids)), 'IDs must be unique')

    def test_obstacle_avoidance(self):
        """Verify generated points strictly avoid rack obstacles."""
        sizes = [15, 30, 50, 100]
        for sz in sizes:
            path = get_workload_path(f'workload_{sz}_tasks.yaml')
            tasks = WorkloadManager.load_from_yaml(path)
            for t in tasks:
                px, py = t.pickup
                dx, dy = t.dropoff

                for ox_min, ox_max, oy_min, oy_max in DEFAULT_OBSTACLE_BOUNDS:
                    in_p = (ox_min <= px <= ox_max and oy_min <= py <= oy_max)
                    in_d = (ox_min <= dx <= ox_max and oy_min <= dy <= oy_max)
                    self.assertFalse(in_p, f'Pickup {px, py} inside obstacle')
                    self.assertFalse(in_d, f'Dropoff {dx, dy} inside obstacle')

    def test_deterministic_seed_reproducibility(self):
        """Verify identical seed produces identical tasks."""
        cfg_a = TaskGeneratorConfig(task_count=20, seed=42)
        cfg_b = TaskGeneratorConfig(task_count=20, seed=42)
        cfg_c = TaskGeneratorConfig(task_count=20, seed=99)

        gen_a = TaskGenerator(cfg_a).generate_workload()
        gen_b = TaskGenerator(cfg_b).generate_workload()
        gen_c = TaskGenerator(cfg_c).generate_workload()

        self.assertEqual(len(gen_a), len(gen_b))
        for ta, tb in zip(gen_a, gen_b):
            self.assertEqual(ta.task_id, tb.task_id)
            self.assertEqual(ta.pickup, tb.pickup)
            self.assertEqual(ta.dropoff, tb.dropoff)
            self.assertEqual(ta.priority, tb.priority)

        pickups_a = [t.pickup for t in gen_a]
        pickups_c = [t.pickup for t in gen_c]
        self.assertNotEqual(pickups_a, pickups_c)


class TestM8ABenchmarkMetadataAndSchema(unittest.TestCase):
    """Test experiment metadata, schema versioning, and termination reasons."""

    def test_termination_reasons(self):
        """Test conversion and discrete states of TerminationReason."""
        self.assertEqual(
            TerminationReason.from_str('ALL_TASKS_COMPLETED'),
            TerminationReason.ALL_TASKS_COMPLETED,
        )
        self.assertEqual(
            TerminationReason.from_str('HORIZON_REACHED'),
            TerminationReason.HORIZON_REACHED,
        )
        self.assertEqual(
            TerminationReason.from_str('SAFETY_ABORT'),
            TerminationReason.SAFETY_ABORT,
        )
        self.assertEqual(
            TerminationReason.from_str('invalid_reason'),
            TerminationReason.EXPERIMENT_ERROR,
        )

    def test_metadata_fields(self):
        """Verify experiment metadata completeness."""
        git_hash = get_git_commit()
        self.assertIsInstance(git_hash, str)
        self.assertTrue(len(git_hash) > 0)

        meta = ExperimentMetadata(
            experiment_id='exp_test_001',
            timestamp_utc='2026-09-13T12:00:00Z',
            timestamp_unix=1789295000.0,
            git_commit=git_hash,
            software_version='m8a.v1',
            fleet_size=5,
            workload_id='workload_15_tasks',
            workload_size=15,
            trial_id=0,
            seed=42,
            mission_horizon_sec=60.0,
            actual_duration_sec=60.01,
            communication_profile='NORMAL',
            map_id='warehouse_grid_small',
            termination_reason=TerminationReason.HORIZON_REACHED.value,
        )
        self.assertEqual(meta.software_version, 'm8a.v1')
        self.assertEqual(meta.termination_reason, 'HORIZON_REACHED')

    def test_task_accounting_invariance(self):
        """Verify task accounting invariant: completed + remaining == gen."""
        m = StandardMetrics(
            generated_tasks=30,
            assigned_tasks=30,
            completed_tasks=4,
            failed_tasks=0,
            remaining_tasks=26,
            completion_rate_pct=round(4 / 30 * 100.0, 1),
            throughput_tasks_per_min=2.0,
            makespan_sec=115.4,
        )
        self.assertEqual(
            m.completed_tasks + m.remaining_tasks, m.generated_tasks
        )
        self.assertEqual(m.failed_tasks, 0)


class TestM8AStatisticalAggregation(unittest.TestCase):
    """Test aggregation, confidence intervals, and null handling."""

    def test_summarize_series_math(self):
        """Test mean, median, std, min, max on a known series."""
        vals = [10.0, 20.0, 30.0, 40.0, 50.0]
        summary = StatisticalAggregator.summarize_series(vals)
        self.assertIsNotNone(summary)
        self.assertEqual(summary['n'], 5)
        self.assertAlmostEqual(summary['mean'], 30.0, places=3)
        self.assertAlmostEqual(summary['median'], 30.0, places=3)
        self.assertAlmostEqual(summary['min_val'], 10.0, places=3)
        self.assertAlmostEqual(summary['max_val'], 50.0, places=3)
        self.assertAlmostEqual(summary['std'], 15.8114, places=3)
        self.assertIsNotNone(summary['p95'])
        self.assertIsNotNone(summary['ci_95_lower'])
        self.assertIsNotNone(summary['ci_95_upper'])

    def test_confidence_interval_suppressed_for_single_observation(self):
        """Confidence interval must be null for n < 3."""
        summary_1 = StatisticalAggregator.summarize_series([42.0])
        self.assertIsNone(summary_1['ci_95_lower'])
        self.assertIsNone(summary_1['ci_95_upper'])
        self.assertEqual(summary_1['std'], 0.0)

        summary_2 = StatisticalAggregator.summarize_series([10.0, 20.0])
        self.assertIsNone(summary_2['ci_95_lower'])
        self.assertIsNone(summary_2['ci_95_upper'])

    def test_missing_and_nan_handling(self):
        """Empty series or all None must return None without raising error."""
        self.assertIsNone(StatisticalAggregator.summarize_series([]))
        self.assertIsNone(
            StatisticalAggregator.summarize_series([None, float('nan')])
        )

    def test_aggregate_trials_multi_sample(self):
        """Test aggregation across multiple raw trial dictionaries."""
        trials = [
            {
                'metadata': {
                    'experiment_id': f'exp_test_{i}',
                    'trial_id': i,
                    'seed': 42 + i,
                    'workload_size': 15,
                    'communication_profile': 'NORMAL',
                    'mission_horizon_sec': 60.0,
                    'actual_duration_sec': 60.0,
                    'termination_reason': 'HORIZON_REACHED',
                },
                'metrics': {
                    'completed_tasks': 1,
                    'completion_rate_pct': 6.7,
                    'throughput_tasks_per_min': 1.0,
                    'makespan_sec': 50.0 + i * 2.0,
                    'replan_count': 70 + i * 5,
                    'planning_latency_mean_ms': 1.2 + i * 0.1,
                    'planning_latency_p95_ms': 2.5 + i * 0.2,
                    'minimum_inter_robot_distance_m': 1.5 - i * 0.05,
                    'collision_contact_events': 0,
                },
            }
            for i in range(5)
        ]

        agg = StatisticalAggregator.aggregate_trials('bench_test_w15', trials)
        self.assertEqual(agg['schema_version'], 'm8a.v1')
        self.assertEqual(agg['trial_count'], 5)
        self.assertEqual(agg['workload_size'], 15)

        ms_summary = agg['aggregated_metrics']['makespan_sec']
        self.assertEqual(ms_summary['n'], 5)
        self.assertAlmostEqual(ms_summary['mean'], 54.0, places=2)
        self.assertEqual(len(agg['individual_trials']), 5)

        with tempfile.TemporaryDirectory() as tmpdir:
            raw_path = os.path.join(tmpdir, 'raw_trial.json')
            with open(raw_path, 'w') as f:
                json.dump(trials[0], f)

            agg_path = os.path.join(tmpdir, 'agg_summary.json')
            with open(agg_path, 'w') as f:
                json.dump(agg, f)

            self.assertTrue(os.path.isfile(raw_path))
            self.assertTrue(os.path.isfile(agg_path))


if __name__ == '__main__':
    unittest.main()

