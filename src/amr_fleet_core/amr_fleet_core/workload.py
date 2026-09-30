"""Workload configuration loader and manager for reproducible task sets."""

import os
from typing import List

from amr_fleet_core.task_generator import TaskGenerator, TaskGeneratorConfig
from amr_fleet_core.task_model import Task, TaskPriority
import yaml


class WorkloadManager:
    """Loads, validates, and manages task workloads from YAML definitions."""

    @staticmethod
    def load_from_yaml(yaml_path: str) -> List[Task]:
        """
        Load tasks from a YAML workload specification.

        Supports both algorithmic generation from parameters and explicit task lists.
        """
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f'Workload file not found: {yaml_path}')

        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        workload_data = data.get('workload', data)

        # Mode A: Explicit task definitions
        if 'explicit_tasks' in workload_data:
            tasks: List[Task] = []
            for item in workload_data['explicit_tasks']:
                d_val = item.get('deadline')
                deadline = float(d_val) if d_val is not None else None
                task = Task(
                    task_id=str(item['task_id']),
                    pickup=(float(item['pickup'][0]), float(item['pickup'][1])),
                    dropoff=(float(item['dropoff'][0]), float(item['dropoff'][1])),
                    priority=TaskPriority.from_str(str(item.get('priority', 'NORMAL'))),
                    created_at=float(item.get('created_at', 0.0)),
                    deadline=deadline,
                    metadata=item.get('metadata', {}),
                    release_time_sec=float(item.get('release_time_sec', 0.0)),
                )
                tasks.append(task)
            return tasks

        # Mode B: Procedural deterministic generation
        config = TaskGeneratorConfig(
            task_count=int(workload_data.get('task_count', 10)),
            seed=int(workload_data.get('seed', 42)),
            mode=str(workload_data.get('generation_mode', 'STATION_PAIR')),
            id_prefix=str(workload_data.get('id_prefix', 'task')),
            base_time=float(workload_data.get('base_time', 0.0)),
        )

        if 'staged_releases' in workload_data:
            config.staged_releases = list(workload_data['staged_releases'])

        if 'priority_weights' in workload_data:
            config.priority_weights = {
                k: float(v) for k, v in workload_data['priority_weights'].items()
            }

        if 'deadline' in workload_data:
            d_cfg = workload_data['deadline']
            config.deadline_enabled = bool(d_cfg.get('enabled', True))
            config.deadline_fraction = float(d_cfg.get('fraction', 1.0))
            if 'window_sec' in d_cfg:
                config.deadline_window_sec = (
                    float(d_cfg['window_sec'][0]),
                    float(d_cfg['window_sec'][1]),
                )

        if 'pickup_stations' in workload_data:
            config.pickup_stations = [
                (float(p[0]), float(p[1])) for p in workload_data['pickup_stations']
            ]

        if 'dropoff_stations' in workload_data:
            config.dropoff_stations = [
                (float(d[0]), float(d[1])) for d in workload_data['dropoff_stations']
            ]

        generator = TaskGenerator(config)
        return generator.generate_workload()

    @staticmethod
    def save_to_yaml(tasks: List[Task], output_path: str, description: str = '') -> None:
        """Export a generated task list to a reproducible YAML file."""
        data = {
            'workload': {
                'description': description or f'Workload of {len(tasks)} tasks',
                'task_count': len(tasks),
                'explicit_tasks': [t.to_dict() for t in tasks],
            }
        }
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, sort_keys=False)

