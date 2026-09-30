"""Unit and integration tests for YAVI-SIH26123 Resilience Testing Dashboard."""

import http.client
import http.server
import json
import os
import socket
import socketserver
import sys
import threading
import time
from typing import Optional, Tuple
import unittest

import rclpy

# Add scripts directory to path to import resilience dashboard components
SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts')
)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from resilience_dashboard import (  # noqa: E402
    RecoveryPipelineTracker,
    ResilienceHTTPHandler,
    ResilienceMonitorNode,
    RobotHealthTracker,
)


def get_free_port() -> int:
    """Find an available TCP port for testing."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return int(s.getsockname()[1])


class TestResilienceDashboardComponents(unittest.TestCase):
    """Test resilience dashboard data structures and state machines."""

    def test_robot_health_tracker_serialization(self) -> None:
        """Test RobotHealthTracker initialization and dict serialization."""
        tracker = RobotHealthTracker('amr_test', x=5.0, y=10.0, yaw=1.57)
        tracker.health_state = 'HEALTHY'
        tracker.active_task_id = 'T_99'
        tracker.assigned_bundle = ['T_99', 'T_100']

        data = tracker.to_dict(now=time.time())
        self.assertEqual(data['robot_id'], 'amr_test')
        self.assertEqual(data['x'], 5.0)
        self.assertEqual(data['y'], 10.0)
        self.assertEqual(data['health_state'], 'HEALTHY')
        self.assertEqual(data['active_task_id'], 'T_99')
        self.assertIn('T_99', data['assigned_bundle'])

    def test_recovery_pipeline_tracker_stages(self) -> None:
        """Test 7-stage recovery pipeline state machine and timing records."""
        pipeline = RecoveryPipelineTracker()
        pipeline.reset(active_victim='amr_1', task_id='T_2')

        self.assertEqual(pipeline.active_victim, 'amr_1')
        self.assertEqual(pipeline.orphaned_task, 'T_2')
        self.assertFalse(pipeline.completed)

        # Mark stages sequentially
        pipeline.mark_stage(
            'STAGE_1_FAULT_INJECTED', 'COMPLETED', 'Injected KILL'
        )
        pipeline.mark_stage(
            'STAGE_2_PEER_DETECTED', 'COMPLETED', 'Peer failure detected'
        )
        pipeline.mark_stage(
            'STAGE_4_TASK_RECLAIMED', 'COMPLETED', 'Task T_2 reclaimed'
        )
        pipeline.mark_stage(
            'STAGE_7_EXECUTION_RESUMED', 'COMPLETED', 'Transit resumed'
        )

        data = pipeline.to_dict()
        self.assertTrue(pipeline.completed)
        self.assertEqual(len(data['stages']), 7)
        s1 = next(
            s for s in data['stages'] if s['name'] == 'STAGE_1_FAULT_INJECTED'
        )
        self.assertEqual(s1['status'], 'COMPLETED')
        self.assertIsNotNone(s1['delta_s'])

    def test_m3_recovery_pipeline_tracker_stages(self) -> None:
        """Test Milestone 3 9-stage environmental recovery stepper."""
        pipeline = RecoveryPipelineTracker()
        pipeline.reset(mode='M3')

        self.assertEqual(pipeline.mode, 'M3')
        self.assertEqual(len(pipeline.stage_names), 9)
        self.assertEqual(pipeline.stage_names[0], 'STAGE_1_OBSTACLE_INJECTED')
        self.assertEqual(pipeline.stage_names[-1], 'STAGE_9_EXECUTION_RESUMED')

        pipeline.mark_stage(
            'STAGE_1_OBSTACLE_INJECTED', 'COMPLETED', 'Obstacle placed'
        )
        pipeline.mark_stage(
            'STAGE_2_SENSOR_OBSERVED', 'COMPLETED', 'LiDAR detected'
        )
        pipeline.mark_stage(
            'STAGE_9_EXECUTION_RESUMED', 'COMPLETED', 'Transit resumed'
        )

        data = pipeline.to_dict()
        self.assertTrue(pipeline.completed)
        self.assertEqual(len(data['stages']), 9)


class TestResilienceDashboardServer(unittest.TestCase):
    """End-to-end HTTP API tests against the embedded dashboard web server."""

    server: Optional[http.server.HTTPServer] = None
    server_thread: Optional[threading.Thread] = None
    node: Optional[ResilienceMonitorNode] = None
    test_port: int = 8091

    @classmethod
    def setUpClass(cls) -> None:
        """Boot ROS 2 test node and embedded HTTP server."""
        if not rclpy.ok():
            rclpy.init()
        cls.test_port = get_free_port()
        cls.node = ResilienceMonitorNode(sim_mode=True)
        ResilienceHTTPHandler.node = cls.node

        class TestHTTPServer(
            socketserver.ThreadingMixIn, http.server.HTTPServer
        ):
            daemon_threads = True

        cls.server = TestHTTPServer(
            ('127.0.0.1', cls.test_port), ResilienceHTTPHandler
        )
        cls.server_thread = threading.Thread(
            target=cls.server.serve_forever, daemon=True
        )
        cls.server_thread.start()
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls) -> None:
        """Shut down HTTP server and ROS 2 test node."""
        if cls.server:
            cls.server.shutdown()
            cls.server.server_close()
        if cls.node:
            cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _request(
        self, method: str, path: str, body: Optional[dict] = None
    ) -> Tuple[int, str]:
        """Perform test HTTP request."""
        conn = http.client.HTTPConnection('127.0.0.1', self.test_port)
        headers = {'Content-Type': 'application/json'} if body else {}
        data = json.dumps(body) if body else None
        conn.request(method, path, body=data, headers=headers)
        resp = conn.getresponse()
        resp_data = resp.read().decode('utf-8')
        conn.close()
        return resp.status, resp_data

    def test_get_index_html(self) -> None:
        """Verify GET / delivers GUI single page app."""
        status, content = self._request('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn('YAVI-SIH26123', content)
        self.assertIn('FAULT INJECTION & RESILIENCE CONSOLE', content)
        self.assertIn('2D WAREHOUSE WORLD', content)

    def test_get_api_state(self) -> None:
        """Verify GET /api/state returns full telemetry and invariants."""
        status, content = self._request('GET', '/api/state')
        self.assertEqual(status, 200)
        data = json.loads(content)
        self.assertIn('fleet_status', data)
        self.assertIn('robots', data)
        self.assertIn('amr_0', data['robots'])
        self.assertIn('amr_1', data['robots'])
        self.assertIn('amr_2', data['robots'])
        self.assertIn('invariants', data)
        self.assertIn('recovery_pipeline', data)
        self.assertIn('map', data)
        self.assertIn('events_log', data)

    def test_post_fault_injection_and_restore(self) -> None:
        """Verify POST /api/fault/inject and POST /api/fault/restore."""
        # Inject KILL to amr_1
        status, content = self._request('POST', '/api/fault/inject', {
            'robot_id': 'amr_1',
            'fault_type': 'KILL',
            'duration_sec': 0.0,
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Check state reflected
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['robots']['amr_1']['health_state'], 'FAILED')
        self.assertTrue(state['robots']['amr_1']['is_chassis_obstacle'])

        # Restore amr_1
        status, content = self._request('POST', '/api/fault/restore', {
            'robot_id': 'amr_1',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['robots']['amr_1']['health_state'], 'HEALTHY')
        self.assertFalse(state['robots']['amr_1']['is_chassis_obstacle'])

    def test_post_fleet_estop_and_resume(self) -> None:
        """Verify POST /api/fleet/estop and POST /api/fleet/resume."""
        # E-stop
        status, content = self._request('POST', '/api/fleet/estop')
        self.assertEqual(status, 200)

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['fleet_status'], 'EMERGENCY_STOP')
        for bot in state['robots'].values():
            self.assertEqual(bot['health_state'], 'EMERGENCY_STOP')

        # Resume
        status, content = self._request('POST', '/api/fleet/resume')
        self.assertEqual(status, 200)

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['fleet_status'], 'HEALTHY')
        for bot in state['robots'].values():
            self.assertEqual(bot['health_state'], 'HEALTHY')

    def test_post_scenario_trigger_m1_a(self) -> None:
        """Verify POST /api/scenario/trigger executes scenario M1-A."""
        if self.node:
            self.node.scenario_status = 'IDLE'
        status, content = self._request('POST', '/api/scenario/trigger', {
            'scenario_id': 'M1-A',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Verify scenario status changes to RUNNING or PASSED
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertIn(state['scenario']['status'], ('RUNNING', 'PASSED'))

    def test_provenance_and_invariants_reported(self) -> None:
        """Verify contact sensor provenance note is explicitly stated."""
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        proxy = state['invariants']['gazebo_safety_proxy']
        self.assertIn('2D OBB Geometric Proxy', proxy['detection_method'])
        self.assertIn('not raw physical bumper', proxy['provenance_note'])

    def test_get_export_endpoints(self) -> None:
        """Verify report export in JSON and Markdown formats."""
        status, json_content = self._request('GET', '/api/export')
        self.assertEqual(status, 200)
        parsed = json.loads(json_content)
        self.assertIn('robots', parsed)

        status, md_content = self._request('GET', '/api/export/markdown')
        self.assertEqual(status, 200)
        self.assertIn(
            '# YAVI-SIH26123 Milestone 1.1 Resilience & Recovery Report', md_content
        )

    def test_network_telemetry_in_state(self) -> None:
        """Verify GET /api/state returns M2 network telemetry/invariants."""
        status, content = self._request('GET', '/api/state')
        self.assertEqual(status, 200)
        data = json.loads(content)
        self.assertIn('network_telemetry', data)
        self.assertIn('total_packets_sent', data['network_telemetry'])
        self.assertIn('overall_observed_loss', data['network_telemetry'])

        # Robot network block
        bot = data['robots']['amr_0']
        self.assertIn('network', bot)
        self.assertIn('profile', bot['network'])
        self.assertIn('configured_loss', bot['network'])
        self.assertIn('observed_loss', bot['network'])

        # M2 Invariants
        invs = data['invariants']
        self.assertIn('false_failure_rejection', invs)
        self.assertIn('reconnection_zero_duplication', invs)
        self.assertIn('reservation_hold_on_expiry', invs)

    def test_post_network_impairment_and_reconnect(self) -> None:
        """Verify POST /api/network/impairment and /api/network/reconnect."""
        # Apply LOSS_HIGH
        status, content = self._request('POST', '/api/network/impairment', {
            'robot_id': 'amr_2',
            'profile': 'LOSS_HIGH',
            'loss_rate': 0.35,
            'delay_ms': 50.0,
            'duration_sec': 0.0,
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Verify state
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        bot_net = state['robots']['amr_2']['network']
        self.assertEqual(bot_net['profile'], 'LOSS_HIGH')
        self.assertAlmostEqual(bot_net['configured_loss'], 0.35)

        # Disconnect amr_2
        status, _ = self._request('POST', '/api/network/impairment', {
            'robot_id': 'amr_2',
            'profile': 'OUTAGE',
            'loss_rate': 1.0,
            'duration_sec': 0.0,
        })
        self.assertEqual(status, 200)
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['robots']['amr_2']['health_state'], 'COMM_LOSS')

        # Reconnect amr_2
        status, content = self._request('POST', '/api/network/reconnect', {
            'robot_id': 'amr_2',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertEqual(state['robots']['amr_2']['health_state'], 'HEALTHY')
        self.assertEqual(
            state['robots']['amr_2']['network']['profile'], 'NORMAL'
        )

    def test_post_scenario_trigger_m2_a(self) -> None:
        """Verify POST /api/scenario/trigger executes scenario M2-A."""
        if self.node:
            self.node.scenario_status = 'IDLE'
        status, content = self._request('POST', '/api/scenario/trigger', {
            'scenario_id': 'M2-A',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Check state: pipeline mode is M2, 9 stages present
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        pipe = state['recovery_pipeline']
        self.assertEqual(pipe.get('mode'), 'M2')
        self.assertEqual(len(pipe.get('stages', [])), 9)

    def test_post_environment_blockage_and_step(self) -> None:
        """Verify POST /api/environment/blockage and /api/environment/step."""
        # Inject aisle blockage
        status, content = self._request('POST', '/api/environment/blockage', {
            'action': 'INJECT',
            'blockage_id': 'BLK_TEST_01',
            'cells': [[7, 4], [7, 5]],
            'duration_sec': 0.0,
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Verify reflected in state
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        env = state.get('environment_telemetry', {})
        blockages = env.get('active_blockages', [])
        self.assertTrue(any(
            b.get('blockage_id') == 'BLK_TEST_01' for b in blockages
        ))

        # Step M3 recovery pipeline
        status, content = self._request('POST', '/api/environment/step', {
            'stage_name': '01_OBSTACLE_DETECTED',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Clear aisle blockage
        status, content = self._request('POST', '/api/environment/blockage', {
            'action': 'CLEAR',
            'blockage_id': 'BLK_TEST_01',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        env = state.get('environment_telemetry', {})
        blockages = env.get('active_blockages', [])
        self.assertFalse(any(
            b.get('blockage_id') == 'BLK_TEST_01' for b in blockages
        ))

    def test_post_environment_conflict(self) -> None:
        """Verify POST /api/environment/conflict injects conflict."""
        status, content = self._request('POST', '/api/environment/conflict', {
            'conflict_type': 'CROSSING_TRAJECTORIES',
            'robot_ids': ['amr_0', 'amr_1'],
            'cell': [4, 5],
            'time_step': 3,
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Verify reflected in state
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        env = state.get('environment_telemetry', {})
        conflicts = env.get('active_conflicts', [])
        self.assertTrue(any(
            c.get('conflict_type') == 'CROSSING_TRAJECTORIES'
            for c in conflicts
        ))

    def test_post_scenario_trigger_m3_a(self) -> None:
        """Verify POST /api/scenario/trigger executes M3-A with 9 stages."""
        if self.node:
            self.node.scenario_status = 'IDLE'
        status, content = self._request('POST', '/api/scenario/trigger', {
            'scenario_id': 'M3-A',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Check state: pipeline mode is M3, 9 stages present
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        pipe = state['recovery_pipeline']
        self.assertEqual(pipe.get('mode'), 'M3')
        self.assertEqual(len(pipe.get('stages', [])), 9)

    def test_post_m4_inject_compound(self) -> None:
        """Verify POST /api/m4/inject_compound registers compound fault."""
        status, content = self._request('POST', '/api/m4/inject_compound', {
            'scenario_id': 'M4-A',
            'robot_ids': ['amr_1'],
            'blockage_cells': [[7, 4], [7, 5]],
            'packet_loss_rate': 0.0,
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        # Verify reflected in state
        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        env = state.get('environment_telemetry', {})
        compound_faults = env.get('active_compound_faults', [])
        self.assertTrue(any(
            c.get('scenario_id') == 'M4-A' for c in compound_faults
        ))

    def test_get_m4_status(self) -> None:
        """Verify GET /api/m4/status returns structured status."""
        status, content = self._request('GET', '/api/m4/status')
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertIn('scenario_status', res)
        self.assertIn('active_compound_faults', res)

    def test_post_scenario_trigger_m4_g(self) -> None:
        """Verify POST /api/scenario/trigger executes M4-G scenario."""
        if self.node:
            self.node.scenario_status = 'IDLE'
        status, content = self._request('POST', '/api/scenario/trigger', {
            'scenario_id': 'M4-G',
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        pipe = state['recovery_pipeline']
        self.assertEqual(pipe.get('mode'), 'M4')
        self.assertEqual(len(pipe.get('stages', [])), 7)

    def test_post_m4_compose_and_run(self) -> None:
        """Verify POST /api/m4/compose_and_run configures and executes custom compound."""
        if self.node:
            self.node.scenario_status = 'IDLE'
        status, content = self._request('POST', '/api/m4/compose_and_run', {
            'scenario_id': 'CUSTOM_COMPOUND',
            'robot_ids': ['amr_1'],
            'fault_type': 'KILL',
            'network_profile': 'LOSS_HIGH',
            'packet_loss_rate': 0.35,
            'duration_sec': 5.0,
            'blockage_cells': [[7, 4], [7, 5]],
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        self.assertIn('compound_fleet_response', state)
        cmp_resp = state['compound_fleet_response']
        self.assertIn('cbba_reallocation', cmp_resp)
        self.assertIn('dynamic_replanning', cmp_resp)
        self.assertIn('local_recovery', cmp_resp)

        invs = state.get('invariants', {})
        self.assertIn('m4_task_uniqueness_i1', invs)
        self.assertIn('m4_reservation_exclusivity_i2', invs)
        self.assertIn('m4_local_clearance_i3', invs)
        self.assertIn('m4_tiered_fault_discrimination', invs)
        self.assertIn('m4_monotonic_cas_reconnection', invs)
        self.assertEqual(invs['m4_task_uniqueness_i1']['status'], 'PASS')
        self.assertEqual(invs['m4_reservation_exclusivity_i2']['status'], 'PASS')
        self.assertEqual(invs['m4_local_clearance_i3']['status'], 'PASS')

    def test_m4_markdown_report_includes_compound_telemetry(self) -> None:
        """Verify markdown report formatting for Milestone 4 compound resilience."""
        if self.node:
            self.node.recovery_tracker.mode = 'M4'
        status, md_content = self._request('GET', '/api/export/markdown')
        self.assertEqual(status, 200)
        self.assertIn('Milestone 4 Compound Fault & Multi-Domain Resilience Report', md_content)
        self.assertIn('Milestone 4 Formal Invariants & Compound Verification', md_content)
        self.assertIn('I1: Task Uniqueness', md_content)
        self.assertIn('I2: Spacetime Exclusivity', md_content)
        self.assertIn('I3: Local LiDAR Clearance', md_content)
        self.assertIn('Three-Tier Fleet Response Telemetry', md_content)

    def test_clear_all_blockages(self) -> None:
        """Verify clearing all blockages via action=CLEAR and blockage_id=ALL."""
        # Add 2 blockages
        self._request('POST', '/api/environment/blockage', {
            'action': 'INJECT', 'blockage_id': 'B1', 'cells': [[1, 1]]
        })
        self._request('POST', '/api/environment/blockage', {
            'action': 'INJECT', 'blockage_id': 'B2', 'cells': [[2, 2]]
        })
        # Clear all
        status, content = self._request('POST', '/api/environment/blockage', {
            'action': 'CLEAR', 'blockage_id': 'ALL'
        })
        self.assertEqual(status, 200)
        res = json.loads(content)
        self.assertTrue(res.get('success'))

        _, state_content = self._request('GET', '/api/state')
        state = json.loads(state_content)
        blockages = state.get('environment_telemetry', {}).get('active_blockages', [])
        self.assertEqual(len(blockages), 0)


