"""
Sensor-Driven Local Obstacle Detector & Bounded Recovery for YAVI-SIH26123 M3.

Transforms onboard raw LaserScan sensor observations into local occupancy
representations and evaluates bounded local recovery without invading higher-level
global planning responsibilities.

Critical Architectural Principles:
1. Sensor / Oracle Separation:
   Obstacles detected via LiDAR represent true sensor observations, strictly
   isolated from scripted environment-oracle events (/environment/aisle_blockages).
2. Bounded Local Recovery:
   The detector evaluates 1-step immediate hazard response (emergency brake,
   bounded lateral sidestep, or LOCAL_SAFETY_HOLD). It does NOT implement a
   secondary global pathfinder. Global rerouting is delegated to RHCR / A*.
3. Safety Primacy:
   The 0.28m experimental/design threshold enforces immediate reactive stopping
   independent of planning and network consensus.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.coordination_models import Position
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_sim.grid_world import GridWorld


class LocalRecoveryAction(Enum):
    """Action recommendation from bounded local recovery evaluator."""

    MAINTAIN_COURSE = 'MAINTAIN_COURSE'
    EMERGENCY_BRAKE = 'EMERGENCY_BRAKE'
    LOCAL_SIDESTEP = 'LOCAL_SIDESTEP'
    LOCAL_SAFETY_HOLD = 'LOCAL_SAFETY_HOLD'


@dataclass
class LocalObstacle:
    """Structure representing a sensor-observed obstacle point and cell."""

    point_robot_frame: Tuple[float, float]
    point_world_frame: Tuple[float, float]
    grid_cell: Position
    distance: float
    detection_time: float
    is_immediate_hazard: bool


class LocalObstacleDetector:
    """
    Onboard sensor-driven obstacle detector and bounded local recovery engine.

    Converts raw LaserScan range arrays into 2D Cartesian coordinates and discrete
    GridWorld cells, evaluates forward path obstruction, and determines whether
    a safe 1-step local avoidance maneuver exists or whether LOCAL_SAFETY_HOLD
    must be engaged while requesting a global replan.
    """

    def __init__(
        self,
        grid_resolution: float = 0.5,
        safety_threshold_m: float = 0.28,
        detection_horizon_m: float = 1.5,
        forward_arc_deg: float = 24.0,
    ) -> None:
        """
        Initialize detector parameters.

        :param grid_resolution: Discretization size of grid cells in meters.
        :param safety_threshold_m: Experimental/design reactive braking distance.
        :param detection_horizon_m: Forward distance to inspect for dynamic obstructions.
        :param forward_arc_deg: Angular cone width (+/- half angle) for collision hazard.
        """
        self.grid_resolution = grid_resolution
        self.safety_threshold_m = safety_threshold_m
        self.detection_horizon_m = detection_horizon_m
        self.forward_arc_rad = math.radians(forward_arc_deg / 2.0)

        # Cache of recently detected sensor obstacles: cell -> LocalObstacle
        self.observed_obstacles: Dict[Position, LocalObstacle] = {}
        self.last_scan_time: float = 0.0

    def clear(self) -> None:
        """Clear all sensor-observed obstacles."""
        self.observed_obstacles.clear()

    def process_scan(
        self,
        ranges: List[float],
        range_min: float,
        range_max: float,
        angle_min: float,
        angle_increment: float,
        robot_pose: Tuple[float, float, float],
        current_time: Optional[float] = None,
    ) -> List[LocalObstacle]:
        """
        Process raw LaserScan ranges and project into world grid cells.

        :param ranges: Array of radial distances from LiDAR sensor.
        :param range_min: Minimum valid sensor range.
        :param range_max: Maximum valid sensor range.
        :param angle_min: Starting angle of scan in radians.
        :param angle_increment: Angular distance between successive rays.
        :param robot_pose: Current (x, y, yaw) in world coordinates.
        :param current_time: Timestamp of observation in seconds.
        :return: List of newly detected LocalObstacle instances.
        """
        now = current_time if current_time is not None else time.time()
        self.last_scan_time = now
        rx, ry, ryaw = robot_pose

        detected: List[LocalObstacle] = []
        num_rays = len(ranges)
        if num_rays == 0:
            return detected

        for idx, r in enumerate(ranges):
            if r <= range_min or r >= range_max or math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + idx * angle_increment
            # Only consider rays within forward detection horizon
            if r > self.detection_horizon_m:
                continue

            # Point in robot coordinate frame
            xr = r * math.cos(angle)
            yr = r * math.sin(angle)

            # Transform to world coordinate frame
            xw = rx + xr * math.cos(ryaw) - yr * math.sin(ryaw)
            yw = ry + xr * math.sin(ryaw) + yr * math.cos(ryaw)

            gx = int(math.floor(xw / self.grid_resolution))
            gy = int(math.floor(yw / self.grid_resolution))
            cell = (gx, gy)

            # Immediate hazard if within the forward arc and closer than safety threshold
            in_forward_arc = abs(angle) <= self.forward_arc_rad
            is_hazard = bool(in_forward_arc and r < self.safety_threshold_m)

            obs = LocalObstacle(
                point_robot_frame=(round(xr, 3), round(yr, 3)),
                point_world_frame=(round(xw, 3), round(yw, 3)),
                grid_cell=cell,
                distance=round(r, 3),
                detection_time=now,
                is_immediate_hazard=is_hazard,
            )
            detected.append(obs)
            self.observed_obstacles[cell] = obs

        return detected

    def get_sensor_occupied_cells(self, max_age_sec: float = 3.0) -> Set[Position]:
        """
        Return currently active sensor-occupied grid cells within persistence age.

        Prunes stale observations older than max_age_sec.
        """
        now = self.last_scan_time if self.last_scan_time > 0 else time.time()
        stale_cells = [
            c for c, obs in self.observed_obstacles.items()
            if (now - obs.detection_time) > max_age_sec
        ]
        for c in stale_cells:
            self.observed_obstacles.pop(c, None)

        return set(self.observed_obstacles.keys())

    def has_immediate_hazard(self) -> bool:
        """Check if any currently observed obstacle breaches the 0.28m safety envelope."""
        return any(obs.is_immediate_hazard for obs in self.observed_obstacles.values())

    def check_footprint_clearance(
        self,
        cand: Position,
        current_cell: Position,
        grid: GridWorld,
        sensor_cells: Set[Position],
        res_table: SpaceTimeReservationTable,
        robot_id: str,
        current_time_step: int,
    ) -> Tuple[bool, str]:
        """
        Verify that a candidate cell provides clearance for the 0.65m x 0.45m AMR footprint.

        On a 0.5m grid, the 0.65m chassis length extends beyond a single cell along
        the direction of motion. This function verifies:
        1. Candidate cell is in bounds, free of static obstacles, and free of sensor obstacles.
        2. Candidate cell is not reserved at time_step + 1 by another robot.
        3. Longitudinal ahead cell (cand + heading) is checked: if in bounds, it must not be
           a static obstacle, a sensor-observed obstacle, or reserved by a peer at time_step + 1.
        4. Edge transitions do not conflict with peer reservations.
        5. If any clearance check fails, the candidate sidestep is rejected.

        :return: (is_clear, reason)
        """
        # 1. Primary candidate cell clearance
        if not grid.is_free(cand):
            return False, f'Candidate {cand} is a static obstacle or out of bounds'
        if cand in sensor_cells:
            return False, f'Candidate {cand} is occupied by sensor-observed obstacle'

        # 2. Reservation check on candidate cell at t+1
        if res_table.is_reserved(cell=cand, time_step=current_time_step + 1, robot_id=robot_id):
            return False, f'Candidate {cand} is reserved at t={current_time_step + 1}'

        # 3. Edge conflict check during transition
        if res_table.is_edge_conflict(
            from_pos=current_cell, to_pos=cand,
            time_step=current_time_step, robot_id=robot_id,
        ):
            return False, (
                f'Edge conflict traversing {current_cell}->{cand} at t={current_time_step}'
            )

        # 4. Footprint swept-envelope clearance:
        # 0.65m chassis length requires longitudinal clearance
        dx = cand[0] - current_cell[0]
        dy = cand[1] - current_cell[1]
        ahead_cell = (cand[0] + dx, cand[1] + dy)

        # If ahead cell is in grid, the front bumper (0.65m chassis) protrudes into it
        if grid.in_bounds(ahead_cell):
            if not grid.is_free(ahead_cell):
                return False, f'Footprint overhang: ahead cell {ahead_cell} is static obstacle'
            if ahead_cell in sensor_cells:
                return False, f'Footprint overhang: ahead cell {ahead_cell} is sensor obstacle'
            if res_table.is_reserved(
                cell=ahead_cell, time_step=current_time_step + 1, robot_id=robot_id,
            ):
                return False, (
                    f'Footprint overhang: ahead cell {ahead_cell} reserved at '
                    f't={current_time_step + 1}'
                )

        return True, 'Footprint envelope clear'

    def evaluate_local_recovery(
        self,
        current_pos: Tuple[float, float],
        current_cell: Position,
        planned_path_cells: List[Position],
        grid: GridWorld,
        res_table: SpaceTimeReservationTable,
        robot_id: str,
        current_time_step: int = 0,
    ) -> Tuple[LocalRecoveryAction, Optional[Position], str]:
        """
        Evaluate bounded local recovery when an obstacle appears on the planned path.

        Follows strict hierarchy:
        1. Immediate Emergency Brake if distance < safety_threshold_m (0.28m).
        2. Check if next 2-3 planned cells intersect sensor-detected obstacles.
        3. If obstructed, evaluate whether a safe 1-step footprint-aware lateral sidestep exists.
        4. If no safe sidestep exists, command LOCAL_SAFETY_HOLD (v=0) and request global replan.
        """
        sensor_cells = self.get_sensor_occupied_cells()

        # 1. Check reactive emergency brake condition (0.28m experimental threshold)
        if self.has_immediate_hazard():
            return (
                LocalRecoveryAction.EMERGENCY_BRAKE,
                None,
                f'Immediate obstacle hazard < {self.safety_threshold_m}m: emergency brake.',
            )

        # 2. Check if planned path intersects any sensor-observed obstacle
        horizon_to_check = planned_path_cells[:4]
        obstructed_cells = [c for c in horizon_to_check if c in sensor_cells]

        if not obstructed_cells:
            return (
                LocalRecoveryAction.MAINTAIN_COURSE,
                None,
                'Planned corridor clear of sensor obstacles.',
            )

        # 3. Path is obstructed: check for a safe lateral 1-step sidestep with footprint clearance
        neighbors = grid.get_neighbors(current_cell, allow_wait=False)
        valid_sidesteps: List[Position] = []

        for cand in neighbors:
            is_clear, _ = self.check_footprint_clearance(
                cand=cand,
                current_cell=current_cell,
                grid=grid,
                sensor_cells=sensor_cells,
                res_table=res_table,
                robot_id=robot_id,
                current_time_step=current_time_step,
            )
            if is_clear:
                valid_sidesteps.append(cand)

        if valid_sidesteps:
            sidestep = valid_sidesteps[0]
            return (
                LocalRecoveryAction.LOCAL_SIDESTEP,
                sidestep,
                f'Path obstructed at {obstructed_cells[0]}; '
                f'executing footprint-safe 1-step sidestep to {sidestep}.',
            )

        # 4. No safe local maneuver exists: enter bounded LOCAL_SAFETY_HOLD
        return (
            LocalRecoveryAction.LOCAL_SAFETY_HOLD,
            None,
            f'Path obstructed at {obstructed_cells[0]} and footprint clearance unavailable: '
            'entering LOCAL_SAFETY_HOLD and requesting global replan.',
        )

