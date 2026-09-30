"""Unit tests for amr_fleet_sim GridWorld model."""

from amr_fleet_sim.grid_world import GridWorld
import pytest


def test_grid_initialization():
    grid = GridWorld(5, 5, obstacles=[(2, 2), (2, 3)])
    assert grid.width == 5
    assert grid.height == 5
    assert (2, 2) in grid.obstacles
    assert grid.is_free((0, 0))
    assert not grid.is_free((2, 2))
    assert not grid.is_free((10, 10))


def test_invalid_dimensions():
    with pytest.raises(ValueError):
        GridWorld(0, 5)
    with pytest.raises(ValueError):
        GridWorld(5, -1)


def test_out_of_bounds_obstacle():
    with pytest.raises(ValueError):
        GridWorld(5, 5, obstacles=[(5, 5)])


def test_neighbors_with_obstacles():
    grid = GridWorld(3, 3, obstacles=[(1, 0), (0, 1)])
    # At (0,0), neighbors are (0,0) [wait] since (1,0) and (0,1) are obstacles
    neighbors = grid.get_neighbors((0, 0), allow_wait=True)
    assert neighbors == [(0, 0)]

    neighbors_no_wait = grid.get_neighbors((0, 0), allow_wait=False)
    assert neighbors_no_wait == []


def test_manhattan_distance():
    assert GridWorld.manhattan_distance((0, 0), (3, 4)) == 7
    assert GridWorld.manhattan_distance((2, 2), (2, 2)) == 0


def test_vertex_and_edge_conflicts():
    # Vertex conflict: same spot at same time
    assert GridWorld.has_vertex_conflict((2, 3), (2, 3))
    assert not GridWorld.has_vertex_conflict((2, 3), (2, 4))

    # Edge conflict: swap collision (Agent A: (1,1)->(1,2), Agent B: (1,2)->(1,1))
    assert GridWorld.has_edge_conflict((1, 1), (1, 2), (1, 2), (1, 1))
    # Non-conflicting traverse in same direction
    assert not GridWorld.has_edge_conflict((1, 1), (1, 2), (1, 1), (1, 2))


def test_from_ascii():
    ascii_map = """
    ...
    .@.
    ...
    """
    grid = GridWorld.from_ascii(ascii_map)
    assert grid.width == 3
    assert grid.height == 3
    assert not grid.is_free((1, 1))
    assert grid.is_free((0, 0))


def test_coordinate_conversion():
    grid = GridWorld(32, 32, resolution=0.5)
    assert grid.to_grid(0.2, 0.2) == (0, 0)
    assert grid.to_grid(2.2, 5.7) == (4, 11)
    assert grid.to_grid(16.0, 16.0) == (31, 31)

    # Round-trip center check
    grid_pos = (4, 11)
    world_pos = grid.to_world(grid_pos)
    assert world_pos == (2.25, 5.75)
    assert grid.to_grid(world_pos[0], world_pos[1]) == grid_pos


def test_create_warehouse_grid():
    grid = GridWorld.create_warehouse_grid(resolution=0.5, warehouse_size=16.0)
    assert grid.width == 32
    assert grid.height == 32
    assert grid.resolution == 0.5

    # Center of warehouse corridor (8.0, 2.0) should be free
    assert grid.is_free(grid.to_grid(8.0, 2.0))
    # Pickup stations at (2.0, 2.0) and (2.0, 13.0) should be free
    assert grid.is_free(grid.to_grid(2.0, 2.0))
    assert grid.is_free(grid.to_grid(2.0, 13.0))

    # Rack 1 at center (4.5, 5.5) must be an obstacle
    assert not grid.is_free(grid.to_grid(4.5, 5.5))
    # Rack 2 at center (4.5, 10.5) must be an obstacle
    assert not grid.is_free(grid.to_grid(4.5, 10.5))

    # Perimeter walls must be obstacles
    assert not grid.is_free(grid.to_grid(0.1, 8.0))
    assert not grid.is_free(grid.to_grid(15.9, 8.0))


def test_from_yaml_m9_v1():
    import os
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )))
    yaml_path = os.path.join(
        repo_root, 'config', 'maps', 'warehouse_m9_v1.yaml'
    )
    if not os.path.exists(yaml_path):
        pytest.skip('warehouse_m9_v1.yaml not found')

    grid = GridWorld.from_yaml(yaml_path)
    assert grid.width == 64
    assert grid.height == 64
    assert grid.resolution == 0.5
    assert grid.map_id == 'warehouse_m9_v1'

    # Verify stations are loaded
    assert len(grid.stations.get('pickups', [])) == 8
    assert len(grid.stations.get('dropoffs', [])) == 4
    assert len(grid.stations.get('charging', [])) == 8

    # Free positions: Central arterial (16.0, 16.0), pickups, dropoffs
    assert grid.is_free(grid.to_grid(16.0, 16.0))
    assert grid.is_free(grid.to_grid(2.5, 2.0))
    assert grid.is_free(grid.to_grid(8.5, 8.5))

    # Rack obstacles: rack_1 at (5.5, 5.0), rack_16 at (26.5, 27.0)
    assert not grid.is_free(grid.to_grid(5.5, 5.0))
    assert not grid.is_free(grid.to_grid(26.5, 27.0))

    # Perimeter walls: (0.2, 16.0), (31.8, 16.0)
    assert not grid.is_free(grid.to_grid(0.2, 16.0))
    assert not grid.is_free(grid.to_grid(31.8, 16.0))

