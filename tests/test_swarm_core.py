#!/usr/bin/env python
"""Core regression checks for tracking, time sync, and assignment."""

import numpy as np

from controller import assign_drones
from drone import Drone
from geometry import fisher_information_matrix, gdop, repositioning_vector_for_drone
from timesync import TimeSync
from tracker import MultiTargetTracker


def test_tracker_maintains_track_identity():
    tracker = MultiTargetTracker(dt=0.1, gate_distance=25.0)

    for step in range(4):
        measurements = [
            {
                "position": np.array([100.0 + step, 50.0, 10.0]),
                "velocity": np.array([10.0, 0.0, 0.0]),
                "covariance": np.eye(6) * 2.0,
            },
            {
                "position": np.array([250.0, 150.0 - step, 20.0]),
                "velocity": np.array([0.0, -10.0, 0.0]),
                "covariance": np.eye(6) * 2.0,
            },
        ]
        tracks = tracker.step(measurements)

    assert len(tracks) == 2
    assert len({track["track_id"] for track in tracks}) == 2
    assert all(track["hits"] >= 4 for track in tracks)


def test_assignment_prioritizes_uncertain_track():
    drones = [
        Drone([0.0, 0.0, 0.0], radar_enabled=False, drone_id=1),
    ]
    tracks = [
        {
            "position": np.array([20.0, 0.0, 0.0]),
            "velocity": np.zeros(3),
            "covariance": np.eye(6) * 1.0,
        },
        {
            "position": np.array([25.0, 0.0, 0.0]),
            "velocity": np.zeros(3),
            "covariance": np.eye(6) * 100.0,
        },
    ]

    assign_drones(drones, tracks, gamma=0.2)

    assert np.allclose(drones[0].target["position"], tracks[1]["position"])


def test_time_sync_reduces_clock_spread():
    drones = [
        Drone([0.0, 0.0, 0.0], radar_enabled=False, drone_id=1),
        Drone([0.0, 0.0, 0.0], radar_enabled=False, drone_id=2),
        Drone([0.0, 0.0, 0.0], radar_enabled=False, drone_id=3),
    ]
    drones[0].clock.set_offset(-0.2)
    drones[1].clock.set_offset(0.1)
    drones[2].clock.set_offset(0.4)

    before = np.ptp([drone.clock.read() for drone in drones])
    TimeSync.synchronize_clocks(drones, TimeSync.build_neighbor_graph(drones), iterations=8)
    after = np.ptp([drone.clock.read() for drone in drones])

    assert after < before


def test_proportional_navigation_updates_velocity_in_3d():
    drone = Drone(
        [0.0, 0.0, 0.0],
        speed=50.0,
        radar_enabled=False,
        drone_id=1,
        navigation_constant=3.0,
        max_acceleration=30.0,
    )
    drone.velocity = np.array([20.0, 0.0, 0.0])
    drone.assign_target({
        "position": np.array([100.0, 40.0, 30.0]),
        "velocity": np.array([0.0, 10.0, 0.0]),
    })

    drone.update(0.1)

    assert drone.position[0] > 0.0
    assert abs(drone.velocity[1]) > 0.0
    assert abs(drone.velocity[2]) > 0.0
    assert np.linalg.norm(drone.velocity) <= drone.speed + 1e-9


def test_geometry_gdop_repositioning_vector():
    drones = [
        Drone([0.0, 0.0, 50.0], radar_enabled=False, drone_id=1),
        Drone([150.0, 0.0, 50.0], radar_enabled=False, drone_id=2),
        Drone([40.0, 120.0, 70.0], radar_enabled=False, drone_id=3),
    ]
    target = {
        "position": np.array([80.0, 50.0, 30.0]),
        "velocity": np.array([5.0, -2.0, 0.0]),
    }

    fim = fisher_information_matrix(drones, target)
    score_before = gdop(drones, target)
    move = repositioning_vector_for_drone(
        drones,
        target,
        drone_index=0,
        step_size=5.0,
        learning_rate=20.0,
        max_vector_norm=15.0,
    )

    new_positions = [d.position.copy() for d in drones]
    new_positions[0] = new_positions[0] + move
    score_after = gdop(new_positions, target)

    assert fim.shape == (6, 6)
    assert np.isfinite(score_before)
    assert move.shape == (3,)
    assert np.linalg.norm(move) <= 15.0 + 1e-9
    assert score_after <= score_before


if __name__ == "__main__":
    test_tracker_maintains_track_identity()
    test_assignment_prioritizes_uncertain_track()
    test_time_sync_reduces_clock_spread()
    test_proportional_navigation_updates_velocity_in_3d()
    test_geometry_gdop_repositioning_vector()
    print("Core swarm regression checks passed")
