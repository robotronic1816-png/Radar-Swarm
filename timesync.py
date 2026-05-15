"""
Phase 3C: Temporal Synchronization

This module simulates local drone clock drift and provides a
network consensus algorithm for timestamp synchronization.

Theory Reference:
- Equation 191: Distributed consensus on timestamps
- Phase 3C: Temporal clock alignment for decentralized drones
"""

import numpy as np


class Clock:
    """Simulate a local drone clock with drift."""

    def __init__(self, drone_id, drift_rate_ppm=None, initial_offset=0.0):
        """
        Args:
            drone_id: Unique identifier for the drone.
            drift_rate_ppm: Clock skew in parts-per-million.
                If None, a small random drift is chosen.
            initial_offset: Initial bias relative to true time.
        """
        self.drone_id = drone_id
        self.drift_rate_ppm = drift_rate_ppm if drift_rate_ppm is not None else np.random.normal(0.0, 1.0)
        self.drift_rate = self.drift_rate_ppm * 1e-6
        self.offset = initial_offset
        self.local_time = 0.0

    def tick(self, dt):
        """Advance local clock by dt, including drift."""
        self.local_time += dt * (1.0 + self.drift_rate)
        return self.read()

    def read(self):
        """Read the current local time, including offset."""
        return self.local_time + self.offset

    def correct(self, correction):
        """Apply a correction to the local clock offset."""
        self.offset += correction

    def set_offset(self, offset):
        """Override the local clock offset explicitly."""
        self.offset = offset


class TimeSync:
    """Consensus-based synchronization for decentralized drone clocks."""

    @staticmethod
    def consensus_step(local_times, adjacency, alpha=0.5):
        """Single consensus update step using neighbor exchange."""
        next_times = {}
        for drone_id, t_i in local_times.items():
            neighbors = adjacency.get(drone_id, [])
            if not neighbors:
                next_times[drone_id] = t_i
                continue

            neighbor_times = [local_times[j] for j in neighbors if j in local_times]
            if not neighbor_times:
                next_times[drone_id] = t_i
                continue

            average_neighbor = np.mean(neighbor_times)
            next_times[drone_id] = t_i + alpha * (average_neighbor - t_i)

        return next_times

    @staticmethod
    def synchronize_clocks(drones, adjacency, iterations=5, alpha=0.4):
        """Synchronize all drone clocks via iterative consensus.

        Args:
            drones: List of Drone instances, each with a `clock` attribute.
            adjacency: Dict mapping drone_id -> list of neighbor drone_ids.
            iterations: Number of consensus exchange rounds.
            alpha: Consensus step size (0 < alpha <= 1).

        Returns:
            Dict of synchronized local times by drone_id.
        """
        local_times = {d.drone_id: d.clock.read() for d in drones}

        for _ in range(iterations):
            local_times = TimeSync.consensus_step(local_times, adjacency, alpha=alpha)

        # Apply the consensus correction to each drone's local clock.
        for d in drones:
            corrected_time = local_times[d.drone_id]
            current_time = d.clock.read()
            d.clock.correct(corrected_time - current_time)

        return local_times

    @staticmethod
    def build_neighbor_graph(drones):
        """Build a fully connected neighbor graph for small swarms."""
        graph = {}
        ids = [d.drone_id for d in drones]
        for drone_id in ids:
            graph[drone_id] = [other_id for other_id in ids if other_id != drone_id]
        return graph
