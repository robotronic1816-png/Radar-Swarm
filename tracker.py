"""State-space tracking and multi-target association for the radar swarm.

The tracker uses a constant-velocity Kalman filter as the workhorse and a
lightweight IMM wrapper that blends constant-velocity and constant-acceleration
models. Track management uses global nearest-neighbor association with the
Hungarian algorithm.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment


def _as_state(measurement):
    """Normalize common measurement formats to a 6D Cartesian state."""
    if isinstance(measurement, dict):
        if "state" in measurement:
            state = np.asarray(measurement["state"], dtype=float)
        else:
            position = np.asarray(measurement.get("position", [0.0, 0.0, 0.0]), dtype=float)
            velocity = np.asarray(measurement.get("velocity", [0.0, 0.0, 0.0]), dtype=float)
            state = np.concatenate([position[:3], velocity[:3]])
    else:
        state = np.asarray(measurement, dtype=float)

    if state.shape[0] < 6:
        state = np.pad(state, (0, 6 - state.shape[0]))
    return state[:6]


def _as_covariance(measurement, default_var=25.0):
    if isinstance(measurement, dict) and measurement.get("covariance") is not None:
        cov = np.asarray(measurement["covariance"], dtype=float)
        if cov.shape == (6, 6):
            return cov
    return np.eye(6) * default_var


@dataclass
class KalmanTrack:
    """Single-target constant-velocity Kalman filter."""

    state: np.ndarray
    covariance: np.ndarray
    dt: float
    track_id: int
    process_var: float = 1.0
    hits: int = 1
    misses: int = 0
    age: int = 1
    confirmed: bool = False

    def __post_init__(self):
        self.state = _as_state(self.state)
        self.covariance = np.asarray(self.covariance, dtype=float)
        if self.covariance.shape != (6, 6):
            self.covariance = np.eye(6) * 25.0

    @property
    def position(self):
        return self.state[:3]

    @property
    def velocity(self):
        return self.state[3:6]

    def transition_matrix(self):
        f = np.eye(6)
        f[0, 3] = self.dt
        f[1, 4] = self.dt
        f[2, 5] = self.dt
        return f

    def process_noise(self):
        q = self.process_var
        dt = self.dt
        block = np.array(
            [[0.25 * dt**4, 0.5 * dt**3], [0.5 * dt**3, dt**2]],
            dtype=float,
        ) * q
        q_mat = np.zeros((6, 6), dtype=float)
        for pos, vel in ((0, 3), (1, 4), (2, 5)):
            q_mat[np.ix_([pos, vel], [pos, vel])] = block
        return q_mat

    def predict(self):
        f = self.transition_matrix()
        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + self.process_noise()
        self.age += 1

    def update(self, measurement, measurement_covariance=None):
        z = _as_state(measurement)
        r = np.asarray(measurement_covariance, dtype=float) if measurement_covariance is not None else _as_covariance(measurement)
        h = np.eye(6)
        innovation = z - h @ self.state
        s = h @ self.covariance @ h.T + r
        k = self.covariance @ h.T @ np.linalg.pinv(s)
        self.state = self.state + k @ innovation
        self.covariance = (np.eye(6) - k @ h) @ self.covariance
        self.hits += 1
        self.misses = 0
        self.confirmed = self.confirmed or self.hits >= 2

    def mark_missed(self):
        self.misses += 1

    def to_dict(self):
        return {
            "track_id": self.track_id,
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "covariance": self.covariance.copy(),
            "hits": self.hits,
            "misses": self.misses,
            "age": self.age,
            "confirmed": self.confirmed,
        }


@dataclass
class IMMTrack(KalmanTrack):
    """Two-model IMM track blending CV and CA process assumptions."""

    model_probabilities: np.ndarray = field(default_factory=lambda: np.array([0.65, 0.35], dtype=float))

    def predict(self):
        cv_noise = self.process_var
        ca_noise = self.process_var * 6.0
        blended_noise = float(self.model_probabilities[0] * cv_noise + self.model_probabilities[1] * ca_noise)
        original = self.process_var
        self.process_var = blended_noise
        super().predict()
        self.process_var = original

    def update(self, measurement, measurement_covariance=None):
        previous_state = self.state.copy()
        super().update(measurement, measurement_covariance)
        residual_norm = np.linalg.norm(self.state[:3] - previous_state[:3])
        ca_boost = np.clip(residual_norm / 100.0, 0.0, 0.35)
        self.model_probabilities = np.array([0.8 - ca_boost, 0.2 + ca_boost])
        self.model_probabilities /= self.model_probabilities.sum()

    def to_dict(self):
        track = super().to_dict()
        track["model_probabilities"] = self.model_probabilities.copy()
        return track


class MultiTargetTracker:
    """Maintain tracks across timesteps with Hungarian assignment."""

    def __init__(self, dt=0.1, gate_distance=80.0, max_misses=8, use_imm=True):
        self.dt = dt
        self.gate_distance = gate_distance
        self.max_misses = max_misses
        self.use_imm = use_imm
        self.tracks = []
        self._next_id = 1

    def _new_track(self, measurement):
        state = _as_state(measurement)
        cov = _as_covariance(measurement)
        cls = IMMTrack if self.use_imm else KalmanTrack
        track = cls(state=state, covariance=cov, dt=self.dt, track_id=self._next_id)
        self._next_id += 1
        return track

    def step(self, measurements):
        measurements = list(measurements or [])

        for track in self.tracks:
            track.predict()

        if self.tracks and measurements:
            predicted = np.array([track.position for track in self.tracks])
            observed = np.array([_as_state(meas)[:3] for meas in measurements])
            cost = np.linalg.norm(predicted[:, None, :] - observed[None, :, :], axis=2)
            rows, cols = linear_sum_assignment(cost)

            assigned_tracks = set()
            assigned_measurements = set()
            for row, col in zip(rows, cols):
                if cost[row, col] <= self.gate_distance:
                    self.tracks[row].update(measurements[col], _as_covariance(measurements[col]))
                    assigned_tracks.add(row)
                    assigned_measurements.add(col)

            for idx, track in enumerate(self.tracks):
                if idx not in assigned_tracks:
                    track.mark_missed()

            for idx, measurement in enumerate(measurements):
                if idx not in assigned_measurements:
                    self.tracks.append(self._new_track(measurement))
        else:
            for track in self.tracks:
                track.mark_missed()
            for measurement in measurements:
                self.tracks.append(self._new_track(measurement))

        self.tracks = [track for track in self.tracks if track.misses <= self.max_misses]
        return [track.to_dict() for track in self.tracks if track.confirmed or track.hits >= 1]
