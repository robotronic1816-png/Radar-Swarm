"""Adaptive swarm geometry utilities for GDOP reduction.

Phase 5A uses target-relative radar Jacobians to estimate how informative the
current drone formation is. The Fisher Information Matrix is:

    FIM = H.T R^-1 H

where H stacks each drone radar measurement Jacobian and R is the measurement
noise covariance. The scalar geometry score is:

    GDOP = sqrt(trace(FIM^-1))

Lower GDOP means the swarm geometry provides better triangulation accuracy.
"""

import numpy as np

from fmcw_radar import DroneRadar


DEFAULT_MEASUREMENT_STD = np.array([0.5, 0.05, 0.05, 0.3], dtype=float)


def _as_vector3(value, default=None):
    if default is None:
        default = np.zeros(3, dtype=float)
    if value is None:
        return np.asarray(default, dtype=float).copy()
    arr = np.asarray(value, dtype=float)
    if arr.shape[0] < 3:
        arr = np.pad(arr, (0, 3 - arr.shape[0]))
    return arr[:3]


def target_state_vector(target):
    """Normalize target objects, track dictionaries, or arrays to a 6D state."""
    if isinstance(target, dict):
        position = _as_vector3(target.get("position"))
        velocity = _as_vector3(target.get("velocity"))
    elif hasattr(target, "position"):
        position = _as_vector3(target.position)
        velocity = _as_vector3(getattr(target, "velocity", None))
    else:
        state = np.asarray(target, dtype=float)
        if state.shape[0] < 6:
            state = np.pad(state, (0, 6 - state.shape[0]))
        return state[:6]

    return np.concatenate([position, velocity])


def drone_position(drone):
    """Return a drone object's position or a raw 3D position vector."""
    return _as_vector3(getattr(drone, "position", drone))


def drone_velocity(drone):
    """Return a drone object's velocity or zero for raw position vectors."""
    return _as_vector3(getattr(drone, "velocity", None))


def active_drone_positions(drones):
    """Extract positions from drones that are active for geometry planning."""
    positions = []
    for drone in drones:
        if getattr(drone, "active", True):
            positions.append(drone_position(drone))
    return positions


def measurement_noise_covariance(measurement_std=None):
    """Build R for [range, azimuth, elevation, radial_velocity]."""
    std = np.asarray(measurement_std if measurement_std is not None else DEFAULT_MEASUREMENT_STD, dtype=float)
    if std.shape[0] != 4:
        raise ValueError("measurement_std must contain 4 entries: range, azimuth, elevation, doppler")
    return np.diag(std**2)


def radar_jacobian_for_geometry(drone_pos, target_state, drone_vel=None):
    """Compute the 4x6 radar Jacobian at a candidate drone position."""
    radar = DroneRadar(
        drone_id=-1,
        position=_as_vector3(drone_pos),
        velocity=_as_vector3(drone_vel),
        orientation=np.zeros(3, dtype=float),
    )
    return radar.get_measurement_jacobian(np.asarray(target_state, dtype=float))


def build_geometry_jacobian(drones_or_positions, target, drone_velocities=None):
    """Stack each active drone's radar measurement Jacobian into one H matrix."""
    target_state = target_state_vector(target)
    rows = []

    for idx, drone in enumerate(drones_or_positions):
        if getattr(drone, "active", True):
            pos = drone_position(drone)
            if drone_velocities is None:
                vel = drone_velocity(drone)
            else:
                vel = _as_vector3(drone_velocities[idx])
            rows.append(radar_jacobian_for_geometry(pos, target_state, vel))

    if not rows:
        return np.empty((0, 6), dtype=float)
    return np.vstack(rows)


def fisher_information_matrix(drones_or_positions, target, measurement_std=None, regularization=1e-6):
    """Calculate the Fisher Information Matrix for a target and swarm geometry.

    Args:
        drones_or_positions: Drone objects or raw 3D drone positions.
        target: Target object, fused track dict, or 6D state array.
        measurement_std: Standard deviations for radar measurements
            [range, azimuth, elevation, radial_velocity].
        regularization: Diagonal loading to keep the FIM invertible when the
            swarm has too few drones or poor geometry.

    Returns:
        6x6 Fisher Information Matrix over target state [x, y, z, vx, vy, vz].
    """
    h_matrix = build_geometry_jacobian(drones_or_positions, target)
    fim = np.eye(6, dtype=float) * regularization

    if h_matrix.size == 0:
        return fim

    r_inv = np.linalg.pinv(measurement_noise_covariance(measurement_std))
    for start in range(0, h_matrix.shape[0], 4):
        h_i = h_matrix[start:start + 4]
        fim += h_i.T @ r_inv @ h_i

    return fim


def gdop(drones_or_positions, target, measurement_std=None, regularization=1e-6):
    """Calculate GDOP = sqrt(trace(FIM^-1)) for the current 3D geometry."""
    fim = fisher_information_matrix(
        drones_or_positions,
        target,
        measurement_std=measurement_std,
        regularization=regularization,
    )
    covariance_bound = np.linalg.pinv(fim)
    return float(np.sqrt(max(np.trace(covariance_bound), 0.0)))


def repositioning_vector_for_drone(
    drones_or_positions,
    target,
    drone_index,
    step_size=10.0,
    learning_rate=50.0,
    max_vector_norm=25.0,
    measurement_std=None,
    min_target_distance=20.0,
):
    """Return a 3D vector that moves one drone toward lower swarm GDOP.

    The gradient is estimated with central finite differences in Cartesian
    coordinates. The output is not applied automatically; pass it to your
    controller or guidance layer as a desired geometry correction.
    """
    positions = np.array([drone_position(d) for d in drones_or_positions], dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("drones_or_positions must provide one or more 3D positions")
    if drone_index < 0 or drone_index >= len(positions):
        raise IndexError("drone_index out of range")

    target_state = target_state_vector(target)
    target_pos = target_state[:3]
    gradient = np.zeros(3, dtype=float)

    for axis in range(3):
        offset = np.zeros(3, dtype=float)
        offset[axis] = step_size

        plus_positions = positions.copy()
        minus_positions = positions.copy()
        plus_positions[drone_index] += offset
        minus_positions[drone_index] -= offset

        gdop_plus = gdop(plus_positions, target_state, measurement_std=measurement_std)
        gdop_minus = gdop(minus_positions, target_state, measurement_std=measurement_std)
        gradient[axis] = (gdop_plus - gdop_minus) / (2.0 * step_size)

    correction = -learning_rate * gradient

    candidate = positions[drone_index] + correction
    rel_to_target = candidate - target_pos
    distance_to_target = np.linalg.norm(rel_to_target)
    if 0.0 < distance_to_target < min_target_distance:
        candidate = target_pos + rel_to_target / distance_to_target * min_target_distance
        correction = candidate - positions[drone_index]

    norm = np.linalg.norm(correction)
    if norm > max_vector_norm:
        correction = correction / norm * max_vector_norm

    return correction


def gdop_gradient_descent_step(
    drones_or_positions,
    target,
    step_size=10.0,
    learning_rate=50.0,
    max_vector_norm=25.0,
    measurement_std=None,
):
    """Return one repositioning vector per drone for adaptive geometry control."""
    return [
        repositioning_vector_for_drone(
            drones_or_positions,
            target,
            drone_index=i,
            step_size=step_size,
            learning_rate=learning_rate,
            max_vector_norm=max_vector_norm,
            measurement_std=measurement_std,
        )
        for i in range(len(drones_or_positions))
    ]
