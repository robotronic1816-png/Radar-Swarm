import numpy as np
from scipy.optimize import linear_sum_assignment

try:
    import fast_math
except ImportError:
    fast_math = None

def _track_position_and_uncertainty(track):
    """Extract a 3D target position and covariance trace from common track formats."""
    if isinstance(track, dict):
        pos = np.asarray(track.get('position', [0.0, 0.0, 0.0]), dtype=float)
        cov = track.get('covariance', None)
    else:
        pos = np.asarray(track[:3], dtype=float)
        cov = None

    if pos.shape[0] < 3:
        pos = np.pad(pos, (0, 3 - pos.shape[0]))

    if cov is None:
        uncertainty = 0.0
    else:
        covariance = np.asarray(cov, dtype=float)
        uncertainty = float(np.trace(covariance[:3, :3] if covariance.ndim == 2 else covariance))

    return pos[:3], uncertainty


def assign_drones(
    drones,
    tracks,
    gamma=0.01,
    uncertainty_priority=True,
    uncertainty_scale_m=100.0,
    return_cost_matrix=False,
):
    """Assign drones to fused tracks using a global cost matrix and Hungarian optimization.

    The assignment problem is solved as:

        min_A sum_i sum_j A_ij C_ij

    where:
        A_ij in {0, 1}
        sum_j A_ij <= 1
        sum_i A_ij <= 1

    The cost for assigning Drone i to fused Target j is:

        C_ij = ||p_i - x_j||_2 - gamma * s * trace(P_j) / max_k(trace(P_k))

    where:
        - p_i is the drone position.
        - x_j is the fused target position from fusion.py.
        - P_j is the fused track covariance from fusion.py.
        - trace(P_j) is target uncertainty.
        - gamma controls uncertainty priority.
        - s is uncertainty_scale_m, converting normalized uncertainty into meters.

    Because scipy.optimize.linear_sum_assignment minimizes total cost, the
    uncertainty term is subtracted so high-uncertainty tracks receive lower
    assignment cost and are prioritized for coverage. Set
    uncertainty_priority=False to use C_ij = distance + gamma * trace(P_j).
    """

    if not tracks or not drones:
        return None if return_cost_matrix else None

    drone_positions = np.array([d.position for d in drones], dtype=float)

    target_positions = []
    target_uncertainties = []
    for track in tracks:
        pos, uncertainty = _track_position_and_uncertainty(track)
        target_positions.append(pos)
        target_uncertainties.append(uncertainty)

    target_positions = np.vstack(target_positions)
    target_uncertainties = np.array(target_uncertainties, dtype=float)

    if fast_math is not None:
        cost_matrix = fast_math.compute_cost_matrix(
            drone_positions,
            target_positions,
            target_uncertainties,
            gamma,
            uncertainty_priority,
            uncertainty_scale_m,
        )
    else:
        num_drones = len(drone_positions)
        num_targets = len(target_positions)

        if uncertainty_priority and np.max(target_uncertainties) > 0:
            scale = np.max(target_uncertainties)
            uncertainty_term = -gamma * (target_uncertainties / scale) * uncertainty_scale_m
        else:
            uncertainty_term = gamma * target_uncertainties

        cost_matrix = np.zeros((num_drones, num_targets), dtype=float)
        for i in range(num_drones):
            distances = np.linalg.norm(target_positions - drone_positions[i], axis=1)
            cost_matrix[i, :] = distances + uncertainty_term

    row_inds, col_inds = linear_sum_assignment(cost_matrix)

    assigned = set()
    for drone_index, target_index in zip(row_inds, col_inds):
        if drone_index < len(drones) and target_index < len(tracks):
            selected_track = tracks[target_index]
            if isinstance(selected_track, dict):
                drones[drone_index].assign_target(selected_track)
            else:
                drones[drone_index].assign_target(target_positions[target_index].tolist())
            assigned.add(drone_index)

    for drone_index, drone in enumerate(drones):
        if drone_index not in assigned:
            drone.assign_target(None)

    if return_cost_matrix:
        return cost_matrix
    return None
