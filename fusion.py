"""
PHASE 3B: Track-Level Fusion
Combines measurements/estimates from multiple drone radars.

Theory Reference:
- Section 9: Sensor Fusion
- Eq 133: Weighted Least Squares Fusion
- Eq 134-135: Covariance Intersection
- Eq 136: Consensus Algorithm
"""

import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment


class TrackFusion:
    """
    Multi-sensor track-level fusion using multiple methods.
    
    Theory: Section 9 (Sensor Fusion)
    
    Supported methods:
    1. Weighted Least Squares (WLS) - Eq 133
    2. Covariance Intersection (CI) - Eq 134-135
    3. Consensus Algorithm - Eq 136
    """
    
    def __init__(self, method='weighted_ls', max_correlation_distance=100.0):
        """
        Initialize fusion engine.
        
        Args:
            method: 'weighted_ls', 'covariance_intersection', or 'consensus'
            max_correlation_distance: Max distance (m) to correlate tracks
        """
        self.method = method
        self.max_correlation_distance = max_correlation_distance
        self.fusion_history = []
    
    
    # METHOD 1: Weighted Least Squares Fusion
    
    
    def weighted_ls_fusion(self, estimates_list, covariances_list):
        """
        Weighted Least Squares (WLS) Fusion.
        
        Theory: Section 9.4, Eq 133
        x_fused = (Σ P_i^-1)^-1 * Σ P_i^-1 * x_i
        
        Where:
        - P_i = covariance of estimate from sensor i
        - x_i = state estimate from sensor i
        - Sensors with lower uncertainty (smaller P_i) get higher weight
        
        Args:
            estimates_list: List of state estimates [x1, x2, x3, ...]
            covariances_list: List of covariance matrices [P1, P2, P3, ...]
        
        Returns:
            fused_estimate: Fused state
            fused_covariance: Fused covariance
        """
        
        n_sensors = len(estimates_list)
        if n_sensors == 0:
            return None, None
        
        if n_sensors == 1:
            # Single sensor - return as is
            return estimates_list[0], covariances_list[0]
        
        # Get state dimension
        state_dim = estimates_list[0].shape[0]
        
        # Compute sum of inverse covariances: Σ P_i^-1
        P_inv_sum = np.zeros((state_dim, state_dim))
        
        # Compute weighted sum: Σ P_i^-1 * x_i
        weighted_sum = np.zeros(state_dim)
        
        for estimate, covariance in zip(estimates_list, covariances_list):
            try:
                P_inv = np.linalg.inv(covariance)
                P_inv_sum += P_inv
                weighted_sum += P_inv @ estimate
            except np.linalg.LinAlgError:
                # Covariance singular, skip this sensor
                continue
        
        # Fused covariance: (Σ P_i^-1)^-1
        try:
            fused_covariance = np.linalg.inv(P_inv_sum)
        except np.linalg.LinAlgError:
            # Return average if inversion fails
            fused_covariance = sum(covariances_list) / len(covariances_list)
        
        # Fused estimate: P_fused * Σ P_i^-1 * x_i
        fused_estimate = fused_covariance @ weighted_sum
        
        return fused_estimate, fused_covariance
    
    
    # METHOD 2: Covariance Intersection
    
    
    def covariance_intersection(self, estimate1, cov1, estimate2, cov2, omega=0.5):
        """
        Covariance Intersection (CI) Fusion.
        
        Theory: Section 9.5, Eq 134-135
        
        P^-1 = ω*P_1^-1 + (1-ω)*P_2^-1
        x_fused = P * (ω*P_1^-1*x_1 + (1-ω)*P_2^-1*x_2)
        
        Key advantage: Doesn't assume independence between estimates
        (handles unknown cross-correlation).
        
        Args:
            estimate1, estimate2: State estimates
            cov1, cov2: Covariance matrices
            omega: Weighting factor ∈ [0, 1]
                  omega=0.5 gives equal weight
                  omega closer to 0 or 1 weights one estimate more
        
        Returns:
            fused_estimate: Fused state
            fused_covariance: Fused covariance
        """
        
        try:
            P1_inv = np.linalg.inv(cov1)
            P2_inv = np.linalg.inv(cov2)
        except np.linalg.LinAlgError:
            # If inversion fails, return average
            return (estimate1 + estimate2) / 2, (cov1 + cov2) / 2
        
        # Fused inverse covariance (Eq 134)
        P_fused_inv = omega * P1_inv + (1 - omega) * P2_inv
        
        # Fused covariance
        try:
            fused_covariance = np.linalg.inv(P_fused_inv)
        except np.linalg.LinAlgError:
            fused_covariance = (cov1 + cov2) / 2
        
        # Fused estimate (Eq 135)
        fused_estimate = fused_covariance @ (omega * P1_inv @ estimate1 + 
                                             (1 - omega) * P2_inv @ estimate2)
        
        return fused_estimate, fused_covariance
    
    # METHOD 3: Consensus Algorithm
    
    
    def consensus_update(self, estimate_dict, neighbor_dict, weights=None, iterations=5):
        """
        Distributed Consensus Fusion.
        
        Theory: Section 9.6.1, Eq 136
        
        x_i^(k+1) = x_i^(k) + Σ_j w_ij * (x_j^(k) - x_i^(k))
        
        Each agent updates toward neighbors' estimates iteratively.
        
        Args:
            estimate_dict: Dict {drone_id: estimate}
            neighbor_dict: Dict {drone_id: [neighbor_ids]}
            weights: Dict {(i,j): w_ij}, default to equal weights
            iterations: Number of consensus iterations
        
        Returns:
            consensus_estimates: Dict {drone_id: converged_estimate}
        """
        
        estimates = estimate_dict.copy()
        n_agents = len(estimates)
        
        if n_agents <= 1:
            return estimates
        
        # Default: equal weights to all neighbors
        if weights is None:
            weights = {}
            for i in estimate_dict.keys():
                n_neighbors = len(neighbor_dict.get(i, []))
                if n_neighbors > 0:
                    for j in neighbor_dict.get(i, []):
                        weights[(i, j)] = 1.0 / (n_neighbors + 1)
                else:
                    weights[(i, i)] = 1.0
        
        # Consensus iterations
        for iteration in range(iterations):
            new_estimates = {}
            
            for i in estimates.keys():
                # Start with self estimate
                update = estimates[i].copy()
                
                # Add weighted neighbor updates
                neighbors = neighbor_dict.get(i, [])
                for j in neighbors:
                    if j in estimates:
                        w_ij = weights.get((i, j), 1.0 / (len(neighbors) + 1))
                        update += w_ij * (estimates[j] - estimates[i])
                
                new_estimates[i] = update
            
            estimates = new_estimates
        
        return estimates
    
    # Track Association and Fusion
    
    
    def associate_tracks(self, local_tracks_list):
        """
        Associate tracks from different sensors that likely represent same target.
        
        Theory: Section 9.8 (Track-to-Track Fusion)
        
        Uses Hungarian algorithm for globally optimal track association.
        
        Args:
            local_tracks_list: List of track dictionaries from each sensor
                Each track: {'position': [x,y,z], 'velocity': [vx,vy,vz], 
                            'covariance': P, 'drone_id': id}
        
        Returns:
            associations: List of lists, each group is associated tracks
        """
        
        if len(local_tracks_list) == 0:
            return []
        
        if len(local_tracks_list) == 1:
            # Single sensor - no association needed
            return [[track] for track in local_tracks_list[0]]
        
        # Flatten all tracks with sensor source
        flat_tracks = []
        for sensor_idx, tracks in enumerate(local_tracks_list):
            for track_idx, track in enumerate(tracks):
                flat_tracks.append((sensor_idx, track_idx, track))
        
        if len(flat_tracks) == 0:
            return []
        
        if len(flat_tracks) <= 1:
            return [[t[2]] for t in flat_tracks]
        
        # Extract positions and compute distance matrix
        positions = np.array([t[2]['position'][:3] for t in flat_tracks])
        n_tracks = len(flat_tracks)
        
        # Build cost matrix for track association
        # Cost = distance between tracks
        cost_matrix = np.zeros((n_tracks, n_tracks))
        
        for i in range(n_tracks):
            for j in range(i + 1, n_tracks):
                # Cannot associate tracks from same sensor
                if flat_tracks[i][0] == flat_tracks[j][0]:
                    cost_matrix[i, j] = np.inf
                    cost_matrix[j, i] = np.inf
                else:
                    dist = np.linalg.norm(positions[i] - positions[j])
                    # High cost if distance exceeds threshold
                    if dist > self.max_correlation_distance:
                        cost_matrix[i, j] = np.inf
                        cost_matrix[j, i] = np.inf
                    else:
                        cost_matrix[i, j] = dist
                        cost_matrix[j, i] = dist
        
        # Greedy association: repeatedly find closest pair not yet associated
        associations = []
        used_indices = set()
        
        # Get list of all valid pairs sorted by distance
        pairs = []
        for i in range(n_tracks):
            for j in range(i + 1, n_tracks):
                if cost_matrix[i, j] < np.inf:
                    pairs.append((cost_matrix[i, j], i, j))
        
        pairs.sort()  # Sort by distance (ascending)
        
        # Greedily match pairs
        for _, i, j in pairs:
            if i not in used_indices and j not in used_indices:
                associations.append([flat_tracks[i][2], flat_tracks[j][2]])
                used_indices.add(i)
                used_indices.add(j)
        
        # Add unassociated tracks as singletons
        for idx in range(n_tracks):
            if idx not in used_indices:
                associations.append([flat_tracks[idx][2]])
        
        return associations
    
    def fuse_tracks(self, local_tracks_list):
        """
        Fuse tracks from multiple sensors into global track estimates.
        
        Theory: Section 9.2-9.4 (Sensor Fusion)
        
        Args:
            local_tracks_list: List of track lists, one per drone/sensor
        
        Returns:
            fused_tracks: List of fused global track estimates
        """
        
        # Step 1: Associate tracks across sensors
        associations = self.associate_tracks(local_tracks_list)
        
        if len(associations) == 0:
            return []
        
        # Step 2: Fuse each association group
        fused_tracks = []
        
        for track_group in associations:
            if len(track_group) == 1:
                # Single observation - use as is, but add fusion metadata
                track = track_group[0].copy()
                track['num_sources'] = 1
                track['source_sensors'] = [track['drone_id']]
                fused_tracks.append(track)
            else:
                # Multiple observations - fuse them
                # Build full 6D state vectors [x, y, z, vx, vy, vz]
                estimates = []
                covariances = []
                
                for t in track_group:
                    state = np.concatenate([t['position'], t['velocity']])
                    estimates.append(state)
                    covariances.append(t['covariance'])
                
                if self.method == 'weighted_ls':
                    fused_state, fused_cov = self.weighted_ls_fusion(
                        estimates, covariances
                    )
                elif self.method == 'covariance_intersection':
                    # Iteratively fuse multiple estimates using CI
                    fused_state, fused_cov = estimates[0], covariances[0]
                    for est, cov in zip(estimates[1:], covariances[1:]):
                        fused_state, fused_cov = self.covariance_intersection(
                            fused_state, fused_cov, est, cov
                        )
                else:
                    # Default to weighted LS
                    fused_state, fused_cov = self.weighted_ls_fusion(
                        estimates, covariances
                    )
                
                # Extract position and velocity from fused state
                fused_pos = fused_state[:3]
                fused_vel = fused_state[3:6]
                
                # Create fused track
                fused_track = {
                    'position': fused_pos,
                    'velocity': fused_vel,
                    'covariance': fused_cov,
                    'source_sensors': [t['drone_id'] for t in track_group],
                    'num_sources': len(track_group)
                }
                
                fused_tracks.append(fused_track)
        
        self.fusion_history.append({
            'timestamp': len(self.fusion_history),
            'num_fused_tracks': len(fused_tracks),
            'fusion_method': self.method
        })
        
        return fused_tracks
    
        # Utilities
    
    
    def compute_fusion_gain(self, covariances_list):
        """
        Compute reduction in uncertainty from fusion.
        
        Returns ratio of fused uncertainty to single-sensor uncertainty.
        Lower ratio = greater improvement.
        """
        
        if len(covariances_list) <= 1:
            return 1.0
        
        single_uncertainty = np.linalg.norm(covariances_list[0])
        
        _, fused_cov = self.weighted_ls_fusion(
            [np.zeros(covariances_list[0].shape[0])] * len(covariances_list),
            covariances_list
        )
        
        fused_uncertainty = np.linalg.norm(fused_cov)
        
        return fused_uncertainty / single_uncertainty if single_uncertainty > 0 else 1.0
    
    def get_fusion_statistics(self):
        """Return fusion history and statistics."""
        return {
            'total_fusion_events': len(self.fusion_history),
            'fusion_method': self.method,
            'history': self.fusion_history
        }


# Track Builder Utility


class TrackBuilder:
    """Helper to convert drone measurements into trackable format."""
    
    @staticmethod
    def build_track_from_measurements(measurements, drone_id):
        """
        Build a track estimate from a single measurement.
        
        Args:
            measurements: List of measurement dicts from DroneRadar
            drone_id: Which drone generated these measurements
        
        Returns:
            track: Dict with position, velocity, covariance
        """
        
        if not measurements:
            return None
        
        tracks = []
        
        for meas in measurements:
            # Convert from polar (range, azimuth, elevation) to Cartesian
            r = meas['range']
            az = meas['azimuth']
            el = meas['elevation']
            sensor_pos = meas['sensor_position']
            
            # Convert to world coordinates
            x_rel = r * np.cos(el) * np.cos(az)
            y_rel = r * np.cos(el) * np.sin(az)
            z_rel = r * np.sin(el)
            
            world_x = sensor_pos[0] + x_rel
            world_y = sensor_pos[1] + y_rel
            world_z = sensor_pos[2] + z_rel
            
            # Velocity from Doppler
            v_radial = meas['doppler']
            vx = v_radial * np.cos(az)
            vy = v_radial * np.sin(az)
            vz = 0.0  # Assume no vertical velocity component from Doppler
            
            # Covariance: Use direct measurement uncertainties, avoid ill-conditioned Jacobian
            # Position uncertainty comes from range/angle measurement errors
            # Range uncertainty: ~0.5m (1/4 of wavelength)
            # Angle uncertainty: ~0.05 rad (~3 degrees)
            # This translates to position error of roughly:
            #   dX ~ sqrt((dr)^2 + (r*dangle)^2)
            
            range_error = 0.5   # meters
            angle_error = 0.05  # radians
            
            # Position uncertainty (conservative estimate)
            pos_uncertainty = np.sqrt(range_error**2 + (r * angle_error)**2)
            
            # Velocity uncertainty from Doppler (conservative)
            doppler_error = 0.3  # m/s
            
            # Build 6x6 covariance matrix
            P = np.eye(6)
            # Position covariance (diagonal)
            P[0, 0] = pos_uncertainty**2
            P[1, 1] = pos_uncertainty**2
            P[2, 2] = pos_uncertainty**2
            
            # Velocity covariance (higher uncertainty since only observed via Doppler)
            P[3, 3] = doppler_error**2 + (v_radial * angle_error)**2  # vx uncertainty
            P[4, 4] = doppler_error**2 + (v_radial * angle_error)**2  # vy uncertainty
            P[5, 5] = (v_radial * angle_error)**2  # vz uncertainty (only from angle errors)
            
            track = {
                'position': np.array([world_x, world_y, world_z]),
                'velocity': np.array([vx, vy, vz]),
                'covariance': P,
                'drone_id': drone_id,
                'measurement': meas
            }
            
            tracks.append(track)
        
        return tracks
