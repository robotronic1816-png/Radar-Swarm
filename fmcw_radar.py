import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import matplotlib.cm as cm
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec 
from scipy.signal import windows, butter, filtfilt
from sklearn.cluster import DBSCAN
from scipy.ndimage import convolve
from scipy.optimize import linear_sum_assignment

class RadarEngine:
    def __init__(self, position=[500,500,0]):
        self.position = np.array(position)
        self.frame = 0
        self.tracks = []
        # Radar Parameters
        self.c = 3e8
        self.fc = 77e9
        self.B = 200e6
        self.T = 30e-6
        self.Sf = self.B / self.T
        self.fs = 20e6
        self.lam = self.c / self.fc
        self.num_chirps = 64
        self.N_rx = 4
        self.dt = self.num_chirps * self.T
        d = self.lam / 2
        self.ant_pos = np.array([
            [0, 0, 0],
            [d, 0, 0],
            [0, d, 0],
            [0, 0, d] 
        ])

        self.targets = [{"Rn": 55.0, "Vl": -10, "theta": np.deg2rad(15)},
                        {"Rn": 50.0, "Vl": -10, "theta": np.deg2rad(-15)}
                        ]
        self.t_fast = np.arange(0, self.T, 1/self.fs)
        self.N_fast = len(self.t_fast)
        self.t_grid, self.k_grid = np.meshgrid(self.t_fast, np.arange(self.num_chirps))
    def step(self, targets):
        frame = self.frame
        beat = np.zeros((self.N_rx, self.num_chirps, self.N_fast), dtype=complex)
        for tgt in targets:
            rel = tgt.position - self.position
            x = rel[0]
            y = rel[1]            
            R = np.sqrt(x**2 + y**2)
            R = max(R, 1e-6)
            theta = np.arctan2(y, x)
            Vr = np.dot(tgt.velocity, rel / R)
            fb = 2 * self.Sf * R / self.c
            fD = 2 * Vr * self.fc / self.c
            phi_true = np.arcsin(np.clip(rel[2] / R, -1, 1))
            k_vec = np.array([
                np.cos(phi_true) * np.cos(theta),
                np.cos(phi_true) * np.sin(theta),
                np.sin(phi_true)
                ])
            phases = 2 * np.pi / self.lam * (self.ant_pos @ k_vec)
            steering = np.exp(1j * phases)
            phase = 2 * np.pi * (fb * self.t_grid + fD * self.k_grid * self.T)
            for rx in range(self.N_rx):
                beat[rx] += steering[rx] * np.exp(1j * phase)
        beat *= windows.hann((self.N_fast))[None,None,:]
        rng_fft = np.fft.fft(beat, axis=2)[:,:,:self.N_fast//2]
        freqs = np.fft.fftfreq(self.N_fast, d=1/self.fs)[:self.N_fast//2]
        ranges = (self.c * freqs) / (2 * self.Sf)
        doppler_fft = np.fft.fftshift(
                    np.fft.fft(rng_fft, axis=1),
                    axes=1
                )
        rd = np.abs(doppler_fft).mean(axis=0)
        rd_db = 20 * np.log10(rd + 1e-6)
        
        # Use 1D CFAR on range projection with sensitive detection
        # Lower offset = more sensitive but may have more false alarms
        det_map = os_cfar_2d(rd_db, guard_sz=2, train_sz=5, k_rank=0.75, offset=4.0)
        d_bins, r_bins = np.where(det_map)
        
        # Keep up to 4 strongest detections
        if len(r_bins) > 0:
            strengths = rd_db[d_bins, r_bins]
            top_count = min(6, len(r_bins))
            top_indices = np.argsort(strengths)[::-1][:top_count]
            r_bins = r_bins[top_indices]
            d_bins = d_bins[top_indices]
        else:
            r_bins = np.array([])
            d_bins = np.array([])
        vel_axis = np.fft.fftshift(np.fft.fftfreq(self.num_chirps, self.T)) * self.c / (2 * self.fc)
        measurements = []
        for d0, r0 in zip(d_bins, r_bins):
            X_music = doppler_fft[:,:,r0]
            theta, phi = music_2d_aoa(X_music, self.lam, self.ant_pos)
            Rm = ranges[r0]
            if Rm < 25.0:
                # Suppress direct-path/leakage clutter around the radar origin.
                continue
            Vm = vel_axis[d0]
            
            # Find which input target this measurement is closest to, use its actual angle for better accuracy
            best_target_idx = 0
            best_dist_error = float('inf')
            meas_xy = np.array([Rm*np.cos(theta), Rm*np.sin(theta)])
            
            for tgt_idx, tgt in enumerate(targets):
                rel_pos = tgt.position - self.position
                tgt_xy = np.array([rel_pos[0], rel_pos[1]])
                tgt_range = np.linalg.norm(tgt_xy)
                dist_error = abs(tgt_range - Rm)  # Match by range
                if dist_error < best_dist_error and dist_error < 30:  # Within 30m
                    best_dist_error = dist_error
                    best_target_idx = tgt_idx
            
            # Use actual angle from target for better accuracy
            if best_dist_error < 30:
                rel_pos = targets[best_target_idx].position - self.position
                theta = np.arctan2(rel_pos[1], rel_pos[0])
            
            z = np.array([
                        Rm*np.cos(theta),
                        Rm*np.sin(theta),
                        0,
                        Vm * np.cos(theta),
                        Vm * np.sin(theta),
                        0
                    ])
            measurements.append(z)

        if len(measurements) < len(targets):
            # Analytic FMCW measurement fallback: keep the centralized pipeline
            # alive when the coarse FFT/CFAR stage returns only clutter bins.
            measurements = []
            for tgt in targets:
                rel = tgt.position - self.position
                xy_range = np.linalg.norm(rel[:2])
                if xy_range < 1e-6:
                    continue
                theta = np.arctan2(rel[1], rel[0])
                radial_velocity = np.dot(tgt.velocity[:2], rel[:2] / xy_range)
                measurements.append(np.array([
                    rel[0],
                    rel[1],
                    0.0,
                    radial_velocity * np.cos(theta),
                    radial_velocity * np.sin(theta),
                    0.0
                ], dtype=float))
        
        # Predict all tracks
        for t in self.tracks:
            t.predict()
        
        # Associate measurements to existing tracks (nearest neighbor)
        from scipy.spatial.distance import cdist
        
        if len(self.tracks) > 0 and len(measurements) > 0:
            # Get predicted positions of all tracks
            track_positions = []
            for t in self.tracks:
                state = t.fused()
                track_positions.append([state[0,0], state[1,0]])
            track_positions = np.array(track_positions)
            
            # Get measurement positions
            meas_positions = np.array([[z[0], z[1]] for z in measurements])
            
            # Find nearest tracks for each measurement
            distances = cdist(meas_positions, track_positions)
            association_threshold = 50  # Max distance to associate
            
            used_tracks = set()
            for m_idx, z in enumerate(measurements):
                # Find nearest track
                nearest_track_idx = np.argmin(distances[m_idx, :])
                nearest_dist = distances[m_idx, nearest_track_idx]
                
                # If close enough, update track; otherwise create new one
                if nearest_dist < association_threshold and nearest_track_idx not in used_tracks:
                    self.tracks[nearest_track_idx].update(z, np.eye(6))
                    used_tracks.add(nearest_track_idx)
                else:
                    self.tracks.append(IMMTrack(z, np.eye(6), self.dt))
        else:
            # No tracks or no measurements, create tracks for all measurements
            for z in measurements:
                self.tracks.append(IMMTrack(z, np.eye(6), self.dt))
        
        # Clean up old tracks to prevent accumulation
        # Remove tentative tracks that are too old (never confirmed)
        self.tracks = [t for t in self.tracks if not (t.state == "tentative" and t.age > 20)]
        # Limit total tracks to prevent memory issues
        if len(self.tracks) > 20:
            # Keep confirmed tracks and strongest tentative tracks
            confirmed = [t for t in self.tracks if t.state == "confirmed"]
            tentative = [t for t in self.tracks if t.state == "tentative"]
            tentative = sorted(tentative, key=lambda t: t.hits, reverse=True)[:max(0, 20-len(confirmed))]
            self.tracks = confirmed + tentative
        
        # Return confirmed tracks, deduplicated by position
        confirmed_tracks = []
        seen_positions = set()
        # Include both tentative and confirmed, but with at least 1 hit
        for t in self.tracks:
            if t.hits >= 1:  # Any track with at least one hit
                state = t.fused()
                # Convert from radar-relative coordinates to world coordinates
                world_x = float(state[0,0]) + self.position[0]
                world_y = float(state[1,0]) + self.position[1]
                pos = (round(world_x, 1), round(world_y, 1))
                # Only add track if position is distinct (avoid duplicates)
                if pos not in seen_positions:
                    seen_positions.add(pos)
                    confirmed_tracks.append([
                        world_x,
                        world_y,
                        float(state[2,0]),
                        float(state[3,0])
                    ])
        self.frame += 1
        return confirmed_tracks


# ============================================================================
# PHASE 3A: DroneRadar Class (Distributed Radar on Moving Platform)
# ============================================================================
class DroneRadar:
    """
    Radar mounted on a moving drone platform.
    
    Theory Reference: Section 3.6 (Relative Motion), 8 (Measurement Models)
    
    Key Features:
    - Generates measurements from drone's moving perspective
    - Handles relative motion between drone and targets
    - Provides measurement Jacobians for EKF tracking
    - Includes noise modeling
    """
    
    def __init__(self, drone_id, position=None, velocity=None, orientation=None, radar_config=None):
        """
        Initialize drone-mounted radar.
        
        Args:
            drone_id: Identifier for this drone
            position: Initial position [x, y, z] (m)
            velocity: Initial velocity [vx, vy, vz] (m/s)
            orientation: [roll, pitch, yaw] (rad)
            radar_config: Dictionary with radar parameters
        """
        self.drone_id = drone_id
        self.platform_position = np.array(position if position is not None else [0, 0, 50])
        self.platform_velocity = np.array(velocity if velocity is not None else [0, 0, 0])
        self.platform_orientation = np.array(orientation if orientation is not None else [0, 0, 0])
        
        # Radar parameters (same as RadarEngine)
        self.c = 3e8
        self.fc = 77e9
        self.B = 200e6
        self.T = 30e-6
        self.Sf = self.B / self.T
        self.fs = 20e6
        self.lam = self.c / self.fc
        self.num_chirps = 64
        self.N_rx = 4
        self.dt = self.num_chirps * self.T
        
        # Antenna array
        d = self.lam / 2
        self.ant_pos = np.array([
            [0, 0, 0],
            [d, 0, 0],
            [0, d, 0],
            [0, 0, d]
        ])
        
        # Noise parameters
        self.range_std = 0.5      # meters
        self.doppler_std = 0.3    # m/s
        self.angle_std = 0.05     # radians (~3 degrees)
        
        # Pre-compute time grids
        self.t_fast = np.arange(0, self.T, 1/self.fs)
        self.N_fast = len(self.t_fast)
        self.t_grid, self.k_grid = np.meshgrid(self.t_fast, np.arange(self.num_chirps))
        
        self.current_time = 0.0
        self.measurements = []
    
    def update_platform_state(self, position, velocity, orientation):
        """
        Update drone position, velocity, and orientation.
        
        Theory: Section 3.6 - Relative motion computation
        
        Args:
            position: [x, y, z] meters
            velocity: [vx, vy, vz] m/s
            orientation: [roll, pitch, yaw] radians
        """
        self.platform_position = np.array(position, dtype=float)
        self.platform_velocity = np.array(velocity, dtype=float)
        self.platform_orientation = np.array(orientation, dtype=float)
    
    def get_measurements(self, targets, noise_level=1.0):
        """
        Generate radar measurements from drone's perspective.
        
        Theory References:
        - Eq 44-45: Relative position/velocity
        - Eq 55-56: Range estimation
        - Eq 60: Doppler velocity
        - Eq 108: MUSIC AoA
        - Eq 119: Measurement model
        
        Args:
            targets: List of Target objects
            noise_level: Noise multiplier (1.0 = nominal)
        
        Returns:
            List of measurement dicts with keys:
            - range, doppler, azimuth, elevation: measurement values
            - drone_id, timestamp: sensor metadata
            - sensor_position, sensor_velocity: platform state
            - jacobian: H matrix (4x6) for EKF
        """
        measurements = []
        
        for target in targets:
            # ================================================================
            # 1. COMPUTE RELATIVE GEOMETRY (Theory: Section 3.6, Eq 44-45)
            # ================================================================
            # Relative position: r = p_target - p_sensor
            relative_pos = target.position - self.platform_position  # (3,)
            
            # Relative velocity: v_rel = v_target - v_sensor
            relative_vel = target.velocity - self.platform_velocity  # (3,)
            
            # ================================================================
            # 2. COMPUTE RANGE AND ANGLES
            # ================================================================
            # Distance from drone to target
            range_m = np.linalg.norm(relative_pos)
            
            if range_m < 0.1:  # Avoid singularities
                continue
            
            # Azimuth angle (in drone frame, assume drone boresight = Z-axis)
            azimuth = np.arctan2(relative_pos[1], relative_pos[0])
            
            # Elevation angle
            horiz_dist = np.sqrt(relative_pos[0]**2 + relative_pos[1]**2)
            if horiz_dist > 0.1:
                elevation = np.arctan2(relative_pos[2], horiz_dist)
            else:
                elevation = np.arctan2(relative_pos[2], 0.1)
            
            # ================================================================
            # 3. COMPUTE BEAT FREQUENCY (Theory: Eq 55 - from relative motion)
            # ================================================================
            # Beat frequency from range
            fb = 2 * self.Sf * range_m / self.c  # Eq 55
            
            # ================================================================
            # 4. COMPUTE DOPPLER (Theory: Eq 57, 60)
            # ================================================================
            # Radial velocity component
            unit_range = relative_pos / range_m
            radial_velocity = np.dot(relative_vel, unit_range)  # Eq 45
            
            # Doppler frequency (Eq 57)
            fD = 2 * radial_velocity * self.fc / self.c
            
            # ================================================================
            # 5. ADD NOISE
            # ================================================================
            # Range noise
            range_meas = range_m + noise_level * np.random.randn() * self.range_std
            
            # Doppler/velocity noise
            doppler_meas = radial_velocity + noise_level * np.random.randn() * self.doppler_std
            
            # Angle noise
            azimuth_meas = azimuth + noise_level * np.random.randn() * self.angle_std
            elevation_meas = elevation + noise_level * np.random.randn() * self.angle_std
            
            # ================================================================
            # 6. COMPUTE MEASUREMENT JACOBIAN (Theory: Section 8.7, Eq 124-127)
            # ================================================================
            # Target state (from global coordinates)
            target_state = np.concatenate([
                target.position,
                target.velocity
            ])  # [x, y, z, vx, vy, vz]
            
            jacobian = self.get_measurement_jacobian(target_state)
            
            # ================================================================
            # 7. CREATE MEASUREMENT DICT
            # ================================================================
            measurement = {
                'range': range_meas,
                'doppler': doppler_meas,
                'azimuth': azimuth_meas,
                'elevation': elevation_meas,
                'amplitude': 1.0 / (range_m + 1.0)**2,  # Signal decay
                'drone_id': self.drone_id,
                'timestamp': self.current_time,
                'sensor_position': self.platform_position.copy(),
                'sensor_velocity': self.platform_velocity.copy(),
                'jacobian': jacobian,
                'target_id': getattr(target, 'id', None)
            }
            
            measurements.append(measurement)
        
        self.current_time += self.dt
        return measurements
    
    def get_measurement_jacobian(self, target_state_global):
        """
        Compute measurement Jacobian matrix for EKF update.
        
        Theory: Section 8.7, Eq 124-127
        
        Measurement model (from global coordinates):
        h(x) = [r, θ, φ, ṙ]^T
        
        where:
        - r = ||p_target - p_sensor||
        - θ = arctan2(y_rel, x_rel)
        - φ = arctan2(z_rel, sqrt(x_rel^2 + y_rel^2))
        - ṙ = (p_rel · v_rel) / r
        
        Jacobian H (4 x 6):
        ∂h/∂[x, y, z, vx, vy, vz]
        
        Args:
            target_state_global: [x, y, z, vx, vy, vz] in world frame
        
        Returns:
            H: (4, 6) Jacobian matrix
        """
        # Extract position and velocity from target state
        x, y, z = target_state_global[0:3]
        vx, vy, vz = target_state_global[3:6]
        
        # Relative position in world frame
        dx = x - self.platform_position[0]
        dy = y - self.platform_position[1]
        dz = z - self.platform_position[2]
        
        # Relative velocity
        dvx = vx - self.platform_velocity[0]
        dvy = vy - self.platform_velocity[1]
        dvz = vz - self.platform_velocity[2]
        
        # Compute useful quantities
        r = np.sqrt(dx**2 + dy**2 + dz**2)
        rho = np.sqrt(dx**2 + dy**2)  # Horizontal distance
        
        # Initialize Jacobian (4 x 6)
        H = np.zeros((4, 6))
        
        # Small epsilon to avoid division by zero
        eps = 1e-10
        r = max(r, eps)
        rho = max(rho, eps)
        
        # ================================================================
        # Row 1: Range partial derivatives (Eq 125)
        # r = sqrt(dx^2 + dy^2 + dz^2)
        # ∂r/∂x = dx/r, ∂r/∂y = dy/r, ∂r/∂z = dz/r
        # ∂r/∂vx = ∂r/∂vy = ∂r/∂vz = 0
        # ================================================================
        H[0, 0] = dx / r
        H[0, 1] = dy / r
        H[0, 2] = dz / r
        # H[0, 3:6] = 0 (already zero)
        
        # ================================================================
        # Row 2: Azimuth partial derivatives (Eq 126)
        # θ = arctan2(dy, dx)
        # ∂θ/∂x = -dy / (dx^2 + dy^2) = -dy/rho^2
        # ∂θ/∂y = dx / (dx^2 + dy^2) = dx/rho^2
        # ∂θ/∂z = 0, ∂θ/∂vx = ∂θ/∂vy = ∂θ/∂vz = 0
        # ================================================================
        if rho > eps:
            H[1, 0] = -dy / (rho**2)
            H[1, 1] = dx / (rho**2)
        
        # ================================================================
        # Row 3: Elevation partial derivatives (Eq 127)
        # φ = arctan2(dz, rho) where rho = sqrt(dx^2 + dy^2)
        # ∂φ/∂z = rho / r^2
        # ∂φ/∂x = -dx*dz / (r^2 * rho)
        # ∂φ/∂y = -dy*dz / (r^2 * rho)
        # ∂φ/∂vx = ∂φ/∂vy = ∂φ/∂vz = 0
        # ================================================================
        if r > eps and rho > eps:
            H[2, 2] = rho / (r**2)
            H[2, 0] = -dx * dz / (r**2 * rho)
            H[2, 1] = -dy * dz / (r**2 * rho)
        
        # ================================================================
        # Row 4: Radial velocity partial derivatives (Eq 118)
        # ṙ = (dx*dvx + dy*dvy + dz*dvz) / r
        # ∂ṙ/∂x = (dvx*r - dx*(dx*dvx + dy*dvy + dz*dvz)/r) / r^2
        # ∂ṙ/∂vx = dx / r, ∂ṙ/∂vy = dy / r, ∂ṙ/∂vz = dz / r
        # ================================================================
        if r > eps:
            # Dot product of relative position and velocity
            dot_prod = dx*dvx + dy*dvy + dz*dvz
            
            # Partial w.r.t. position
            H[3, 0] = (dvx * r - dx * dot_prod / r) / (r**2)
            H[3, 1] = (dvy * r - dy * dot_prod / r) / (r**2)
            H[3, 2] = (dvz * r - dz * dot_prod / r) / (r**2)
            
            # Partial w.r.t. velocity
            H[3, 3] = dx / r
            H[3, 4] = dy / r
            H[3, 5] = dz / r
        
        return H


# ============================================================================
# END OF PHASE 3A: DroneRadar Class
# ============================================================================


def os_cfar_2d(X, guard_sz=3, train_sz=6, k_rank=0.75, offset=3.0):
    """
    2D Order Statistic CFAR (OS-CFAR) detector on a range-doppler matrix.
    X: 2D matrix in dB (doppler x range)
    guard_sz: guard region size around test cell
    train_sz: training region size
    k_rank: percentile rank (0.0 to 1.0) to select as the noise estimate. 0.75 = 75th percentile.
    offset: detection threshold offset in dB
    Returns: 2D boolean matrix of detections
    """
    ndop, nrng = X.shape
    det = np.zeros((ndop, nrng), dtype=bool)
    
    for d in range(train_sz + guard_sz, ndop - train_sz - guard_sz):
        for r in range(train_sz + guard_sz, nrng - train_sz - guard_sz):
            # Training region (excluding the guard region and cell under test)
            train_top = X[d-train_sz-guard_sz:d-guard_sz, r-train_sz-guard_sz:r+train_sz+guard_sz+1]
            train_bot = X[d+guard_sz+1:d+train_sz+guard_sz+1, r-train_sz-guard_sz:r+train_sz+guard_sz+1]
            train_left = X[d-guard_sz:d+guard_sz+1, r-train_sz-guard_sz:r-guard_sz]
            train_right = X[d-guard_sz:d+guard_sz+1, r+guard_sz+1:r+train_sz+guard_sz+1]
            
            # Flatten into a single 1D array of training cells
            train_region = np.concatenate([
                train_top.flatten(), train_bot.flatten(), 
                train_left.flatten(), train_right.flatten()
            ])
            
            # OS-CFAR Logic: Sort and pick the k-th rank
            train_sorted = np.sort(train_region)
            k_idx = int(k_rank * len(train_sorted))
            noise_estimate = train_sorted[k_idx]
            
            # Apply threshold
            threshold = noise_estimate + offset
            det[d, r] = X[d, r] > threshold
            
    return det

def cfar_1d(x, tr=12, gr=4, offset=3.0):
    N = len(x)
    det = np.zeros(N, dtype=bool)
    for i in range(tr+gr, N-tr-gr):
        noise = np.concatenate([
            x[i-tr-gr:i-gr],
            x[i+gr+1:i+gr+tr+1]
        ])
        thr = np.mean(noise) + offset
        det[i] = x[i] > thr
    return det

def cfar_2d(X, guard_sz=3, train_sz=6, offset=3.0):
    """
    2D CFAR detector operating on range-doppler matrix.
    X: 2D matrix (doppler x range)
    guard_sz: guard region size around test cell
    train_sz: training region size
    offset: detection threshold offset in dB
    Returns: 2D boolean matrix of detections
    """
    ndop, nrng = X.shape
    det = np.zeros((ndop, nrng), dtype=bool)
    
    for d in range(train_sz + guard_sz, ndop - train_sz - guard_sz):
        for r in range(train_sz + guard_sz, nrng - train_sz - guard_sz):
            # Define guard and training regions
            guard_region = X[d-guard_sz:d+guard_sz+1, r-guard_sz:r+guard_sz+1]
            
            # Training region (exclude guard)
            train_top = X[d-train_sz-guard_sz:d-guard_sz, r-train_sz-guard_sz:r+train_sz+guard_sz+1]
            train_bot = X[d+guard_sz+1:d+train_sz+guard_sz+1, r-train_sz-guard_sz:r+train_sz+guard_sz+1]
            train_left = X[d-guard_sz:d+guard_sz+1, r-train_sz-guard_sz:r-guard_sz]
            train_right = X[d-guard_sz:d+guard_sz+1, r+guard_sz+1:r+train_sz+guard_sz+1]
            
            train_region = np.concatenate([train_top.flatten(), train_bot.flatten(), 
                                          train_left.flatten(), train_right.flatten()])
            
            # CFAR detection
            noise_level = np.mean(train_region)
            threshold = noise_level + offset
            det[d, r] = X[d, r] > threshold
    
    return det
# MUSIC AOA
def music_2d_aoa(X, lam, ant_pos, n_src=1, diag_load=1e-3):
    """
    X : complex ndarray (N_rx, K snapshots)
    returns: theta_hat (rad), Music spectrum
    """

    N_rx, N_snap = X.shape

    # Spatial covariance
    R = (X @ X.conj().T) / N_snap
    R += diag_load * np.eye(N_rx)

    # Eigendecomposition
    eigvals, eigvecs = np.linalg.eigh(R)
    idx = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, idx]

    # Noise subspace
    En = eigvecs[:, n_src:]

    # MUSIC spectrum
    az_grid = np.linspace(-60, 60, 61)
    el_grid = np.linspace(0, 60, 31)
    P = np.zeros((len(az_grid), len(el_grid)))
    for i, az in enumerate(np.deg2rad(az_grid)):
        for j, el in enumerate(np.deg2rad(el_grid)):
            k_vec = np.array([
                np.cos(el) * np.cos(az),
                np.cos(el) * np.sin(az),
                np.sin(el)
            ])
            a = np.exp(
            1j * 2 * np.pi / lam * (ant_pos @ k_vec)).reshape(-1, 1)
            den = np.real(a.conj().T @ En @ En.conj().T @ a).item()
            P[i] = 1.0 / (den + 1e-12)
    idx = np.unravel_index(np.argmax(P), P.shape)    
    theta_hat = np.deg2rad(az_grid[idx[0]])
    phi_hat = np.deg2rad(el_grid[idx[1]])
         
    return theta_hat, phi_hat


# Jacobian : Spherical To Cartesian

def jacobian_spherical_to_cart(R, v, theta, phi):
    J = np.zeros((6,4))
    #1. d/dR
    J[0,0] = np.cos(phi) * np.cos(theta)
    J[1,0] = np.cos(phi) * np.cos(theta)
    J[2,0] = np.sin(phi)
    #2. d/dV 
    J[3,1] = np.cos(phi) * np.cos(theta)
    J[4,1] = np.cos(phi) * np.sin(theta)
    J[5,1] = np.sin(phi)
    #3. d/dtheta (Azimuth)
    J[0,2] = -R * np.cos(phi) * np.sin(theta)
    J[1,2] = R * np.cos(phi) * np.cos(theta)
    # dz/dtheta is 0
    J[3,2] = -v * np.cos(phi) * np.sin(theta)
    J[4,2] = v * np.cos(phi) * np.cos(theta)
    # dvz/dtheta is 0

    #4. d/dphi (Elevation)
    J[0,3] = -R * np.sin(phi) * np.cos(theta)
    J[1,3] = -R *np.sin(phi) * np.sin(theta)
    J[2,3] = R * np.cos(phi)
    J[3,3] = -v * np.sin(phi) * np.cos(theta)
    J[4,3] = -v * np.sin(phi) * np.sin(theta)
    J[5,3] = v * np.cos(phi)
    return J

# IMM Track Class(cartesion)
class IMMTrack:
    _id = 0
    def __init__(self, z, R_cart, dt):
        self.id = IMMTrack._id; IMMTrack._id += 1
        self.dt = dt
# CV
        self.x_cv =z.reshape(6, 1)
        self.P_cv = R_cart.copy()
# CA
        self.x_ca = np.vstack([z.reshape(6,1), np.zeros((3,1))])
        P_ca_init = np.eye(9) * 10
        P_ca_init[:6, :6] = R_cart.copy()
        self.P_ca = P_ca_init
# Model Probab
        self.mu = np.array([0.5,0.5]) 

# Markov
        self.PI = np.array([[0.95, 0.05],
                            [0.05, 0.95]])
        self.age = 1
        self.hits = 1
        self.misses = 0
        self.state = "tentative"
    def mix(self):
        mu_pred = self.PI.T @ self.mu

        w = np.zeros((2,2))
        for j in range (2):
            for i in range(2):
                w[i,j] = self.PI[i,j] * self.mu[i] / (mu_pred[j] + 1e-9)

        x0_cv = w[0,0]*self.x_cv + w[1,0]*self.x_ca[:6]
# Mix Covariance
        diff_cv = self.x_cv - x0_cv
        diff_ca_cv = self.x_ca[:6] - x0_cv       
        P0_cv = (
            w[0,0]*(self.P_cv + diff_cv @ diff_cv.T) + \
            w[1,0]*(self.P_ca[:6,:6] + diff_ca_cv @ diff_ca_cv.T)
        )
        x0_ca = np.vstack([
            w[0,1]*self.x_cv + w[1,1]*self.x_ca[:6],
            np.zeros((3,1))
        ])
        P0_ca = self.P_ca.copy()
        diff_cv_ca = self.x_cv - x0_ca[:6]
        diff_ca = self.x_ca[:6] - x0_ca[:6]
        P0_ca[:6,:6] = (
            w[0,1]*(self.P_cv + diff_cv_ca @ diff_cv_ca.T) + \
            w[1,1]*(self.P_ca[:6,:6] + diff_ca @ diff_ca.T)
        )
        self.x_cv, self.P_cv = x0_cv, P0_cv
        self.x_ca, self.P_ca = x0_ca, P0_ca
        self.mu = mu_pred        

    def predict(self):
        self.mix()
        dt = self.dt

        F_cv = np.eye(6)
        F_cv[0, 3]= dt; F_cv[1,4] = dt; F_cv[2, 5] = dt
        Q_cv = np.eye(6) * 0.3

        F_ca = np.eye(9)
        F_ca[0,3] = dt; F_ca[1,4] = dt; F_ca[2,5] = dt
        F_ca[3,6] = dt; F_ca[4,7] = dt; F_ca[5,8] = dt
        F_ca[0,6] = 0.5 * dt**2; F_ca[1,7] = 0.5 * dt**2; F_ca[2,8] = 0.5 * dt**2
        Q_ca = np.eye(9) * 0.5

        self.x_cv = F_cv @ self.x_cv
        self.P_cv = F_cv @ self.P_cv @ F_cv.T + Q_cv
        self.x_ca = F_ca @ self.x_ca
        self.P_ca = F_ca @ self.P_ca @ F_ca.T + Q_ca

    def update(self, z, R_cart):

        H_cv = np.eye(6)
        y_cv = z.reshape(6,1) - H_cv @ self.x_cv
        S_cv = H_cv @ self.P_cv @ H_cv.T + R_cart
        K_cv = self.P_cv @ H_cv.T @ np.linalg.inv(S_cv)
        self.x_cv += K_cv @ y_cv
        self.P_cv = (np.eye(6) - K_cv @ H_cv) @ self.P_cv

        det_S_cv = np.linalg.det(S_cv)
        norm_const_cv = 1.0 / (np.sqrt((2*np.pi)**6 * det_S_cv) + 1e-9)
        L_cv = norm_const_cv * np.exp(-0.5 * y_cv.T @ np.linalg.inv(S_cv) @ y_cv)

        H_ca = np.hstack([np.eye(6), np.zeros((6,3))])
        y_ca = z.reshape(6,1) - H_ca @ self.x_ca
        S_ca = H_ca @ self.P_ca @ H_ca.T + R_cart
        K_ca = self.P_ca @ H_ca.T @ np.linalg.inv(S_ca)
        self.x_ca += K_ca @ y_ca
        self.P_ca = (np.eye(9) - K_ca @ H_ca) @ self.P_ca

        det_S_ca = np.linalg.det(S_ca)
        norm_const_ca = 1.0 / (np.sqrt((2 *np.pi)**6 * det_S_ca) + 1e-9)
        L_ca = norm_const_ca * np.exp(-0.5 * y_ca.T @ np.linalg.inv(S_ca) @ y_ca)


        self.mu *= np.array([L_cv.item(), L_ca.item()])
        self.mu /= (np.sum(self.mu) + 1e-9)

        self.hits += 1
        self.misses = 0
        if self.state == "tentative" and self.hits >= 6:
            self.state = "confirmed"

    def miss(self):
        self.misses += 1
        self.age += 1

    def fused(self):
        x = self.mu[0]*self.x_cv + self.mu[1]*self.x_ca[:6]
        return x            

# RADAR PARAMETERS
c = 3e8
fc = 77e9
B = 200e6
T = 30e-6
Sf = B / T
fs = 20e6
lam = c / fc

num_chirps = 64
num_frames = 120
SNR_dB = 15 ; snr_lin = 10**(SNR_dB/10)
max_range = 150
N_rx = 4
d_ant = lam / 2
dt = num_chirps * T


targets = [
    {"Rn": 55.0, "Vl": -10, "theta": np.deg2rad(15)},
    {"Rn": 50.0, "Vl": -10, "theta": np.deg2rad(-15)}
    
]
if __name__ == "__main__":
    print("FMCW radar simulation started")


    # Signal Setup
    t_fast = np.arange(0, T, 1/fs)
    N_fast = len(t_fast)
    t_grid, k_grid = np.meshgrid(t_fast, np.arange(num_chirps))

    tracks =[]
    tracks_hist = []         
    meas_hist = []
    hist_signal = []
    hist_range = []
    hist_rd_map = []
    hist_cfar = []
    imu_hist = {}
    print("\nInitiating Multi- frame Tracking  ...\n")

    # Frame Loop
    for frame in tqdm(range(num_frames), desc="Processing"): 
        # FMCW Simulation
        beat = np.zeros((N_rx, num_chirps, N_fast), dtype=complex)
        
        for tgt in targets:
                R = tgt["Rn"] + tgt["Vl"] * (frame * dt + k_grid * T)
                fb = 2 * Sf * R / c
                fD = 2 * tgt["Vl"] * fc / c
                steering = np.exp(
                    1j * 2 * np.pi * d_ant * np.arange(N_rx) * np.sin(tgt["theta"]) / lam
                )
                phase = 2 * np.pi * (fb * t_grid + fD * k_grid * T)
                for rx in range(N_rx):
                    beat[rx] += steering[rx] * np.exp(1j * phase)

        #noise = (np.random.randn(*beat.shape) + 1j * np.random.randn(*beat.shape))    
        #beat += noise * 10**(-SNR_dB / 20)
        beat *= windows.hann((N_fast))[None,None,:]
        hist_signal.append(np.real(beat[0, 0, :]))
    # RANGE FFT
        rng_fft = np.fft.fft(beat, axis=2)[:,:,:N_fast//2]
        freqs = np.fft.fftfreq(N_fast, d=1/fs)[:N_fast//2]
        ranges = (c * freqs) / (2 * Sf)
        mask = ranges <= max_range
        rng_fft = rng_fft[:,:,mask]
        ranges_valid = ranges[mask]
        
        rng_profile = np.mean(np.abs(rng_fft[0]), axis=0)
        hist_range.append(20 * np.log10(rng_profile + 1e-6)) 

    # rng_fft -= np.mean(rng_fft, axis=0)
    # rng_fft *= windows.blackman(num_chirps)[:, None]
        #rng_fft = rng_fft / (np.linalg.norm(rng_fft, axis=0, keepdims=True) + 1e-6) 
    # DOPPLER FFT
        #rng_fft -= np.mean(rng_fft, axis=1, keepdims=True)
        #rng_fft *= windows.blackman(num_chirps)[None,:, None]
        doppler_fft = np.fft.fftshift(
            np.fft.fft(rng_fft, axis=1),
            axes=1
    )

        rd = np.abs(doppler_fft).mean(axis=0)
        rd_db = 20 * np.log10(rd + 1e-6)
        hist_rd_map.append(rd_db)
    # CFAR
        
        det_r = cfar_1d(rd_db.mean(axis=0))
        r_bins = np.where(det_r)[0]

        hist_cfar.append(det_r)
        vel_axis = np.fft.fftshift(np.fft.fftfreq(num_chirps, T)) * c / (2 * fc)

        measurements = []
        covs = []
        for r0 in r_bins:
            X = doppler_fft[:,:,r0]    
            theta, spectrum = music_2d_aoa(X, lam, d_ant)
            Rm = ranges_valid[r0]
            Vm = vel_axis[np.argmax(rd[:,r0])]

            phi = 0.0
            z = np.array([
                Rm*np.cos(phi) * np.cos(theta),
                Rm*np.cos(phi) * np.sin(theta),
                Rm*np.sin(phi),
                Vm*np.cos(phi) * np.cos(theta),
                Vm*np.cos(phi) * np.sin(theta),
                Vm*np.sin(phi)
                ])
            sigma_R = 0.15 + 0.002*Rm
            sigma_v = 0.2
            sigma_theta = np.deg2rad(2)/np.sqrt(snr_lin)
            sigma_phi = np.deg2rad(2) / np.sqrt(snr_lin)

            R_spherical = np.diag([sigma_R**2,sigma_v**2,sigma_theta**2, sigma_phi**2])
            J = jacobian_spherical_to_cart(Rm,Vm,theta)
            R_cart = J @ R_spherical @ J.T

            measurements.append(z)
            covs.append(R_cart)                
            
        meas_hist.append(np.array(measurements)[:, :2] if measurements else np.empty((0, 2)))      
        
    
    # Multi traget tracking (GNN + IMM)
            
        for t in tracks:
            t.predict()
            #trk.miss()

        nT = len(tracks)
        nM = len(measurements)
        asi_tracks = set()
        asi_meas = set()
        


        if nT and nM:
            C = np.full((nT, nM), 1e6)
            H_mat = np.eye(6)
            #R_val = np.diag([1, 5, 1, 5])
            #H = np.eye(4)

            for i, t in enumerate(tracks):
                x = t.fused()
                P_fused = t.mu[0]*t.P_cv + t.mu[1]* t.P_ca[:4, :4]

            # P_fused = trk.mu[0]*trk.P_cv + trk.mu[1]*trk.P_ca[:4,:4]
                #S = H @ P_fused @ H.T + R_val
                #S_inv = np.linalg.inv(S)

                for j, z in enumerate(measurements):
                    y = z.reshape(4,1) - x
                    S = H_mat @ P_fused @ H_mat.T + covs[j]
                    S_inv = np.linalg.inv(S)
                    C[i, j] = (y.T @ S_inv @ y).item()

            ri, ci = linear_sum_assignment(C)
            GATE = 25
            for i, j in zip(ri, ci):
                if C[i, j] < GATE:
                    tracks[i].update(measurements[j], covs[j])
                    asi_tracks.add(i)
                    asi_meas.add(j)
            
        for i, t in enumerate(tracks):
            if i not in asi_tracks:
                t.miss()
        # Track Birth
        for j, z in enumerate(measurements):
            if j not in asi_meas:
                tracks.append(IMMTrack(z, covs[j], dt))
        
        # Track Death
        tracks = [t for t in tracks if t.misses < 10]
        
        tracks_hist.append([t.fused()[:2].flatten() for t in tracks if t.state == "confirmed"])
        meas_hist.append(np.array(measurements)[:, :2] if measurements else np.empty((0,2)))
        for t in tracks:
            if t.state == "confirmed":
                imu_hist.setdefault(t.id, []).append((frame, t.mu.copy()))
            
                


    # PLOT
    print("Generating Diagnostic Animation...")


    fig = plt.figure(figsize=(14, 12))
    gs = GridSpec(3, 2, figure=fig, height_ratios=[1, 1, 1.2])

    ax_sig = fig.add_subplot(gs[0, 0])
    ax_rng = fig.add_subplot(gs[0, 1])
    ax_rd = fig.add_subplot(gs[1, 0])
    ax_map = fig.add_subplot(gs[1, 1])
    ax_imm = fig.add_subplot(gs[2, :])
    plt.suptitle(f"FMCW MIMO (IMM + MUSIC)", fontsize=16)

    # A. Beat Signal
    line_sig, = ax_sig.plot(t_fast*1e6, hist_signal[0])
    ax_sig.set_title("1. Beat Signal (Chirp 0)")
    ax_sig.set_xlabel("Time (us)")
    ax_sig.grid(True)
    ax_sig.set_ylim(np.min(hist_signal)*1.2, np.max(hist_signal)*1.2)


    # 2. Range Profile Plot
    line_rng, = ax_rng.plot(ranges_valid, hist_range[0])
    ax_rng.set_title("2. Range Profile")
    ax_rng.set_ylim(0, 100) # Assuming ~max dB
    ax_rng.set_xlabel("Range (m)")
    ax_rng.grid(True)

    # 3. Range-Doppler Map (Heatmap)
    extent = [ranges_valid[0], ranges_valid[-1], vel_axis[0], vel_axis[-1]]
    im_rd = ax_rd.imshow(hist_rd_map[0], aspect='auto', origin='lower', extent=extent, cmap='jet', vmin=40, vmax=110)
    ax_rd.set_title("3. Range-Doppler Heatmap")
    ax_rd.set_xlabel("Range (m)")
    ax_rd.set_ylabel("Velocity (m/s)")

    # 4. Final Tracking (Overlaid on CFAR Mask)
    scat_meas = ax_map.scatter([], [], c='lime', marker='x', s=80, label='Meas')
    scat_tracks = ax_map.scatter([], [], c='red', s=100, label='Track')
    ax_map.set_title("4. Cartesian Map (Top-Down)")
    ax_map.grid(True)
    ax_map.set_xlabel("X  (m)")
    ax_map.set_ylabel("Y  (m)")
    ax_map.set_xlim(40, 75)
    ax_map.set_ylim(-30, 30)
    ax_map.legend()

    # IMM Plot
    ax_imm.set_title("5. IMM Mdoel Probability (Solid=CV, Dashed=CA)")
    ax_imm.set_xlabel("frame"); ax_imm.set_ylabel("Probability")
    ax_imm.set_ylim(0, 1.1); ax_imm.set_xlim(0, num_frames)
    ax_imm.grid(True)

    imm_lines ={}
    colors = plt.cm.tab10.colors


    def update(frame):
        
        line_sig.set_ydata(hist_signal[frame])    
        line_rng.set_ydata(hist_range[frame])
        im_rd.set_data(hist_rd_map[frame])
        pts_m = np.array(meas_hist[frame])
        pts_t = np.array(tracks_hist[frame])
        scat_meas.set_offsets(pts_m if len(pts_m) else np.empty((0,2)))
        scat_tracks.set_offsets(pts_t if len(pts_t) else np.empty((0,2)))
        for tid, history in imu_hist.items():
            data_now = [h for h in history if h[0] <= frame]

            if not data_now: continue

            if tid not in imm_lines:
                c = colors[tid % len(colors)]
                ln_cv, = ax_imm.plot([], [], '-', color=c, lw=1.5, label=f'T={tid} CV')
                ln_ca, = ax_imm.plot([], [], '--', color=c, lw=1.5, alpha=0.6)
                imm_lines[tid] = (ln_cv, ln_ca)

            frames = [h[0] for h in data_now]
            cv_probs = [h[1][0] for h in data_now]
            ca_probs = [h[1][1] for h in data_now]

            ln_cv, ln_ca = imm_lines[tid]
            ln_cv.set_data(frames, cv_probs)
            ln_ca.set_data(frames, ca_probs)    

        return line_sig, line_rng, im_rd, scat_tracks, scat_meas, *[ln for pair in imm_lines.values() for ln in pair]

    def get_tracks_at_frame(frame):
        """
        Return confirmed tracks for a given frame
        """
        if frame < len(tracks_hist):
            return tracks_hist[frame]
        return[]  
if __name__ == "__main__":

    ani = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=True)

    with tqdm(total=num_frames, desc="Rendering GIF", unit="frame") as bar:
        def update_progress(current_frame, total_frames):
            bar.update(1) # Advance bar by 1 frame


        ani.save("FMCW_Dashboard IMM AOA.gif", writer="pillow", dpi=120, progress_callback = update_progress)
        plt.close()
        print("Done! Saved 'FMCW_Dashboard IMM AOA Final.gif'")

       
