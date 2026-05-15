import numpy as np
from fmcw_radar import DroneRadar  # Import DroneRadar for Phase 3A
from timesync import Clock

class Drone:
    """
    Drone agent with optional radar sensor (Phase 3A) and local clock drift (Phase 3C).
    
    Theory Reference:
    - Section 3: Drone dynamics and motion models
    - Section 3.6: Relative motion for radar measurements
    - Section 191: Temporal synchronization via timestamp consensus
    """
    
    def __init__(
        self,
        position,
        speed=40,
        radar_enabled=True,
        drone_id=None,
        navigation_constant=3.0,
        max_acceleration=None,
    ):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        self.speed = speed
        self.navigation_constant = navigation_constant
        self.max_acceleration = max_acceleration if max_acceleration is not None else speed * 2.0
        self.target = None
        
        # PHASE 3A: Add drone ID and radar
        self.drone_id = drone_id if drone_id is not None else 0
        self.orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # PHASE 3C: Local clock for each drone
        self.clock = Clock(drone_id=self.drone_id)
        
        # Radar sensor (optional, for Phase 3)
        self.radar_enabled = radar_enabled
        if radar_enabled:
            self.radar = DroneRadar(
                drone_id=self.drone_id,
                position=self.position,
                velocity=self.velocity,
                orientation=self.orientation
            )
        else:
            self.radar = None

    def assign_target(self, target):
        self.target = target

    def _target_state(self):
        """Return assigned target position and velocity as 3D vectors."""
        if self.target is None:
            return None, None

        if isinstance(self.target, dict):
            pos = np.asarray(self.target.get('position', [0.0, 0.0, 0.0]), dtype=float)
            vel = np.asarray(self.target.get('velocity', [0.0, 0.0, 0.0]), dtype=float)
        elif hasattr(self.target, 'position'):
            pos = np.asarray(self.target.position, dtype=float)
            vel = np.asarray(getattr(self.target, 'velocity', [0.0, 0.0, 0.0]), dtype=float)
        else:
            pos = np.asarray(self.target, dtype=float)
            vel = np.zeros(3, dtype=float)

        if pos.shape[0] < 3:
            pos = np.pad(pos, (0, 3 - pos.shape[0]))
        if vel.shape[0] < 3:
            vel = np.pad(vel, (0, 3 - vel.shape[0]))

        return pos[:3], vel[:3]

    def _proportional_navigation_acceleration(self, target_pos, target_vel):
        """Compute 3D proportional-navigation acceleration.

        Vector PN uses:

            omega_LOS = (r x v_rel) / ||r||^2
            V_c = -v_rel dot r_hat
            a_c = N * V_c * (omega_LOS x r_hat)

        where r is target-relative position, v_rel is target-relative
        velocity, N is the navigation constant, and V_c is closing speed.
        """
        rel_pos = target_pos - self.position
        rel_vel = target_vel - self.velocity
        distance = np.linalg.norm(rel_pos)
        eps = 1e-9

        if distance < eps:
            return np.zeros(3, dtype=float), distance

        los_unit = rel_pos / distance
        closing_velocity = -float(np.dot(rel_vel, los_unit))
        los_rate = np.cross(rel_pos, rel_vel) / max(distance**2, eps)

        pn_accel = (
            self.navigation_constant
            * max(closing_velocity, 0.0)
            * np.cross(los_rate, los_unit)
        )

        # A stationary interceptor has no LOS-rate authority yet, so add a
        # short boost along LOS until it has useful closing speed.
        if np.linalg.norm(self.velocity) < 0.25 * self.speed or closing_velocity <= 0.0:
            pn_accel += los_unit * self.max_acceleration

        accel_norm = np.linalg.norm(pn_accel)
        if accel_norm > self.max_acceleration:
            pn_accel = pn_accel / accel_norm * self.max_acceleration

        return pn_accel, distance

    def update(self, dt):
        """
        Update drone position, velocity, and local clock.
        Also updates radar platform state (Phase 3A).
        """
        # PHASE 3C: Advance the local drifted clock.
        self.clock.tick(dt)

        if self.target is None:
            return
        
        target_pos, target_vel = self._target_state()
        if target_pos is None:
            return
        
        acceleration, distance = self._proportional_navigation_acceleration(target_pos, target_vel)
        
        if distance < 20:
            self.target = None
            return
        
        self.velocity += acceleration * dt
        speed_now = np.linalg.norm(self.velocity)
        if speed_now > self.speed:
            self.velocity = self.velocity / speed_now * self.speed

        self.position += self.velocity * dt
        
        # PHASE 3A: Update radar platform state when drone moves
        if self.radar_enabled and self.radar is not None:
            self.radar.update_platform_state(
                position=self.position,
                velocity=self.velocity,
                orientation=self.orientation
            )
    
    def generate_measurements(self, targets):
        """
        PHASE 3A: Generate independent radar measurements from drone's perspective.
        
        Theory: Section 3.6 (Relative Motion), 8 (Measurement Models)

        Returns:
            List of measurement dicts if radar enabled, else empty list
        """
        if not self.radar_enabled or self.radar is None:
            return []
        
        measurements = self.radar.get_measurements(targets)
        
        # Add drone metadata and local timestamp to each measurement
        timestamp = self.clock.read()
        for meas in measurements:
            meas['sensor_id'] = self.drone_id
            meas['sensor_type'] = 'DroneRadar'
            meas['timestamp'] = timestamp
        
        return measurements
    
    def get_measurement_jacobian(self, target_state):
        """
        PHASE 3A: Get Jacobian matrix for this drone's radar measurements.
        
        Theory: Section 8.7, Eq 124-127
        
        Returns:
            Jacobian matrix H (4 x 6), or None if radar not enabled
        """
        if not self.radar_enabled or self.radar is None:
            return None
        
        return self.radar.get_measurement_jacobian(target_state)
                
