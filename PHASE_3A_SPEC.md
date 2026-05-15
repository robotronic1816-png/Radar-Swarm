# Phase 3A: Distributed Radar Nodes - Technical Specification

**Status:** Ready for Implementation  
**Priority:** HIGH (Blocking Phase 3B & 3C)  
**Effort:** 1 week  
**Theory Ref:** Sections 3.6, 8, 14.3 (Measurement Generation)

---

## Objective

Enable each drone in the swarm to generate independent radar measurements from its moving perspective, replacing current single ground-based radar with distributed sensing network.

---

## Current State (Phase 2)

```python
# Current: Single centralized radar
radar = FMCW_Radar(position=[0, 0, 100])  # Stationary ground radar

targets = [Target(...), Target(...)]
measurements = radar.get_measurements(targets)  # One set of measurements

tracker.update(measurements)  # All drones use same measurements
```

---

## Target State (Phase 3A)

```python
# New: Distributed radar on each drone
swarm = Swarm(num_drones=3)

for drone in swarm.drones:
    # Each drone has its own radar
    drone.radar = DroneRadar(drone_id=drone.id, 
                             position=drone.position,
                             orientation=drone.orientation)
    
    # Each drone independently measures targets
    local_measurements = drone.radar.get_measurements(targets, 
                                                      drone_position=drone.position)
    
    # Each drone locally tracks
    drone.local_tracker.update(local_measurements)
    
    # Measurements will be fused in Phase 3B

# Result: 3 independent measurement sets per timestep
```

---

## Implementation Tasks

### Task 1: Create `DroneRadar` Class
**File:** `fmcw_radar.py` (extend existing)  
**Estimated Time:** 2 days

#### Pseudocode:
```python
class DroneRadar(FMCW_Radar):
    """Radar mounted on moving drone platform"""
    
    def __init__(self, drone_id, position, orientation, radar_config):
        super().__init__(radar_config)
        self.drone_id = drone_id
        self.platform_position = position     # Drone position: [x, y, z]
        self.platform_velocity = None         # Will be updated externally
        self.platform_orientation = orientation  # Roll, pitch, yaw
        
    def update_platform_state(self, position, velocity, orientation):
        """Update drone dynamics"""
        self.platform_position = position
        self.platform_velocity = velocity
        self.platform_orientation = orientation
    
    def get_measurements(self, targets, noise_level=1.0):
        """
        Generate radar measurements from drone perspective.
        
        Theory:
        - Relative position: r = p_target - p_drone (Eq 44)
        - Relative velocity: v_rel = v_target - v_drone (Eq 45)
        - Use relative motion to compute beat frequency
        
        Returns:
            List of detections: [(range, doppler, azimuth, elevation, amplitude), ...]
        """
        measurements = []
        
        for target in targets:
            # 1. Compute relative geometry (Section 3.6)
            relative_pos = target.position - self.platform_position
            relative_vel = target.velocity - self.platform_velocity
            
            # 2. Apply platform orientation (transform to radar frame)
            rel_pos_radar_frame = self._rotate_to_radar_frame(relative_pos)
            rel_vel_radar_frame = self._rotate_to_radar_frame(relative_vel)
            
            # 3. Compute beat frequency (using relative velocity)
            # Doppler shift from drone's own velocity included here
            beat_freq = self._compute_beat_frequency(rel_pos_radar_frame, 
                                                     rel_vel_radar_frame)
            
            # 4. Estimate range from beat frequency (Eq 56: R = c*f_b/(2*S))
            range_m = self._beat_freq_to_range(beat_freq)
            
            # 5. Estimate Doppler (Eq 60: v = λ*Δφ/(4πT))
            doppler_vel = self._compute_doppler_velocity(rel_vel_radar_frame)
            
            # 6. Estimate angles (MUSIC - Eq 108)
            azimuth, elevation = self._estimate_aoa(rel_pos_radar_frame)
            
            # 7. Add noise
            range_m += noise_level * np.random.randn() * self.range_std
            doppler_vel += noise_level * np.random.randn() * self.doppler_std
            azimuth += noise_level * np.random.randn() * self.angle_std
            elevation += noise_level * np.random.randn() * self.angle_std
            
            measurement = {
                'range': range_m,
                'doppler': doppler_vel,
                'azimuth': azimuth,
                'elevation': elevation,
                'amplitude': np.linalg.norm(relative_pos) ** (-2),  # Signal decay
                'timestamp': self.current_time,
                'drone_id': self.drone_id,
                'target_id': target.id if hasattr(target, 'id') else None
            }
            
            measurements.append(measurement)
        
        return measurements
    
    def _rotate_to_radar_frame(self, vector_global):
        """
        Transform vector from global frame to radar frame using platform orientation.
        
        Radar measures in its own frame of reference:
        - Range along boresight
        - Angles from platform orientation
        """
        # TODO: Implement rotation matrix based on self.platform_orientation
        # For now, assume radar boresight = global Z-axis
        pass
    
    def _compute_beat_frequency(self, relative_pos, relative_vel):
        """
        Compute beat frequency from relative position and velocity.
        
        Theory:
        - Beat frequency from range: f_b = 2*S*R/c (Eq 55)
        - Doppler shift: f_D = 2*v*f_c/c (Eq 57)
        - Combined: f_b_total = 2*S*R/c + 2*v_radial*f_c/c
        
        Here, v_radial = dot(relative_vel, relative_pos) / ||relative_pos||
        """
        pass
    
    def get_measurement_jacobian(self, target_state_cartesian):
        """
        Return Jacobian matrix for EKF update.
        
        Theory: Section 8.7, Eq 124-127
        H = ∂h(x)/∂x, where h(x) transforms state to measurement space
        
        State: x = [x, y, z, vx, vy, vz]^T (6D)
        Measurement: z = [r, θ, φ, ṙ]^T (4D)
        
        Jacobian H (4x6):
        [∂r/∂x,  ∂r/∂y,  ∂r/∂z,  ∂r/∂vx,  ∂r/∂vy,  ∂r/∂vz]
        [∂θ/∂x,  ∂θ/∂y,  ∂θ/∂z,  ∂θ/∂vx,  ∂θ/∂vy,  ∂θ/∂vz]
        [∂φ/∂x,  ∂φ/∂y,  ∂φ/∂z,  ∂φ/∂vx,  ∂φ/∂vy,  ∂φ/∂vz]
        [∂ṙ/∂x,  ∂ṙ/∂y,  ∂ṙ/∂z,  ∂ṙ/∂vx,  ∂ṙ/∂vy,  ∂ṙ/∂vz]
        """
        x, y, z = target_state_cartesian[0:3]
        vx, vy, vz = target_state_cartesian[3:6]
        
        # Relative position from drone
        dx = x - self.platform_position[0]
        dy = y - self.platform_position[1]
        dz = z - self.platform_position[2]
        
        # Distance
        r = np.sqrt(dx**2 + dy**2 + dz**2)
        rho = np.sqrt(dx**2 + dy**2)  # Horizontal distance
        
        # Initialize Jacobian (4x6)
        H = np.zeros((4, 6))
        
        # Row 1: Range partial derivatives (Eq 125)
        H[0, 0] = dx / r      # ∂r/∂x
        H[0, 1] = dy / r      # ∂r/∂y
        H[0, 2] = dz / r      # ∂r/∂z
        # H[0, 3:6] = 0 (range doesn't depend on velocity directly in measurement)
        
        # Row 2: Azimuth partial derivatives (Eq 126)
        if rho > 0.01:  # Avoid singularity
            H[1, 0] = -dy / (rho**2)  # ∂θ/∂x
            H[1, 1] = dx / (rho**2)   # ∂θ/∂y
            # H[1, 2] = 0 (azimuth doesn't depend on z)
        
        # Row 3: Elevation partial derivatives (Eq 127)
        if r > 0.01:
            H[2, 2] = rho / r**2      # ∂φ/∂z
            if rho > 0.01:
                H[2, 0] = -dx * dz / (r**2 * rho)  # ∂φ/∂x
                H[2, 1] = -dy * dz / (r**2 * rho)  # ∂φ/∂y
        
        # Row 4: Radial velocity (Doppler) partial derivatives (Eq 118)
        # ṙ = (x*vx + y*vy + z*vz) / r = dot(position, velocity) / r
        if r > 0.01:
            H[3, 0] = (vx * r - dx * (dx*vx + dy*vy + dz*vz) / r) / r**2
            H[3, 1] = (vy * r - dy * (dx*vx + dy*vy + dz*vz) / r) / r**2
            H[3, 2] = (vz * r - dz * (dx*vx + dy*vy + dz*vz) / r) / r**2
            H[3, 3] = dx / r   # ∂ṙ/∂vx
            H[3, 4] = dy / r   # ∂ṙ/∂vy
            H[3, 5] = dz / r   # ∂ṙ/∂vz
        
        return H
```

#### Key Methods to Implement:

| Method | Theory | Priority | Notes |
|--------|--------|----------|-------|
| `update_platform_state()` | Section 3.6 | 🔴 HIGH | Called before each measurement |
| `get_measurements()` | Section 3.6, 8 | 🔴 HIGH | Core measurement generation |
| `_rotate_to_radar_frame()` | Section 8.3 | 🟡 MEDIUM | Handle drone orientation |
| `_compute_beat_frequency()` | Section 4.4 | 🔴 HIGH | Include relative motion |
| `get_measurement_jacobian()` | Section 8.7 | 🔴 HIGH | Required for EKF tracking |

---

### Task 2: Extend `drone.py`
**File:** `drone.py`  
**Estimated Time:** 1.5 days

#### Current Structure:
```python
class Drone:
    def __init__(self, drone_id, position, velocity):
        self.id = drone_id
        self.position = position
        self.velocity = velocity
        self.orientation = [0, 0, 0]  # Roll, pitch, yaw
        # NO RADAR
        
    def update(self, dt, target_forces):
        # Update position/velocity
        pass
```

#### Extended Structure:
```python
class Drone:
    def __init__(self, drone_id, position, velocity, radar_config=None):
        self.id = drone_id
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.orientation = np.array([0, 0, 0])  # Roll, pitch, yaw
        
        # NEW: Add radar if config provided
        if radar_config:
            self.radar = DroneRadar(
                drone_id=drone_id,
                position=self.position,
                orientation=self.orientation,
                radar_config=radar_config
            )
        else:
            self.radar = None
    
    def update(self, dt, target_forces):
        # Update dynamics
        self.velocity += target_forces * dt
        self.position += self.velocity * dt
        
        # Update radar platform state
        if self.radar is not None:
            self.radar.update_platform_state(
                position=self.position,
                velocity=self.velocity,
                orientation=self.orientation
            )
    
    def generate_measurements(self, targets):
        """NEW: Generate independent measurements"""
        if self.radar is None:
            return []
        
        measurements = self.radar.get_measurements(targets)
        
        # Add drone metadata
        for meas in measurements:
            meas['sensor_id'] = self.id
            meas['sensor_position'] = self.position.copy()
            meas['sensor_velocity'] = self.velocity.copy()
        
        return measurements
    
    def get_measurement_jacobian(self, target_state):
        """NEW: Get Jacobian for tracking update"""
        if self.radar is None:
            return None
        
        return self.radar.get_measurement_jacobian(target_state)
```

---

### Task 3: Update `main.py` Loop
**File:** `main.py`  
**Estimated Time:** 1 day

#### Current Simulation Loop:
```python
# Phase 2 (current)
for timestep in range(num_timesteps):
    # 1. Update targets
    for target in targets:
        target.update(dt)
    
    # 2. Get measurements from single ground radar
    measurements = radar.get_measurements(targets)
    
    # 3. Track
    tracker.update(measurements)
    
    # 4. Control drones
    for drone in swarm:
        drone.update(dt, forces)
```

#### Extended Loop (Phase 3A):
```python
# Phase 3A (NEW)
for timestep in range(num_timesteps):
    # 1. Update targets
    for target in targets:
        target.update(dt)
    
    # 2a. Update drone positions
    for drone in swarm:
        drone.update(dt, forces)
    
    # 2b. Generate measurements from EACH DRONE
    all_measurements = []
    for drone in swarm:
        local_measurements = drone.generate_measurements(targets)
        all_measurements.extend(local_measurements)
    
    # 3. Tracking (will use all measurements in Phase 3B)
    # For now, use all measurements like before
    tracker.update(all_measurements)
    
    # 4. Get Jacobians for tracking (Phase 3B prep)
    jacobians = {}
    for drone in swarm:
        jacobians[drone.id] = drone.get_measurement_jacobian(tracker.state)
    
    # 5. Store measurement metadata
    # (will be used for fusion in Phase 3B)
    for meas in all_measurements:
        meas['jacobian'] = jacobians[meas['drone_id']]
    
    # 6. Control drones (unchanged)
    for drone in swarm:
        control_forces = controller.compute_control(drone, tracked_target)
        drone.velocity += control_forces * dt
```

---

### Task 4: Unit Tests
**File:** `test_drone_radar.py`  
**Estimated Time:** 1.5 days

```python
import unittest
import numpy as np
from fmcw_radar import DroneRadar
from drone import Drone
from target import Target

class TestDroneRadar(unittest.TestCase):
    
    def setUp(self):
        """Create test fixtures"""
        self.radar_config = {
            'fc': 77e9,      # 77 GHz
            'bandwidth': 4e9,
            'chirp_duration': 60e-6,
            'num_chirps': 128,
        }
        self.drone_radar = DroneRadar(
            drone_id=1,
            position=np.array([0, 0, 50]),
            orientation=np.array([0, 0, 0]),
            radar_config=self.radar_config
        )
        
        self.target = Target(
            position=np.array([100, 50, 100]),
            velocity=np.array([10, 5, 0])
        )
    
    def test_relative_motion_computation(self):
        """
        Test: Relative velocity affects Doppler correctly
        Theory: Section 3.6, Eq 45
        """
        self.drone_radar.update_platform_state(
            position=np.array([0, 0, 50]),
            velocity=np.array([0, 0, 0]),  # Stationary drone
            orientation=np.array([0, 0, 0])
        )
        
        measurements = self.drone_radar.get_measurements([self.target])
        
        # Should see target with positive Doppler (approaching? or receding?)
        self.assertGreater(len(measurements), 0)
        self.assertIn('doppler', measurements[0])
    
    def test_moving_drone_affects_measurement(self):
        """
        Test: Drone motion changes measurements
        Theory: Section 3.6, Eq 44-45
        
        Scenario:
        - Drone moving toward target
        - Should see increased Doppler shift
        """
        # Stationary drone
        self.drone_radar.update_platform_state(
            position=np.array([0, 0, 50]),
            velocity=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0])
        )
        meas_stationary = self.drone_radar.get_measurements([self.target])[0]
        
        # Drone moving toward target
        self.drone_radar.update_platform_state(
            position=np.array([0, 0, 50]),
            velocity=np.array([10, 0, 0]),  # Moving in x-direction toward target
            orientation=np.array([0, 0, 0])
        )
        meas_moving = self.drone_radar.get_measurements([self.target])[0]
        
        # Moving drone should see different Doppler
        doppler_diff = meas_moving['doppler'] - meas_stationary['doppler']
        self.assertNotAlmostEqual(doppler_diff, 0, places=2)
    
    def test_jacobian_dimensions(self):
        """
        Test: Jacobian has correct shape
        Theory: Section 8.7, Eq 124
        """
        state = np.array([100, 50, 100, 10, 5, 0])  # Position + velocity
        H = self.drone_radar.get_measurement_jacobian(state)
        
        # Should be 4x6 (4 measurements, 6 state variables)
        self.assertEqual(H.shape, (4, 6))
    
    def test_jacobian_range_partials(self):
        """
        Test: Range Jacobian matches theory
        Theory: Section 8.7.2, Eq 125
        ∂r/∂x = x/r, ∂r/∂y = y/r, ∂r/∂z = z/r
        """
        # Position target at (100, 0, 0) relative to drone at origin
        target_state = np.array([100, 0, 0, 0, 0, 0])
        self.drone_radar.platform_position = np.array([0, 0, 0])
        
        H = self.drone_radar.get_measurement_jacobian(target_state)
        
        # Expected: ∂r/∂x = 100/100 = 1
        self.assertAlmostEqual(H[0, 0], 1.0, places=5)
        # Expected: ∂r/∂y = 0/100 = 0
        self.assertAlmostEqual(H[0, 1], 0.0, places=5)
        # Expected: ∂r/∂z = 0/100 = 0
        self.assertAlmostEqual(H[0, 2], 0.0, places=5)
    
    def test_multiple_targets(self):
        """
        Test: Radar handles multiple targets
        """
        targets = [
            Target(position=np.array([100, 0, 100]), velocity=np.array([0, 0, 0])),
            Target(position=np.array([50, 50, 150]), velocity=np.array([5, 5, 0])),
            Target(position=np.array([200, 100, 80]), velocity=np.array([-5, 0, 0])),
        ]
        
        measurements = self.drone_radar.get_measurements(targets)
        
        # Should have one measurement per target
        self.assertEqual(len(measurements), 3)
        
        # Each measurement should have required fields
        for meas in measurements:
            self.assertIn('range', meas)
            self.assertIn('doppler', meas)
            self.assertIn('azimuth', meas)
            self.assertIn('elevation', meas)

class TestDroneIntegration(unittest.TestCase):
    
    def test_drone_has_radar(self):
        """Test: Drone can be equipped with radar"""
        radar_config = {'fc': 77e9}
        drone = Drone(drone_id=1, position=[0, 0, 50], velocity=[0, 0, 0],
                      radar_config=radar_config)
        
        self.assertIsNotNone(drone.radar)
    
    def test_drone_generates_measurements(self):
        """Test: Drone generates measurements"""
        radar_config = {'fc': 77e9, 'bandwidth': 4e9}
        drone = Drone(drone_id=1, position=[0, 0, 50], velocity=[0, 0, 0],
                      radar_config=radar_config)
        
        target = Target(position=[100, 0, 100], velocity=[10, 0, 0])
        
        measurements = drone.generate_measurements([target])
        
        self.assertGreater(len(measurements), 0)
        # Check metadata
        self.assertEqual(measurements[0]['sensor_id'], 1)

if __name__ == '__main__':
    unittest.main()
```

---

## Validation Checklist

### Before Integration:

- [ ] `DroneRadar` class passes all unit tests
- [ ] Relative motion correctly implemented (Eq 44-45)
- [ ] Jacobian matrices numerically verified (Eq 124-127)
- [ ] Multiple drones produce different measurements
- [ ] Noise model correctly applied
- [ ] Simulation loop runs without errors

### After Integration:

- [ ] Single drone radar matches single-radar Phase 2 behavior
- [ ] Multiple drones produce sensible different measurements
- [ ] Tracking with multi-sensor measurements works
- [ ] No numerical instabilities or NaNs
- [ ] Visualization shows drone positions and measurements

---

## Performance Metrics (Phase 3A)

| Metric | Target | Validation |
|--------|--------|------------|
| **Measurement Generation Time** | <10ms per drone | Benchmark |
| **Jacobian Computation Time** | <1ms | Benchmark |
| **Jacobian Numerical Accuracy** | <1e-5 error | Finite difference check |
| **Multi-drone Consistency** | No errors | N=3, N=5 tests |

---

## Dependencies & Blockers

✅ **Ready to Start:**
- Phase 1 (sensing) fully implemented
- Phase 2 (tracking) fully implemented
- Python environment configured

❌ **Potential Issues:**
- Coordinate frame transformations (orientation handling)
- Numerical stability of Jacobians (especially near singularities)
- Performance with many drones (N>10)

---

## Success Criteria

**Phase 3A is complete when:**

1. ✅ Each drone in swarm has independent `DroneRadar`
2. ✅ Measurements include: range, Doppler, azimuth, elevation, and metadata
3. ✅ Relative motion correctly implemented (Eq 44-45)
4. ✅ Jacobian matrices correctly computed (Eq 124-127)
5. ✅ All unit tests pass
6. ✅ Simulation runs with N=2,3,5 drones without errors
7. ✅ Measurements verified against theory

---

## Next Phase (3B) Dependency

Once Task 1-4 complete, Phase 3B is unblocked:

- Phase 3B needs: Per-drone measurements with Jacobians ✅
- Phase 3B adds: Track-level fusion logic (new module `fusion.py`)

---

**Estimated Total Time: 5-7 days**

**Start: ASAP | Complete By: May 1, 2026**
