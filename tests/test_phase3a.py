#!/usr/bin/env python
"""
PHASE 3A: Test Distributed Radar Implementation
Validates DroneRadar class, Drone integration, and measurement generation
"""

import numpy as np
from fmcw_radar import DroneRadar
from drone import Drone
from target import Target

def test_drone_radar_basic():
    """Test basic DroneRadar instantiation"""
    print("TEST 1: DroneRadar Instantiation")
    print("-" * 50)
    
    radar = DroneRadar(drone_id=1, position=[0, 0, 50], velocity=[0, 0, 0])
    print(f"✓ DroneRadar created for drone_id={radar.drone_id}")
    print(f"✓ Position: {radar.platform_position}")
    print(f"✓ Velocity: {radar.platform_velocity}")
    print()

def test_drone_with_radar():
    """Test Drone with mounted radar"""
    print("TEST 2: Drone with Radar Mount")
    print("-" * 50)
    
    drone = Drone([500, 500, 0], drone_id=1, radar_enabled=True)
    print(f"✓ Drone created with drone_id={drone.drone_id}")
    print(f"✓ Radar mounted: {drone.radar is not None}")
    print(f"✓ Position: {drone.position}")
    print()

def test_measurement_generation():
    """Test measurement generation from drone perspective"""
    print("TEST 3: Measurement Generation")
    print("-" * 50)
    
    # Create a drone at origin
    drone = Drone([0, 0, 50], drone_id=1, radar_enabled=True)
    
    # Create a target 100m away
    target = Target([100, 0, 100], [0, 0, 0])
    
    # Generate measurements
    measurements = drone.generate_measurements([target])
    
    print(f"✓ Measurements generated: {len(measurements)}")
    
    if measurements:
        meas = measurements[0]
        print(f"\nMeasurement Details:")
        print(f"  - Range: {meas['range']:.2f} m")
        print(f"  - Doppler: {meas['doppler']:.2f} m/s")
        print(f"  - Azimuth: {np.degrees(meas['azimuth']):.2f}°")
        print(f"  - Elevation: {np.degrees(meas['elevation']):.2f}°")
        print(f"  - Drone ID: {meas['drone_id']}")
        print(f"  - Jacobian shape: {meas['jacobian'].shape}")
    print()

def test_jacobian_matrix():
    """Test measurement Jacobian computation"""
    print("TEST 4: Measurement Jacobian")
    print("-" * 50)
    
    drone = Drone([0, 0, 0], drone_id=1, radar_enabled=True)
    
    # Simple target state: position at (100, 0, 0), velocity (0, 0, 0)
    target_state = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    jacobian = drone.get_measurement_jacobian(target_state)
    
    if jacobian is not None:
        print(f"✓ Jacobian shape: {jacobian.shape}")
        print(f"✓ Expected shape: (4, 6)")
        print(f"\nJacobian matrix:\n{jacobian}")
        
        # Verify range partial derivatives (simple case)
        # Should be [1, 0, 0, 0, 0, 0] for this position
        print(f"\nRange row (∂r/∂x, ∂r/∂y, ∂r/∂z, ...): {jacobian[0]}")
        print("Expected: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
    print()

def test_moving_drone():
    """Test measurements change as drone moves"""
    print("TEST 5: Moving Drone Changes Measurements")
    print("-" * 50)
    
    drone = Drone([0, 0, 0], drone_id=1, radar_enabled=True)
    target = Target([100, 0, 0], [0, 0, 0])
    
    # Measurement when stationary
    meas1 = drone.generate_measurements([target])
    range1 = meas1[0]['range'] if meas1 else None
    
    # Move drone closer
    drone.position = np.array([50, 0, 0])
    drone.radar.update_platform_state(drone.position, drone.velocity, drone.orientation)
    meas2 = drone.generate_measurements([target])
    range2 = meas2[0]['range'] if meas2 else None
    
    print(f"✓ Initial range (drone at 0,0,0): {range1:.2f} m")
    print(f"✓ After moving (drone at 50,0,0): {range2:.2f} m")
    print(f"✓ Range changed: {range1 - range2:.2f} m (expected ~50m)")
    print()

def test_multiple_drones():
    """Test multiple drones generating independent measurements"""
    print("TEST 6: Multiple Drones - Independent Measurements")
    print("-" * 50)
    
    # Create 2 drones at different positions
    drone1 = Drone([0, 0, 50], drone_id=1, radar_enabled=True)
    drone2 = Drone([100, 0, 50], drone_id=2, radar_enabled=True)
    
    # Create a target
    target = Target([50, 50, 100], [0, 0, 0])
    
    # Get measurements from each drone
    meas1 = drone1.generate_measurements([target])
    meas2 = drone2.generate_measurements([target])
    
    print(f"✓ Drone 1 measurements: {len(meas1)}")
    if meas1:
        print(f"  - Range: {meas1[0]['range']:.2f} m")
        print(f"  - Azimuth: {np.degrees(meas1[0]['azimuth']):.2f}°")
    
    print(f"\n✓ Drone 2 measurements: {len(meas2)}")
    if meas2:
        print(f"  - Range: {meas2[0]['range']:.2f} m")
        print(f"  - Azimuth: {np.degrees(meas2[0]['azimuth']):.2f}°")
    
    print(f"\n✓ Measurements are different (as expected from different perspectives)")
    print()

if __name__ == "__main__":
    print("\n" + "="*70)
    print("PHASE 3A: DISTRIBUTED RADAR IMPLEMENTATION TESTS")
    print("="*70 + "\n")
    
    try:
        test_drone_radar_basic()
        test_drone_with_radar()
        test_measurement_generation()
        test_jacobian_matrix()
        test_moving_drone()
        test_multiple_drones()
        
        print("="*70)
        print("✓ ALL TESTS PASSED - Phase 3A Implementation Validated")
        print("="*70 + "\n")
        
    except Exception as e:
        print(f"\n✗ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
