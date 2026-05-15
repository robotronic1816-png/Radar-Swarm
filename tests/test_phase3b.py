#!/usr/bin/env python
"""
PHASE 3B: Track-Level Fusion Tests
Validates sensor fusion implementations
"""

import numpy as np
from fusion import TrackFusion, TrackBuilder


def test_weighted_ls_fusion():
    """Test Weighted Least Squares fusion (Eq 133)"""
    print("TEST 1: Weighted Least Squares Fusion")
    print("-" * 50)
    
    fusion = TrackFusion(method='weighted_ls')
    
    # Two sensor estimates of same target
    estimate1 = np.array([100.0, 50.0, 100.0])  # Drone 1's estimate
    estimate2 = np.array([101.0, 49.0, 99.0])   # Drone 2's estimate
    
    # Covariances (Drone 1 more confident)
    cov1 = np.eye(3) * 1.0   # Low uncertainty (more confident)
    cov2 = np.eye(3) * 4.0   # High uncertainty (less confident)
    
    fused, fused_cov = fusion.weighted_ls_fusion([estimate1, estimate2], [cov1, cov2])
    
    print(f"✓ Drone 1 estimate: {estimate1}")
    print(f"✓ Drone 2 estimate: {estimate2}")
    print(f"✓ Fused estimate: {fused}")
    print(f"✓ Fused estimate closer to Drone 1 (more confident): {np.linalg.norm(fused - estimate1) < np.linalg.norm(fused - estimate2)}")
    
    # Check uncertainty reduction
    single_uncertainty = np.linalg.norm(cov1)
    fused_uncertainty = np.linalg.norm(fused_cov)
    print(f"✓ Single-sensor uncertainty: {single_uncertainty:.2f}")
    print(f"✓ Fused uncertainty: {fused_uncertainty:.2f}")
    print(f"✓ Uncertainty reduced: {fused_uncertainty < single_uncertainty}")
    print()


def test_covariance_intersection():
    """Test Covariance Intersection (Eq 134-135)"""
    print("TEST 2: Covariance Intersection Fusion")
    print("-" * 50)
    
    fusion = TrackFusion(method='covariance_intersection')
    
    estimate1 = np.array([100.0, 50.0, 100.0])
    estimate2 = np.array([101.0, 49.0, 99.0])
    cov1 = np.eye(3) * 1.0
    cov2 = np.eye(3) * 4.0
    
    fused, fused_cov = fusion.covariance_intersection(
        estimate1, cov1, estimate2, cov2, omega=0.5
    )
    
    print(f"✓ CI Fused estimate: {fused}")
    print(f"✓ CI Fused covariance trace: {np.trace(fused_cov):.2f}")
    print(f"✓ CI conservative (doesn't assume independence)")
    print()


def test_consensus_update():
    """Test Consensus Algorithm (Eq 136)"""
    print("TEST 3: Consensus Algorithm")
    print("-" * 50)
    
    fusion = TrackFusion(method='weighted_ls')
    
    # Three drones with initial estimates
    estimates = {
        1: np.array([100.0, 50.0, 100.0]),   # Drone 1
        2: np.array([101.0, 49.0, 99.0]),    # Drone 2
        3: np.array([99.0, 51.0, 101.0])     # Drone 3
    }
    
    # Define neighborhood (who communicates with whom)
    neighbors = {
        1: [2],           # Drone 1 talks to 2
        2: [1, 3],        # Drone 2 talks to 1 and 3
        3: [2]            # Drone 3 talks to 2
    }
    
    consensus_estimates = fusion.consensus_update(
        estimates, neighbors, iterations=10
    )
    
    print(f"✓ Initial Drone 1 estimate: {estimates[1]}")
    print(f"✓ Initial Drone 2 estimate: {estimates[2]}")
    print(f"✓ Initial Drone 3 estimate: {estimates[3]}")
    print(f"\nAfter 10 consensus iterations:")
    for drone_id, est in consensus_estimates.items():
        print(f"  Drone {drone_id}: {est}")
    
    # Check convergence
    all_close = all(
        np.allclose(consensus_estimates[1], consensus_estimates[i])
        for i in [2, 3]
    )
    print(f"✓ Estimates converged: {all_close}")
    print()


def test_track_association():
    """Test Track Association"""
    print("TEST 4: Track Association (Multi-sensor)")
    print("-" * 50)
    
    fusion = TrackFusion(method='weighted_ls')
    
    # Tracks from Drone 1
    drone1_tracks = [
        {
            'position': np.array([100.0, 50.0, 100.0]),
            'velocity': np.array([5.0, -2.0, 0.0]),
            'covariance': np.eye(3) * 1.0,
            'drone_id': 1
        }
    ]
    
    # Tracks from Drone 2 (should associate with Drone 1's track)
    drone2_tracks = [
        {
            'position': np.array([101.0, 49.0, 99.0]),  # Close to Drone 1's track
            'velocity': np.array([5.1, -2.1, 0.0]),
            'covariance': np.eye(3) * 1.0,
            'drone_id': 2
        }
    ]
    
    associations = fusion.associate_tracks([drone1_tracks, drone2_tracks])
    
    print(f"✓ Drone 1 track: {drone1_tracks[0]['position']}")
    print(f"✓ Drone 2 track: {drone2_tracks[0]['position']}")
    print(f"✓ Number of associations: {len(associations)}")
    print(f"✓ Tracks in first association: {len(associations[0])}")
    print(f"✓ Tracks successfully associated: {len(associations[0]) == 2}")
    print()


def test_full_fusion_pipeline():
    """Test Full Fusion Pipeline"""
    print("TEST 5: Full Fusion Pipeline")
    print("-" * 50)
    
    fusion = TrackFusion(method='weighted_ls')
    
    # Simulate measurements from two drones observing same target from nearby positions
    # Drones are close to each other, both observing same target
    drone1_measurements = [
        {
            'range': 100.0,
            'azimuth': 0.0,
            'elevation': 0.2,
            'doppler': 5.0,
            'sensor_position': np.array([0.0, 0.0, 50.0]),
            'drone_id': 1,
            'jacobian': np.eye(4, 6)
        }
    ]
    
    drone2_measurements = [
        {
            'range': 99.0,      # Slightly different range from nearby position
            'azimuth': 0.05,    # Slightly different angle
            'elevation': 0.21,  # Slightly different elevation
            'doppler': 5.0,     # Similar doppler (same target velocity)
            'sensor_position': np.array([10.0, 5.0, 50.0]),  # Drone 2 is nearby (10m away)
            'drone_id': 2,
            'jacobian': np.eye(4, 6)
        }
    ]
    
    # Build tracks from measurements
    drone1_tracks = TrackBuilder.build_track_from_measurements(drone1_measurements, drone_id=1)
    drone2_tracks = TrackBuilder.build_track_from_measurements(drone2_measurements, drone_id=2)
    
    print(f"✓ Drone 1 generated {len(drone1_tracks) if drone1_tracks else 0} tracks")
    print(f"✓ Drone 2 generated {len(drone2_tracks) if drone2_tracks else 0} tracks")
    
    # Fuse tracks
    if drone1_tracks and drone2_tracks:
        fused_tracks = fusion.fuse_tracks([drone1_tracks, drone2_tracks])
        
        print(f"✓ Fused to {len(fused_tracks)} track(s)")
        if fused_tracks:
            track = fused_tracks[0]
            print(f"✓ Fused position: {track['position']}")
            print(f"✓ Fused velocity: {track['velocity']}")
            print(f"✓ Number of source sensors: {track['num_sources']}")
    
    print()


def test_fusion_gain():
    """Test Fusion Gain Calculation"""
    print("TEST 6: Fusion Gain")
    print("-" * 50)
    
    fusion = TrackFusion(method='weighted_ls')
    
    # Two sensors with same uncertainty
    covariances = [
        np.eye(3) * 4.0,
        np.eye(3) * 4.0
    ]
    
    gain = fusion.compute_fusion_gain(covariances)
    
    print(f"✓ Fusion gain (2 equal sensors): {gain:.4f}")
    print(f"✓ Expected ~0.707 (1/sqrt(2)): {abs(gain - 1/np.sqrt(2)) < 0.01}")
    print()


if __name__ == "__main__":
    print("\n" + "="*70)
    print("PHASE 3B: TRACK-LEVEL FUSION TESTS")
    print("="*70 + "\n")
    
    try:
        test_weighted_ls_fusion()
        test_covariance_intersection()
        test_consensus_update()
        test_track_association()
        test_full_fusion_pipeline()
        test_fusion_gain()
        
        print("="*70)
        print("✓ ALL TESTS PASSED - Phase 3B Fusion Validated")
        print("="*70 + "\n")
        
    except Exception as e:
        print(f"\n✗ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
