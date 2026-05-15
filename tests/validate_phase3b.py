#!/usr/bin/env python
"""
PHASE 3B VALIDATION: Tracking Accuracy Comparison
Compares Phase 3A (distributed radar) vs Phase 3B (distributed radar + fusion)
"""

import numpy as np
from target import Target
from drone import Drone
from fmcw_radar import RadarEngine
from fusion import TrackFusion, TrackBuilder

print("="*80)
print("PHASE 3B VALIDATION: Tracking Accuracy Improvement")
print("="*80)
print("\nComparing Phase 3A (No Fusion) vs Phase 3B (With Fusion)")
print("-"*80)

# Setup
targets = [
    Target([200, 500, 400], [5, -2, 0]),
    Target([700, 200, 600], [-3, 4, 0])
]

# Initialize drones for both modes
drones_3a = [
    Drone([450, 500, 0], drone_id=1, radar_enabled=True),
    Drone([550, 520, 0], drone_id=2, radar_enabled=True)
]

drones_3b = [
    Drone([450, 500, 0], drone_id=1, radar_enabled=True),
    Drone([550, 520, 0], drone_id=2, radar_enabled=True)
]

# Ground truth targets (for comparison)
targets_ground_truth = [
    Target([200, 500, 400], [5, -2, 0]),
    Target([700, 200, 600], [-3, 4, 0])
]

# Initialize fusion engine for Phase 3B
fusion_engine = TrackFusion(method='weighted_ls', max_correlation_distance=100.0)

DT = 0.1
NUM_STEPS = 50

# Storage for results
results_3a = {'rmse': [], 'tracks_per_step': []}
results_3b = {'rmse': [], 'tracks_per_step': []}

print(f"\nSimulation parameters:")
print(f"  - Duration: {NUM_STEPS} steps ({NUM_STEPS * DT:.1f} seconds)")
print(f"  - Number of targets: {len(targets)}")
print(f"  - Number of drones: {len(drones_3a)}")
print(f"  - Fusion method: Weighted Least Squares (WLS)")

# ============================================================================
# PHASE 3A: Distributed Radar WITHOUT Fusion (Baseline)
# ============================================================================
print("\n" + "="*80)
print("PHASE 3A: Distributed Radar (No Fusion) - 50 steps")
print("="*80)

for step in range(NUM_STEPS):
    # Update targets
    for t in targets:
        t.update(DT)
    
    # Update ground truth
    for t_gt in targets_ground_truth:
        t_gt.update(DT)
    
    # Generate measurements from drones (no fusion)
    all_measurements = []
    for drone in drones_3a:
        measurements = drone.generate_measurements(targets)
        all_measurements.extend(measurements)
    
    # Convert to simple track list (convert measurements to positions)
    tracks_3a = []
    for meas in all_measurements:
        world_x = meas['sensor_position'][0] + meas['range'] * np.cos(meas['azimuth'])
        world_y = meas['sensor_position'][1] + meas['range'] * np.sin(meas['azimuth'])
        world_z = meas['sensor_position'][2] + meas['range'] * np.sin(meas['elevation'])
        tracks_3a.append(np.array([world_x, world_y, world_z]))
    
    # Update drones
    for d in drones_3a:
        d.update(DT)
    
    # Calculate RMSE vs ground truth
    if tracks_3a:
        rmse = 0.0
        for track in tracks_3a:
            # Find closest ground truth target
            min_dist = min(np.linalg.norm(track - t_gt.position) for t_gt in targets_ground_truth)
            rmse += min_dist ** 2
        rmse = np.sqrt(rmse / len(tracks_3a))
        results_3a['rmse'].append(rmse)
        results_3a['tracks_per_step'].append(len(tracks_3a))
    
    if (step + 1) % 10 == 0:
        print(f"  Step {step+1:2d}/{NUM_STEPS}: {len(tracks_3a)} measurements, RMSE: {rmse:.2f}m")

avg_rmse_3a = np.mean(results_3a['rmse']) if results_3a['rmse'] else 0
print(f"\nPhase 3A Summary:")
print(f"  - Average RMSE: {avg_rmse_3a:.2f}m")
print(f"  - Average measurements/step: {np.mean(results_3a['tracks_per_step']):.1f}")
print(f"  - Min/Max RMSE: {min(results_3a['rmse']):.2f}m / {max(results_3a['rmse']):.2f}m")

# Reset targets for Phase 3B
for t in targets:
    t.position = np.array([200.0 if t.position[0] < 500 else 700.0,
                           500.0 if t.position[0] < 500 else 200.0,
                           400.0 if t.position[0] < 500 else 600.0], dtype=float)
    t.velocity = np.array([5.0 if t.position[0] < 500 else -3.0,
                           -2.0 if t.position[0] < 500 else 4.0,
                           0.0], dtype=float)

for t_gt in targets_ground_truth:
    t_gt.position = np.array([200.0 if t_gt.position[0] < 500 else 700.0,
                              500.0 if t_gt.position[0] < 500 else 200.0,
                              400.0 if t_gt.position[0] < 500 else 600.0], dtype=float)
    t_gt.velocity = np.array([5.0 if t_gt.position[0] < 500 else -3.0,
                              -2.0 if t_gt.position[0] < 500 else 4.0,
                              0.0], dtype=float)

# ============================================================================
# PHASE 3B: Distributed Radar WITH Fusion
# ============================================================================
print("\n" + "="*80)
print("PHASE 3B: Distributed Radar + Track-Level Fusion - 50 steps")
print("="*80)

for step in range(NUM_STEPS):
    # Update targets
    for t in targets:
        t.update(DT)
    
    # Update ground truth
    for t_gt in targets_ground_truth:
        t_gt.update(DT)
    
    # Generate measurements from drones
    all_measurements = []
    for drone in drones_3b:
        measurements = drone.generate_measurements(targets)
        all_measurements.extend(measurements)
    
    # Group by drone and build tracks for fusion
    if all_measurements:
        measurements_by_drone = {d.drone_id: [] for d in drones_3b}
        for meas in all_measurements:
            if 'drone_id' in meas:
                measurements_by_drone[meas['drone_id']].append(meas)
        
        # Build local tracks
        local_tracks_list = []
        for drone in drones_3b:
            drone_meas = measurements_by_drone[drone.drone_id]
            if drone_meas:
                drone_tracks = TrackBuilder.build_track_from_measurements(drone_meas, drone_id=drone.drone_id)
                if drone_tracks:
                    local_tracks_list.append(drone_tracks)
        
        # Fuse tracks
        tracks_3b = []
        if len(local_tracks_list) > 1:
            fused_tracks = fusion_engine.fuse_tracks(local_tracks_list)
            tracks_3b = [t['position'] for t in fused_tracks]
        else:
            # Single drone - use measurements directly
            for meas in all_measurements:
                world_x = meas['sensor_position'][0] + meas['range'] * np.cos(meas['azimuth'])
                world_y = meas['sensor_position'][1] + meas['range'] * np.sin(meas['azimuth'])
                world_z = meas['sensor_position'][2] + meas['range'] * np.sin(meas['elevation'])
                tracks_3b.append(np.array([world_x, world_y, world_z]))
    else:
        tracks_3b = []
    
    # Update drones
    for d in drones_3b:
        d.update(DT)
    
    # Calculate RMSE vs ground truth
    if tracks_3b:
        rmse = 0.0
        for track in tracks_3b:
            # Find closest ground truth target
            min_dist = min(np.linalg.norm(track - t_gt.position) for t_gt in targets_ground_truth)
            rmse += min_dist ** 2
        rmse = np.sqrt(rmse / len(tracks_3b))
        results_3b['rmse'].append(rmse)
        results_3b['tracks_per_step'].append(len(tracks_3b))
    
    if (step + 1) % 10 == 0:
        print(f"  Step {step+1:2d}/{NUM_STEPS}: {len(tracks_3b)} fused tracks, RMSE: {rmse:.2f}m")

avg_rmse_3b = np.mean(results_3b['rmse']) if results_3b['rmse'] else 0
print(f"\nPhase 3B Summary:")
print(f"  - Average RMSE: {avg_rmse_3b:.2f}m")
print(f"  - Average fused tracks/step: {np.mean(results_3b['tracks_per_step']):.1f}")
print(f"  - Min/Max RMSE: {min(results_3b['rmse']):.2f}m / {max(results_3b['rmse']):.2f}m")

# ============================================================================
# COMPARISON & VALIDATION
# ============================================================================
print("\n" + "="*80)
print("ACCURACY IMPROVEMENT ANALYSIS")
print("="*80)

improvement = ((avg_rmse_3a - avg_rmse_3b) / avg_rmse_3a * 100) if avg_rmse_3a > 0 else 0

print(f"\nPhase 3A (No Fusion):")
print(f"  - Average RMSE: {avg_rmse_3a:.2f}m")

print(f"\nPhase 3B (With Fusion):")
print(f"  - Average RMSE: {avg_rmse_3b:.2f}m")

print(f"\nImprovement:")
print(f"  - RMSE Reduction: {avg_rmse_3a - avg_rmse_3b:.2f}m")
print(f"  - Percentage Improvement: {improvement:.1f}%")

# Target validation
target_improvement = 30.0  # Goal: 30% improvement
if improvement >= target_improvement:
    print(f"\n✓ TARGET ACHIEVED: {improvement:.1f}% >= {target_improvement}% improvement goal")
    print("✓ Phase 3B validation SUCCESSFUL")
else:
    print(f"\n! Target improvement {target_improvement}% not yet met: {improvement:.1f}%")
    print("  (This may improve with more targets or longer simulation)")

print("\n" + "="*80)
print("PHASE 3B VALIDATION COMPLETE")
print("="*80)
