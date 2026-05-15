#!/usr/bin/env python
"""
PHASE 3B VALIDATION V2: Fair comparison with detailed diagnostics
"""

import numpy as np
from target import Target
from drone import Drone
from fusion import TrackFusion, TrackBuilder

print("="*80)
print("PHASE 3B VALIDATION V2: Fair Comparison")
print("="*80)

targets_gt = [
    Target([200, 500, 400], [5, -2, 0]),
    Target([700, 200, 600], [-3, 4, 0])
]

drones = [
    Drone([450, 500, 0], drone_id=1, radar_enabled=True),
    Drone([550, 520, 0], drone_id=2, radar_enabled=True)
]

fusion_engine = TrackFusion(method='weighted_ls', max_correlation_distance=100.0)

DT = 0.1
NUM_STEPS = 50

errors_3a = []  # Error per measurement
errors_3b_per_target = []  # Error per target estimate

print(f"\nRunning {NUM_STEPS} steps...\n")

for step in range(NUM_STEPS):
    # Update ground truth
    for t_gt in targets_gt:
        t_gt.update(DT)
    
    # Get measurements from both drones
    measurements_d1 = drones[0].generate_measurements(targets_gt)
    measurements_d2 = drones[1].generate_measurements(targets_gt)
    
    # === PHASE 3A: Direct measurement positions ===
    measurements_3a = measurements_d1 + measurements_d2
    
    for meas in measurements_3a:
        world_pos = np.array([
            meas['sensor_position'][0] + meas['range'] * np.cos(meas['azimuth']),
            meas['sensor_position'][1] + meas['range'] * np.sin(meas['azimuth']),
            meas['sensor_position'][2] + meas['range'] * np.sin(meas['elevation'])
        ])
        
        # Find closest target
        min_error = min(np.linalg.norm(world_pos - t_gt.position) for t_gt in targets_gt)
        errors_3a.append(min_error)
    
    # === PHASE 3B: Fused track positions ===
    tracks_d1 = TrackBuilder.build_track_from_measurements(measurements_d1, drone_id=1) if measurements_d1 else []
    tracks_d2 = TrackBuilder.build_track_from_measurements(measurements_d2, drone_id=2) if measurements_d2 else []
    
    if tracks_d1 and tracks_d2:
        fused_tracks = fusion_engine.fuse_tracks([tracks_d1, tracks_d2])
        
        for fused in fused_tracks:
            min_error = min(np.linalg.norm(fused['position'] - t_gt.position) for t_gt in targets_gt)
            errors_3b_per_target.append(min_error)
            
            # Debug spike
            if min_error > 1000:
                print(f"SPIKE at step {step}: error={min_error:.1f}m")
                print(f"  Fused position: {fused['position']}")
                print(f"  Num sources: {fused['num_sources']}")
                for i, t_gt in enumerate(targets_gt):
                    print(f"  Target {i}: {t_gt.position}, error={np.linalg.norm(fused['position'] - t_gt.position):.1f}m")
    
    # Update drones for next step
    for d in drones:
        d.update(DT)
    
    if (step + 1) % 10 == 0:
        avg_3a_recent = np.mean(errors_3a[-40:])  # Last 40 measurements (4 per step * 10 steps)
        avg_3b_recent = np.mean(errors_3b_per_target[-20:]) if len(errors_3b_per_target) >= 20 else np.mean(errors_3b_per_target)
        print(f"Step {step+1:2d}: Phase 3A avg {avg_3a_recent:.1f}m (last 40 meas), Phase 3B avg {avg_3b_recent:.1f}m (last 20 tracks)")

print("\n" + "="*80)
print("RESULTS")
print("="*80)

print(f"\nPhase 3A (Raw measurements):")
print(f"  Total measurements: {len(errors_3a)}")
print(f"  Average error: {np.mean(errors_3a):.2f}m")
print(f"  Std dev: {np.std(errors_3a):.2f}m")
print(f"  Min/Max: {np.min(errors_3a):.2f}m / {np.max(errors_3a):.2f}m")

print(f"\nPhase 3B (Fused estimates):")
print(f"  Total fused tracks: {len(errors_3b_per_target)}")
print(f"  Average error: {np.mean(errors_3b_per_target):.2f}m")
print(f"  Std dev: {np.std(errors_3b_per_target):.2f}m")
print(f"  Min/Max: {np.min(errors_3b_per_target):.2f}m / {np.max(errors_3b_per_target):.2f}m")

# Fair comparison: errors per track/measurement
errors_3a_per_track = []
for i in range(0, len(errors_3a), 2):  # Group errors in pairs (2 meas per target)
    if i+1 < len(errors_3a):
        # Average the two measurements for same target from different drones
        avg_error = (errors_3a[i] + errors_3a[i+1]) / 2
        errors_3a_per_track.append(avg_error)

print(f"\n--- FAIR COMPARISON (per target) ---")
print(f"\nPhase 3A (averaged per target):")
print(f"  Average error: {np.mean(errors_3a_per_track):.2f}m")

print(f"\nPhase 3B (fused per target):")
print(f"  Average error: {np.mean(errors_3b_per_target):.2f}m")

improvement_fair = ((np.mean(errors_3a_per_track) - np.mean(errors_3b_per_target)) / np.mean(errors_3a_per_track) * 100)
print(f"\nImprovement: {improvement_fair:.1f}%")

if improvement_fair >= 30:
    print(f"✓ TARGET ACHIEVED: {improvement_fair:.1f}% >= 30%")
else:
    print(f"! Not yet at 30% goal: {improvement_fair:.1f}%")
    if improvement_fair > 0:
        print("  (But fusion is still improving accuracy)")
    else:
        print("  (Fusion is making things worse - diagnostics needed)")

print("\n" + "="*80)
