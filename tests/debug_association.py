#!/usr/bin/env python
"""
Debug track association in fusion
"""

import numpy as np
from target import Target
from drone import Drone
from fusion import TrackFusion, TrackBuilder

print("="*80)
print("DEBUG: Track Association During Fusion")
print("="*80)

# Setup - two targets at different locations
target1 = Target([200, 500, 400], [5, -2, 0])
target2 = Target([700, 200, 600], [-3, 4, 0])

drone1 = Drone([450, 500, 0], drone_id=1, radar_enabled=True)
drone2 = Drone([550, 520, 0], drone_id=2, radar_enabled=True)

fusion = TrackFusion(method='weighted_ls', max_correlation_distance=100.0)

print(f"\nTarget 1: position={target1.position}, velocity={target1.velocity}")
print(f"Target 2: position={target2.position}, velocity={target2.velocity}")

# Generate measurements
measurements_d1 = drone1.generate_measurements([target1, target2])
measurements_d2 = drone2.generate_measurements([target1, target2])

print(f"\nDrone 1 generated {len(measurements_d1)} measurements")
print(f"Drone 2 generated {len(measurements_d2)} measurements")

# Build tracks
tracks_d1 = TrackBuilder.build_track_from_measurements(measurements_d1, drone_id=1)
tracks_d2 = TrackBuilder.build_track_from_measurements(measurements_d2, drone_id=2)

print(f"\nDrone 1 built {len(tracks_d1)} tracks")
for i, t in enumerate(tracks_d1):
    print(f"  Track {i}: position={t['position']}, distance_to_t1={np.linalg.norm(t['position']-target1.position):.1f}m, distance_to_t2={np.linalg.norm(t['position']-target2.position):.1f}m")

print(f"\nDrone 2 built {len(tracks_d2)} tracks")
for i, t in enumerate(tracks_d2):
    print(f"  Track {i}: position={t['position']}, distance_to_t1={np.linalg.norm(t['position']-target1.position):.1f}m, distance_to_t2={np.linalg.norm(t['position']-target2.position):.1f}m")

# Test association
print(f"\n--- Testing Track Association ---")
print(f"Max correlation distance: {fusion.max_correlation_distance}m")

associations = fusion.associate_tracks([tracks_d1, tracks_d2])

print(f"\nFound {len(associations)} associations:")
for i, assoc in enumerate(associations):
    print(f"\nAssociation {i}: {len(assoc)} tracks")
    for j, track in enumerate(assoc):
        print(f"  Track {j} (drone {track['drone_id']}): position={track['position']}")
    
    # Try to identify which target this association represents
    avg_pos = np.mean([t['position'] for t in assoc], axis=0)
    dist_to_t1 = np.linalg.norm(avg_pos - target1.position)
    dist_to_t2 = np.linalg.norm(avg_pos - target2.position)
    closest_target = 1 if dist_to_t1 < dist_to_t2 else 2
    print(f"  Average position: {avg_pos}")
    print(f"  Closest to Target {closest_target} (dist={min(dist_to_t1, dist_to_t2):.1f}m)")

# Now fuse and check results
print(f"\n--- Fusing Associated Tracks ---")
fused_tracks = fusion.fuse_tracks([tracks_d1, tracks_d2])

print(f"\nFused {len(fused_tracks)} tracks:")
for i, fused in enumerate(fused_tracks):
    print(f"\nFused track {i}:")
    print(f"  Position: {fused['position']}")
    print(f"  Num sources: {fused['num_sources']}")
    print(f"  Distance to T1: {np.linalg.norm(fused['position']-target1.position):.1f}m")
    print(f"  Distance to T2: {np.linalg.norm(fused['position']-target2.position):.1f}m")
    
    closest_target = 1 if np.linalg.norm(fused['position']-target1.position) < np.linalg.norm(fused['position']-target2.position) else 2
    print(f"  Closest to Target {closest_target}")
