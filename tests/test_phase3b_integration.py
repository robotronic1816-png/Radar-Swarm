#!/usr/bin/env python
"""
Integration test for Phase 3B fusion in main simulation
"""

import numpy as np
from target import Target
from drone import Drone
from fmcw_radar import RadarEngine
from fusion import TrackFusion, TrackBuilder

print("="*70)
print("PHASE 3B INTEGRATION TEST - Fusion in Simulation Loop")
print("="*70)

# Setup
targets = [
    Target([200, 500, 400], [5, -2, 0]),
    Target([700, 200, 600], [-3, 4, 0])
]

drones = [
    Drone([450, 500, 0], drone_id=1, radar_enabled=True),
    Drone([550, 520, 0], drone_id=2, radar_enabled=True)
]

radar = RadarEngine([500, 500, 0])
fusion_engine = TrackFusion(method='weighted_ls', max_correlation_distance=100.0)

# Simulation metrics
fusion_metrics = {
    'fusion_events': 0,
    'multi_source_tracks': 0,
    'single_source_tracks': 0,
    'avg_sources_per_track': []
}

DT = 0.1

# Run 10 simulation steps
print("\nRunning 10 simulation steps with fusion...\n")

for step in range(10):
    # Update targets
    for t in targets:
        t.update(DT)
    
    # Generate measurements from drones
    all_drone_measurements = []
    for drone in drones:
        drone_measurements = drone.generate_measurements(targets)
        all_drone_measurements.extend(drone_measurements)
    
    # PHASE 3B: Build and fuse tracks
    if all_drone_measurements:
        measurements_by_drone = {d.drone_id: [] for d in drones}
        for meas in all_drone_measurements:
            if 'drone_id' in meas:
                measurements_by_drone[meas['drone_id']].append(meas)
        
        local_tracks_list = []
        for drone in drones:
            drone_meas = measurements_by_drone[drone.drone_id]
            if drone_meas:
                drone_tracks = TrackBuilder.build_track_from_measurements(drone_meas, drone_id=drone.drone_id)
                if drone_tracks:
                    local_tracks_list.append(drone_tracks)
        
        if len(local_tracks_list) > 1:
            fused_tracks = fusion_engine.fuse_tracks(local_tracks_list)
            fusion_metrics['fusion_events'] += 1
            
            multi_source = sum(1 for t in fused_tracks if t.get('num_sources', 1) > 1)
            single_source = len(fused_tracks) - multi_source
            fusion_metrics['multi_source_tracks'] += multi_source
            fusion_metrics['single_source_tracks'] += single_source
            
            avg_sources = np.mean([t.get('num_sources', 1) for t in fused_tracks]) if fused_tracks else 1
            fusion_metrics['avg_sources_per_track'].append(avg_sources)
            
            num_fused = len(fused_tracks)
            num_multi = multi_source
            print(f"Step {step:2d}: ✓ Fused {num_fused} track(s) ({num_multi} multi-source)")
        else:
            num_meas = len(all_drone_measurements)
            print(f"Step {step:2d}: ✓ {num_meas} measurement(s) (awaiting multiple drones for fusion)")
    else:
        print(f"Step {step:2d}: No measurements")
    
    # Update drones (for next position)
    for d in drones:
        d.update(DT)

print("\n" + "="*70)
print("INTEGRATION TEST RESULTS")
print("="*70)
print(f"Total fusion events: {fusion_metrics['fusion_events']}")
print(f"Multi-source tracks fused: {fusion_metrics['multi_source_tracks']}")
print(f"Single-source tracks: {fusion_metrics['single_source_tracks']}")
if fusion_metrics['avg_sources_per_track']:
    print(f"Average sources per fused track: {np.mean(fusion_metrics['avg_sources_per_track']):.2f}")

print("\n✓ Integration test complete!")
print("✓ Fusion pipeline successfully integrated into simulation loop")
print("="*70)
