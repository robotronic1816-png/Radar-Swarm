#!/usr/bin/env python
"""
Debug: Analyze track building and fusion
"""

import numpy as np
from target import Target
from drone import Drone
from fusion import TrackBuilder

print("="*80)
print("DEBUG: Track Building and Fusion")
print("="*80)

# Setup
target = Target([200, 500, 400], [5, -2, 0])
drone1 = Drone([450, 500, 0], drone_id=1, radar_enabled=True)
drone2 = Drone([550, 520, 0], drone_id=2, radar_enabled=True)

print(f"\nTarget true position: {target.position}")
print(f"Target true velocity: {target.velocity}")

# Generate one measurement
measurements_d1 = drone1.generate_measurements([target])
measurements_d2 = drone2.generate_measurements([target])

print(f"\n--- Drone 1 Measurement ---")
if measurements_d1:
    m1 = measurements_d1[0]
    print(f"Range: {m1['range']:.2f}m")
    print(f"Azimuth: {m1['azimuth']:.4f} rad ({np.degrees(m1['azimuth']):.2f}°)")
    print(f"Elevation: {m1['elevation']:.4f} rad ({np.degrees(m1['elevation']):.2f}°)")
    print(f"Doppler: {m1['doppler']:.2f} m/s")
    print(f"Sensor position: {m1['sensor_position']}")
    
    # Manual conversion
    r = m1['range']
    az = m1['azimuth']
    el = m1['elevation']
    sensor_pos = m1['sensor_position']
    
    x_rel = r * np.cos(el) * np.cos(az)
    y_rel = r * np.cos(el) * np.sin(az)
    z_rel = r * np.sin(el)
    
    world_x = sensor_pos[0] + x_rel
    world_y = sensor_pos[1] + y_rel
    world_z = sensor_pos[2] + z_rel
    
    print(f"Converted position: [{world_x:.2f}, {world_y:.2f}, {world_z:.2f}]")
    print(f"Distance to target: {np.linalg.norm(np.array([world_x, world_y, world_z]) - target.position):.2f}m")

print(f"\n--- Drone 2 Measurement ---")
if measurements_d2:
    m2 = measurements_d2[0]
    print(f"Range: {m2['range']:.2f}m")
    print(f"Azimuth: {m2['azimuth']:.4f} rad ({np.degrees(m2['azimuth']):.2f}°)")
    print(f"Elevation: {m2['elevation']:.4f} rad ({np.degrees(m2['elevation']):.2f}°)")
    print(f"Doppler: {m2['doppler']:.2f} m/s")
    print(f"Sensor position: {m2['sensor_position']}")
    
    # Manual conversion
    r = m2['range']
    az = m2['azimuth']
    el = m2['elevation']
    sensor_pos = m2['sensor_position']
    
    x_rel = r * np.cos(el) * np.cos(az)
    y_rel = r * np.cos(el) * np.sin(az)
    z_rel = r * np.sin(el)
    
    world_x = sensor_pos[0] + x_rel
    world_y = sensor_pos[1] + y_rel
    world_z = sensor_pos[2] + z_rel
    
    print(f"Converted position: [{world_x:.2f}, {world_y:.2f}, {world_z:.2f}]")
    print(f"Distance to target: {np.linalg.norm(np.array([world_x, world_y, world_z]) - target.position):.2f}m")

# Now test track building
print(f"\n--- TrackBuilder Output (Drone 1) ---")
tracks_d1 = TrackBuilder.build_track_from_measurements(measurements_d1, drone_id=1)
if tracks_d1:
    t1 = tracks_d1[0]
    print(f"Position: {t1['position']}")
    print(f"Velocity: {t1['velocity']}")
    print(f"Covariance shape: {t1['covariance'].shape}")
    print(f"Distance to target: {np.linalg.norm(t1['position'] - target.position):.2f}m")

print(f"\n--- TrackBuilder Output (Drone 2) ---")
tracks_d2 = TrackBuilder.build_track_from_measurements(measurements_d2, drone_id=2)
if tracks_d2:
    t2 = tracks_d2[0]
    print(f"Position: {t2['position']}")
    print(f"Velocity: {t2['velocity']}")
    print(f"Covariance shape: {t2['covariance'].shape}")
    print(f"Distance to target: {np.linalg.norm(t2['position'] - target.position):.2f}m")

# Test fusion
if tracks_d1 and tracks_d2:
    print(f"\n--- Fusion Test ---")
    from fusion import TrackFusion
    
    fusion = TrackFusion(method='weighted_ls')
    fused_tracks = fusion.fuse_tracks([tracks_d1, tracks_d2])
    
    if fused_tracks:
        fused = fused_tracks[0]
        print(f"Fused position: {fused['position']}")
        print(f"Fused velocity: {fused['velocity']}")
        print(f"Num sources: {fused['num_sources']}")
        print(f"Distance to target: {np.linalg.norm(fused['position'] - target.position):.2f}m")
