import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation  # NEW: Required for generating the GIF

from target import Target
from drone import Drone
from controller import assign_drones
from fmcw_radar import RadarEngine
from fusion import TrackFusion, TrackBuilder
from geometry import gdop_gradient_descent_step
from timesync import TimeSync

WORLD_SIZE = 1000
DT = 0.1
TOTAL_FRAMES = 200  # Number of frames for the GIF. Adjust for longer/shorter clips.

targets = [
    Target([200, 500, 400], [5, -2, 0]),
    Target([700, 200, 600], [-3, 4, 0])
]

# PHASE 3A: Create drones with radar sensors enabled
drones = [
    Drone([450, 500, 0], drone_id=1, radar_enabled=True),
    Drone([550, 520, 0], drone_id=2, radar_enabled=True)
]

# Optional: Keep ground radar for comparison (Phase 2 baseline)
radar = RadarEngine([500, 500, 0])

# PHASE 3B: Initialize track-level fusion engine
fusion_engine = TrackFusion(method='weighted_ls', max_correlation_distance=100.0)

# PHASE 3A: Track measurements from each drone
drone_measurements_history = {d.drone_id: [] for d in drones}

# PHASE 3B: Tracking metrics for comparison
fusion_metrics = {
    'fusion_events': 0,
    'multi_source_tracks': 0,
    'single_source_tracks': 0,
    'avg_sources_per_track': []
}

fig = plt.figure(figsize=(10, 8)) # Added figure size for better GIF resolution
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=25, azim=45)

def _measurement_track_from_polar(meas):
    """Convert a synchronized drone radar measurement into a track dict."""
    world_x = meas['sensor_position'][0] + meas['range'] * np.cos(meas['azimuth'])
    world_y = meas['sensor_position'][1] + meas['range'] * np.sin(meas['azimuth'])
    world_z = meas['sensor_position'][2] + meas['range'] * np.sin(meas['elevation'])
    azimuth = meas['azimuth']
    elevation = meas['elevation']
    radial_velocity = meas['doppler']

    velocity = radial_velocity * np.array([
        np.cos(elevation) * np.cos(azimuth),
        np.cos(elevation) * np.sin(azimuth),
        np.sin(elevation)
    ])

    return {
        'position': np.array([world_x, world_y, world_z], dtype=float),
        'velocity': velocity,
        'covariance': np.eye(6),
        'source_sensors': [meas.get('drone_id')],
        'num_sources': 1,
        'timestamp': meas.get('timestamp')
    }


def _sync_measurement_timestamps(measurements):
    """Replace local sensor timestamps with the post-consensus clock readings."""
    clock_by_id = {drone.drone_id: drone.clock.read() for drone in drones}
    for meas in measurements:
        sensor_id = meas.get('sensor_id', meas.get('drone_id'))
        if sensor_id in clock_by_id:
            meas['timestamp'] = clock_by_id[sensor_id]


def _build_fused_tracks(all_drone_measurements):
    """Phase 3B: build local drone tracks and fuse them into global tracks."""
    if not all_drone_measurements:
        return []

    measurements_by_drone = {d.drone_id: [] for d in drones}
    for meas in all_drone_measurements:
        drone_id = meas.get('drone_id')
        if drone_id in measurements_by_drone:
            measurements_by_drone[drone_id].append(meas)

    local_tracks_list = []
    for drone in drones:
        drone_meas = measurements_by_drone[drone.drone_id]
        if drone_meas:
            drone_tracks = TrackBuilder.build_track_from_measurements(
                drone_meas,
                drone_id=drone.drone_id
            )
            if drone_tracks:
                local_tracks_list.append(drone_tracks)

    if len(local_tracks_list) > 1:
        fused_tracks = fusion_engine.fuse_tracks(local_tracks_list)
        fusion_metrics['fusion_events'] += 1

        multi_source = sum(1 for track in fused_tracks if track.get('num_sources', 1) > 1)
        single_source = len(fused_tracks) - multi_source
        fusion_metrics['multi_source_tracks'] += multi_source
        fusion_metrics['single_source_tracks'] += single_source

        avg_sources = np.mean([track.get('num_sources', 1) for track in fused_tracks]) if fused_tracks else 1
        fusion_metrics['avg_sources_per_track'].append(avg_sources)
        return fused_tracks

    return [_measurement_track_from_polar(meas) for meas in all_drone_measurements]


def _fallback_ground_tracks():
    """Convert ground-radar fallback output into track dictionaries."""
    fallback_tracks = []
    for track in radar.step(targets):
        position = np.asarray(track[:3], dtype=float)
        if position.shape[0] < 3:
            position = np.pad(position, (0, 3 - position.shape[0]))
        speed = float(track[3]) if len(track) > 3 else 0.0
        fallback_tracks.append({
            'position': position[:3],
            'velocity': np.array([speed, 0.0, 0.0], dtype=float),
            'covariance': np.eye(6) * 10.0,
            'source_sensors': ['ground_radar'],
            'num_sources': 1
        })
    return fallback_tracks


def _apply_adaptive_geometry(tracks):
    """Phase 5A: move unassigned drones into GDOP-improving geometry."""
    unassigned = [drone for drone in drones if drone.target is None]
    if not unassigned:
        return

    geometry_targets = tracks if tracks else [target for target in targets if target.active]
    if not geometry_targets:
        return

    geometry_target = max(
        geometry_targets,
        key=lambda track: np.trace(track.get('covariance', np.eye(6))) if isinstance(track, dict) else 0.0
    )

    max_step = max(drone.speed for drone in drones) * DT
    reposition_vectors = gdop_gradient_descent_step(
        drones,
        geometry_target,
        step_size=5.0,
        learning_rate=20.0,
        max_vector_norm=max_step
    )

    for idx, drone in enumerate(drones):
        if drone.target is not None:
            continue

        move = reposition_vectors[idx]
        drone.position = np.clip(drone.position + move, 0.0, WORLD_SIZE)
        if DT > 0:
            drone.velocity = move / DT

        if drone.radar_enabled and drone.radar is not None:
            drone.radar.update_platform_state(
                position=drone.position,
                velocity=drone.velocity,
                orientation=drone.orientation
            )


def update(frame):
    """Core simulation loop wrapped in an animation update function."""
    ax.clear()
    ax.set_xlim(0, WORLD_SIZE)
    ax.set_ylim(0, WORLD_SIZE)
    ax.set_zlim(0, WORLD_SIZE)
    
    # 1. Update targets
    for t in targets:
        t.update(DT)

    # ========================================================================
    # PHASE 3C: Temporal Synchronization before sensing/fusion
    # ========================================================================
    if len(drones) > 1:
        adjacency = TimeSync.build_neighbor_graph(drones)
        TimeSync.synchronize_clocks(drones, adjacency, iterations=4, alpha=0.4)

    # ========================================================================
    # PHASE 3A: Generate synchronized measurements from EACH drone
    # ========================================================================
    all_drone_measurements = []
    for drone in drones:
        drone_measurements = drone.generate_measurements(targets)
        all_drone_measurements.extend(drone_measurements)
        drone_measurements_history[drone.drone_id].append(drone_measurements)

    _sync_measurement_timestamps(all_drone_measurements)

    # ========================================================================
    # PHASE 3B: Track-Level Fusion with synchronized measurements
    # ========================================================================
    tracks = _build_fused_tracks(all_drone_measurements)
    if not tracks:
        tracks = _fallback_ground_tracks()

    # ========================================================================
    # PHASE 4A + 4B: Hungarian assignment feeds PN target states to drones
    # ========================================================================
    assign_drones(drones, tracks)

    # ========================================================================
    # PHASE 4B + 5A: PN interception for assigned drones, GDOP geometry for idle
    # ========================================================================
    for drone in drones:
        drone.update(DT)

    _apply_adaptive_geometry(tracks)
    
    # ========================================================================
    # Visualization
    # ========================================================================
    
    ax.scatter(*radar.position, c='green', s=150, label="Ground Radar")
    
    for t in targets:
        if t.active:
            ax.scatter(*t.position, c='red', s=80, label="Target")
    
    for d in drones:
        # Plot drone with different color per drone
        color = ['blue', 'orange', 'purple', 'brown'][d.drone_id % 4]
        ax.scatter(*d.position, c=color, s=100, label=f"Drone {d.drone_id}")
        
        # Draw line from drone to target if assigned
        if d.target is not None:
            if isinstance(d.target, dict):
                target_pos = np.asarray(d.target.get('position', [0.0, 0.0, 0.0]), dtype=float)
            else:
                target_pos = np.asarray(d.target, dtype=float)
            if target_pos.shape[0] < 3:
                target_pos = np.pad(target_pos, (0, 3 - target_pos.shape[0]))
            ax.plot([d.position[0], target_pos[0]],
                   [d.position[1], target_pos[1]],
                   [d.position[2], target_pos[2]],
                   'k--', alpha=0.3)
    
    # --- SAFE LEGEND HANDLER ---
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        # Only show unique labels
        unique_labels = []
        unique_handles = []
        seen = set()
        for handle, label in zip(handles, labels):
            if label not in seen:
                unique_labels.append(label)
                unique_handles.append(handle)
                seen.add(label)
        ax.legend(unique_handles, unique_labels, loc="upper right")
    
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Altitude (m)")
    
    # Count drones actively chasing targets
    drones_chasing = sum(1 for d in drones if d.target is not None)
    
    # PHASE 3A: Show measurement count from drones
    total_drone_meas = sum(len(m) for m in all_drone_measurements)
    
    fusion_status = f"Fusion Events: {fusion_metrics['fusion_events']}" if fusion_metrics['fusion_events'] > 0 else "Awaiting Fusion"
    
    ax.set_title(f"Radar Swarm (Phase 3B) - Step {frame} | "
                 f"Chasing: {drones_chasing}/{len(drones)} | "
                 f"Drone Meas: {total_drone_meas} | "
                 f"Fused Tracks: {len(tracks)} | {fusion_status}")

    return fig,

# ============================================================================
# Execute Animation and Save to GIF
# ============================================================================
print(f"Simulating {TOTAL_FRAMES} frames and rendering GIF... This will take a moment.")
ani = animation.FuncAnimation(fig, update, frames=TOTAL_FRAMES, interval=50, blit=False)

# Save as GIF
output_filename = 'radar_swarm_phase3.gif'
ani.save(output_filename, writer='pillow', fps=20)
print(f"✅ GIF successfully saved as '{output_filename}'!")

# ============================================================================
# Analysis and summary
# ============================================================================
print("\n" + "="*70)
print("PHASE 3A + 3B: Distributed Radar + Track-Level Fusion Summary")
print("="*70)
print(f"Total steps simulated: {TOTAL_FRAMES}")
print(f"Number of drones: {len(drones)}")
print(f"Number of targets: {len(targets)}")

for drone in drones:
    total_measurements = sum(len(m) for m in drone_measurements_history[drone.drone_id])
    print(f"\nDrone {drone.drone_id}:")
    print(f"  - Total measurements generated: {total_measurements}")
    print(f"  - Average per timestep: {total_measurements / TOTAL_FRAMES:.2f}")

print("\n" + "-"*70)
print("Phase 3B - Track-Level Fusion Metrics:")
print("-" * 70)
print(f"Total fusion events: {fusion_metrics['fusion_events']}")
print(f"Multi-source tracks fused: {fusion_metrics['multi_source_tracks']}")
print(f"Single-source tracks: {fusion_metrics['single_source_tracks']}")
if fusion_metrics['avg_sources_per_track']:
    print(f"Average sources per fused track: {np.mean(fusion_metrics['avg_sources_per_track']):.2f}")

print("\nPhase 3A + 3B complete! Distributed radar with track-level fusion operational.")
print("="*70)
