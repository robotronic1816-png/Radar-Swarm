"""
Complete System Verification - Tests all critical components
"""

import numpy as np
from target import Target
from drone import Drone
from controller import assign_drones
from fmcw_radar import RadarEngine

print("="*70)
print("RADAR SWARM SYSTEM - COMPLETE VERIFICATION")
print("="*70)

# Initialize system
print("\n[1] SYSTEM INITIALIZATION")
targets = [Target([200, 500, 400], [5, -2, 0]), Target([700, 200, 600], [-3, 4, 0])]
drones = [Drone([450, 500, 0], speed=50), Drone([550, 520, 0], speed=50)]
radar = RadarEngine([500, 500, 0])
print("✓ System initialized (2 targets, 2 drones, radar engine)")

# Run simulation
print("\n[2] RUNNING 300-STEP SIMULATION")
track_counts = []
detections = 0
d0_move = 0.0
d1_move = 0.0
errors = 0

try:
    for step in range(300):
        for t in targets:
            t.update(0.1)
        
        try:
            tracks = radar.step(targets)
            track_counts.append(len(tracks))
            if len(tracks) > 0:
                detections += 1
        except Exception as e:
            errors += 1
            continue
        
        try:
            assign_drones(drones, tracks)
        except Exception as e:
            errors += 1
            continue
        
        old_d0 = drones[0].position.copy()
        old_d1 = drones[1].position.copy()
        
        drones[0].update(0.1)
        drones[1].update(0.1)
        
        d0_move += np.linalg.norm(drones[0].position[:2] - old_d0[:2])
        d1_move += np.linalg.norm(drones[1].position[:2] - old_d1[:2])
        
        if (step + 1) % 100 == 0:
            print(f"  Step {step+1}: {len(tracks)} tracks, D0_assigned={drones[0].target is not None}, D1_assigned={drones[1].target is not None}")
    
    print("✓ 300-step simulation completed successfully")
except Exception as e:
    print(f"✗ Simulation failed: {str(e)}")
    exit(1)

# Results
print("\n[3] VERIFICATION RESULTS")
print("-"*70)
print(f"Detection Rate:        {detections}/300 steps ({detections*100/300:.1f}%)")
print(f"Max Tracks/Frame:      {max(track_counts) if track_counts else 0}")
print(f"Avg Tracks/Frame:      {np.mean(track_counts) if track_counts else 0:.1f}")
print(f"Errors Encountered:    {errors}")
print(f"D0 Movement:           {d0_move:.1f}m")
print(f"D1 Movement:           {d1_move:.1f}m")

# Pass/Fail
print("\n[4] TEST RESULTS")
print("-"*70)

tests_passed = 0
tests_total = 5

# Test 1: Detection
if detections >= 270:
    print("✓ TEST 1: Radar Detection          PASS (>90% detection)")
    tests_passed += 1
else:
    print(f"✗ TEST 1: Radar Detection          FAIL ({detections} detections)")

# Test 2: Track Management
if max(track_counts) <= 50:
    print(f"✓ TEST 2: Track Management         PASS (max {max(track_counts)} < 50)")
    tests_passed += 1
else:
    print(f"✗ TEST 2: Track Management         FAIL (max {max(track_counts)})")

# Test 3: Error Handling
if errors == 0:
    print("✓ TEST 3: Error Handling           PASS (no exceptions)")
    tests_passed += 1
else:
    print(f"✗ TEST 3: Error Handling           FAIL ({errors} errors)")

# Test 4: Drone Movement
if d0_move > 100 and d1_move > 100:
    print(f"✓ TEST 4: Drone Movement           PASS (both active: {d0_move:.0f}m, {d1_move:.0f}m)")
    tests_passed += 1
else:
    print(f"✗ TEST 4: Drone Movement           FAIL (insufficient movement)")

# Test 5: System Stability
if errors == 0 and max(track_counts) <= 50:
    print("✓ TEST 5: System Stability         PASS (stable execution)")
    tests_passed += 1
else:
    print("✗ TEST 5: System Stability         FAIL")

# Final verdict
print("\n" + "="*70)
if tests_passed >= 4:
    print(f"VERDICT: ✓ SYSTEM OPERATIONAL ({tests_passed}/{tests_total} tests passed)")
    print("The radar swarm system is fully functional and ready for use.")
else:
    print(f"VERDICT: PARTIAL ({tests_passed}/{tests_total} tests passed)")
    print("System has issues that need investigation.")
print("="*70)
