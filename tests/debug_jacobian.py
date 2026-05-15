#!/usr/bin/env python
"""
Debug: Check Jacobian matrix conditioning and fusion stability
"""

import numpy as np
from target import Target
from drone import Drone
from fusion import TrackBuilder

print("="*80)
print("DEBUG: Jacobian Conditioning and Track Covariance")
print("="*80)

target = Target([200, 500, 400], [5, -2, 0])
drone1 = Drone([450, 500, 0], drone_id=1, radar_enabled=True)

# Generate measurements multiple times
print("\nGenerating 10 measurements and analyzing Jacobians:\n")

for step in range(10):
    target.update(0.1)
    
    measurements = drone1.generate_measurements([target])
    
    if measurements:
        meas = measurements[0]
        
        # Check Jacobian
        if 'jacobian' in meas:
            H = meas['jacobian']
            print(f"Step {step}: H shape = {H.shape}")
            print(f"  H rank: {np.linalg.matrix_rank(H)}")
            print(f"  H:\n{H[:2, :]}...")  # Print first 2 rows
            
            # Try the covariance computation
            R = np.eye(4)
            R[0, 0] = 0.5**2
            R[1, 1] = 0.05**2
            R[2, 2] = 0.05**2
            R[3, 3] = 0.3**2
            
            try:
                R_inv = np.linalg.inv(R)
                HTR = H.T @ R_inv @ H
                
                # Check condition number
                cond_number = np.linalg.cond(HTR)
                print(f"  H.T @ R_inv @ H condition number: {cond_number:.2e}")
                
                if cond_number < 1e10:
                    P_computed = np.linalg.inv(HTR)
                    print(f"  P computed successfully")
                    print(f"  P diagonal (position): [{P_computed[0,0]:.1f}, {P_computed[1,1]:.1f}, {P_computed[2,2]:.1f}]")
                    print(f"  P trace: {np.trace(P_computed):.1f}")
                else:
                    print(f"  WARNING: Matrix nearly singular, condition number too high!")
            except Exception as e:
                print(f"  ERROR: {e}")
    
    print()
