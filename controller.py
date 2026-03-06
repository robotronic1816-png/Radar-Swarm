import numpy as np

def assign_drones(drones, targets):
    available_targets = [t for t in targets if t.active]
    for drone in drones:
        if drone.target is None and available_targets:
            closest = min(available_targets, key=lambda t: np.linalg.norm(t.position - drone.position)
                          )
            drone.assign_target(closest)
            available_targets.remove(closest) 