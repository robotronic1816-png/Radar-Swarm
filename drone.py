import numpy as np
class Drone:
    def __init__(self, position, speed=40):
        self.position = np.array(position, dtype=float)
        self.speed = speed 
        self.target = None

    def assign_target(self, target):
        self.target = target

    def update(self, dt):
        if self.target is None or not self.target.active:
            return
        direction = self.target.position - self.position
        dist = np.linalg.norm(direction)
        if dist < 20:
            self.target.active = False
            self.target = None
            return
        direction = direction / dist
        self.position += direction * self.speed * dt
                
