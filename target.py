import numpy as np 

class Target:
    def __init__(self, position, velocity):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.active = True

    def update(self, dt):
        if self.active:
            self.position += self.velocity * dt
