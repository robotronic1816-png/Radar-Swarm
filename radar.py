import numpy as np

class Radar:
    def __init__(self, position, detection_range=600):
        self.position = np.array(position)
        self.range = detection_range

    def scan(self, targets):
        detections = []
        for t in targets:
            if not t.active:
                continue
            distance = np.linalg.norm(t.position - self.position)
            if distance <= self.range:
                detections.append(t)
        return detections             