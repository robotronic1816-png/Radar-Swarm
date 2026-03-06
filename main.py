import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from target import Target
from drone import Drone
from radar import Radar
from controller import assign_drones

WORLD_SIZE = 1000
DT = 0.1

targets = [
    Target([200,500,400], [5,-2,1]),
    Target([700,200,600], [-3,4,-1])
]
drones = [
    Drone([500,500,0]),
    Drone([520,520,0])
]

radar = Radar([500,500,0])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for step in range(1000):
    ax.clear()
    for t in targets:
        t.update(DT)
    detections = radar.scan(targets)
    assign_drones(drones, detections)
    for d in drones:
        d.update(DT)
    ax.scatter(*radar.position, c='green', s=100)
    for t in targets:
        if t.active:
            ax.scatter(*t.position, c='red')
    for d in drones:
        ax.scatter(*d.position, c='blue')        
    ax.set_xlim(0,WORLD_SIZE)
    ax.set_ylim(0,WORLD_SIZE)
    ax.set_zlim(0,WORLD_SIZE)

    plt.pause(0.01)
plt.show()        