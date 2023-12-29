import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
import numpy as np
from Map import grid
from Robot import update_robot, draw_detection_area
from Sensing import sensing
from Execute import execute

GRID_SIZE = 50
OBSTACLE_POSITIONS = [(30,20),(37,40),(22,34)]
DESTINATIONS = [(30+9, 20), (37+9, 40), (22, 34-9), (22+7, 44)]
obstacles, destinations, trace_points = [],[], []
ANGLE = 90
detect = 5
state = [0,0,0,0]
i = 0

figure, ax = grid(GRID_SIZE, OBSTACLE_POSITIONS, DESTINATIONS, obstacles, destinations)
robot = plt.Circle((0, 0), 1.5, color="orange")
wedge = draw_detection_area(Wedge, 0, 0, 0, 1.5 + detect, ANGLE, ax)
collision_message = ax.text(0.05, 0.95, "No Collision", transform=ax.transAxes, color='green')
log = ax.text(0.75, 0.95, str(state), transform=ax.transAxes, color='green')
ax.add_artist(robot)

coe = [0.7535,1.0046,0.6664,1.0267]

while True:

    new_state = execute(state, DESTINATIONS[0], obstacles, robot, coe)
    new_x,new_y,theta,speed = new_state
    update_robot(new_state,robot, wedge,trace_points, ax, obstacles,ANGLE, sensing)
    state = new_state

    # Check for collisions
    if sensing(robot, obstacles, detect, theta, ANGLE):
        collision_message.remove()
        collision_message = ax.text(0.05, 0.95, "Collision detected!", transform=ax.transAxes, color='red')
    else:
        
        collision_message.remove()
        collision_message = ax.text(0.05, 0.95, "No Collision", transform=ax.transAxes, color='green')

    log.remove()
    log = ax.text(0.65, 0.97, str([round(x, 2) for x in state ]), transform=ax.transAxes, color='green')
    

    # drawing updated values
    figure.canvas.draw()

    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()

    plt.pause(0.1)
    i += 1