import matplotlib.pyplot as plt
import numpy as np

GRID_SIZE = 50
OBSTACLE_POSITIONS = [(30,20),(37,40),(22,34)]
DESTINATIONS = [(30+9, 20), (37+9, 40), (22, 34-9), (22+7, 44)]
obstacles, destinations = [],[]


def update_robot(x, y,robot, trace_points, ax, obstacles):
    if sensing(robot,obstacles,0):
        return
    if len(trace_points) > 1:
        ax.plot(*zip(*trace_points), color="orange", linewidth=0.75)
    robot.center = (x,y)
    trace_points.append((x, y))


def grid(size):
    
    # to run GUI event loop
    plt.ion()
    
    figure, ax = plt.subplots(figsize=(6, 6))

    
    plt.xlim(0, size)
    plt.ylim(0, size)
    ax.grid(True)
    plt.title("HOCBF Robot in 2D", fontsize=10)

    # Display obstacles on the grid
    for pos in OBSTACLE_POSITIONS:
        obstacle = plt.Circle(pos, 7, color='lightblue')
        obstacles.append(obstacle)
        ax.add_artist(obstacle)

    # Display destinations on the grid
    for dest in DESTINATIONS:
        destination = plt.Circle(dest, 1, color='green')
        destinations.append(destination)
        ax.add_artist(destination)
        ax.text(dest[0], dest[1], 'D', color='white', ha='center', va='center')

    return figure, ax

figure, ax = grid(GRID_SIZE)
robot = plt.Circle((0, 0), 1.5, color="orange")
ax.add_artist(robot)
trace_points = []


def sensing(robot, obstacles, sensing_distance):
    robot_center = robot.center
    robot_radius = robot.radius

    for obstacle in obstacles:
        obstacle_center = obstacle.center
        obstacle_radius = obstacle.radius

        # Calculate the distance between the centers of the robot and the obstacle
        distance = np.sqrt((robot_center[0] - obstacle_center[0])**2 + (robot_center[1] - obstacle_center[1])**2)

        # Adjust the collision check for sensing distance
        if distance < (robot_radius + obstacle_radius + sensing_distance):
            return obstacle

    return False

def execute(state, destination, obstacles):
    x, y, theta, speed = state
    dest_x, dest_y = destination

    # Basic motion planning logic
    # 1. Calculate the angle to the destination
    angle_to_dest = np.arctan2(dest_y - y, dest_x - x)

    # 2. Adjust the robot's orientation to face the destination
    new_theta = theta + (angle_to_dest - theta) * 0.1  # Adjust orientation gradually

    # 3. Calculate a basic speed towards the destination
    distance_to_dest = np.sqrt((dest_x - x) ** 2 + (dest_y - y) ** 2)
    new_speed = min(speed + 0.1, distance_to_dest)  # Increase speed, but cap it at distance to destination

    # 4. Update position based on new speed and orientation
    new_x = x + new_speed * np.cos(new_theta)
    new_y = y + new_speed * np.sin(new_theta)

    # Basic obstacle avoidance
    obstacle_detected = sensing(robot, obstacles, 0.5)
    if obstacle_detected:
        # Adjust the orientation away from the obstacle
        obs_x, obs_y = obstacle_detected.center
        angle_away_from_obs = np.arctan2(y - obs_y, x - obs_x)
        new_theta = theta + (angle_away_from_obs - theta) * 0.2
        
        # Recalculate position with adjusted orientation
        new_x = (x + new_speed * np.cos(new_theta))/2
        new_y = (y + new_speed * np.sin(new_theta))/2

    return [new_x/1.5, new_y/1.5, new_theta, new_speed]

i = 0
state = [0,0,0,0]
collision_message = ax.text(0.05, 0.95, "No Collision", transform=ax.transAxes, color='green')
log = ax.text(0.75, 0.95, str(state), transform=ax.transAxes, color='green')

while True:

    

    new_state = execute(state, DESTINATIONS[0], obstacles)
    new_x,new_y,theta,speed = new_state
    update_robot(new_x,new_y,robot,trace_points, ax, obstacles)
    state = new_state

    # Check for collisions
    if sensing(robot, obstacles,1):
        collision_message.remove()
        collision_message = ax.text(0.05, 0.95, "Collision detected!", transform=ax.transAxes, color='red')
    else:
        
        collision_message.remove()
        collision_message = ax.text(0.05, 0.95, "No Collision", transform=ax.transAxes, color='green')

    log.remove()
    log = ax.text(0.75, 0.95, str([round(x, 2) for x in state ]), transform=ax.transAxes, color='green')
    

    # drawing updated values
    figure.canvas.draw()

    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()

    plt.pause(0.1)
    i += 1