from Sensing import sensing
import numpy as np

def execute(state, destination, obstacles, robot, ANGLE):
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
    obstacle_detected = sensing(robot, obstacles, 0.5, theta, ANGLE)
    if obstacle_detected:
        # Adjust the orientation away from the obstacle
        obs_x, obs_y = obstacle_detected.center
        angle_away_from_obs = np.arctan2(y - obs_y, x - obs_x)
        new_theta = theta + (angle_away_from_obs - theta) * 0.2
        
        # Recalculate position with adjusted orientation
        new_x = (x + new_speed * np.cos(new_theta))/2
        new_y = (y + new_speed * np.sin(new_theta))/2

    return [new_x/1.5, new_y/1.5, new_theta, new_speed]