import numpy as np

def sensing(robot, obstacles, sensing_distance, robot_orientation, detection_angle):
    robot_center = robot.center
    robot_radius = robot.radius
    detection_angle_rad = np.radians(detection_angle)  # Convert to radians and halve (as it's spread across both sides)

    for obstacle in obstacles:
        obstacle_center = obstacle.center
        obstacle_radius = obstacle.radius

        # Calculate the distance between the centers of the robot and the obstacle
        center_distance = np.sqrt((robot_center[0] - obstacle_center[0])**2 + (robot_center[1] - obstacle_center[1])**2)

        # Adjust the distance for the radius of the obstacle and the robot
        edge_distance = center_distance - obstacle_radius - robot_radius

        if edge_distance < sensing_distance:
            angle_to_obstacle = np.arctan2(obstacle_center[1] - robot_center[1], obstacle_center[0] - robot_center[0])

            # Normalize angles to be between -pi and pi
            robot_orientation = np.mod(robot_orientation, 2*np.pi)
            if robot_orientation > np.pi:
                robot_orientation -= 2*np.pi

            # Check if the obstacle is within the detection angle of the robot's orientation
            angle_difference = np.abs(robot_orientation - angle_to_obstacle)
            if angle_difference > np.pi:
                angle_difference = 2*np.pi - angle_difference

            if angle_difference <= detection_angle_rad:
                return obstacle

    return False


