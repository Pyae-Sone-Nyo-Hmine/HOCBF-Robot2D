import numpy as np

def update_robot(state, robot, wedge, trace_points, ax, obstacles, ANGLE, sensing):
    x,y,theta,speed = state
    if sensing(robot, obstacles, 0, theta, ANGLE):
        return
    if len(trace_points) > 1:
        ax.plot(*zip(*trace_points), color="orange", linewidth=0.75)
    robot.center = (x,y)
    trace_points.append((x, y))

    start_angle = np.degrees(theta) - ANGLE / 2
    end_angle = start_angle + ANGLE

    wedge.set_center((x, y))
    wedge.theta1 = start_angle
    wedge.theta2 = end_angle

def draw_detection_area( Wedge,x, y, orientation, distance, angle, ax):
    # Calculate the start and end angles for the wedge
    start_angle = np.degrees(orientation) - angle / 2
    end_angle = start_angle + angle

    # Create a wedge from the start to the end angle with the specified distance
    wedge = Wedge(center=(x, y), r=distance, theta1=start_angle, theta2=end_angle, color='red', alpha=0.2)

    # Add the wedge to the plot
    wedge = ax.add_patch(wedge)

    return wedge