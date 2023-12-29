from Sensing import sensing
import numpy as np
from scipy.integrate import solve_ivp # solve diff eq
from scipy.optimize import minimize # qp
from Dynamics import dynamics

detect = 5
ANGLE = 90
OBSTACLE_POSITIONS = [(30,20),(37,40),(22,34)]

def execute(state, dst, obstacles, robot, coe):
    global u, b_c, set1, set2, theta_d, pobsA, pobsB, pobsC

    x, y, theta, speed = state
    px, py = dst
    radius = 7
    p1, q1, p2, q2 = coe
    u_max, u_min = 0.2, -0.2
    a_max, a_min = 0.5, -0.5
    v_max, v_min = 2, 0

    # calculate dist_dst and theta

    dist_dst = np.sqrt((x - px)**2 + (y - py)**2)
    eps = 10
    psc = 1

    if theta < -np.pi:
        theta = np.pi
        state[2] = np.pi

    if theta > np.pi:
        theta = -np.pi
        state[2] = -np.pi

    theta_d = np.arctan2(py - y, px - x)

    if theta < 0 and theta > -np.pi and theta_d >= np.pi + theta and theta_d <= np.pi:
        theta_d = -1.5 * np.pi

    if theta > 0 and theta < np.pi and theta_d <= -np.pi + theta and theta_d >= -np.pi:
        theta_d = 1.5 * np.pi

    # calculate V, LfV, LgV, b_V

    V = (theta - theta_d)**2
    LfV = 0
    LgV = 2 * (theta - theta_d)
    b_V = -LfV - eps * V

    # Obs A avoidance

    nx, ny = OBSTACLE_POSITIONS[0]
    dist = np.sqrt((x - nx)**2 + (y - ny)**2)

    if sensing(robot, [obstacles[0]], detect, theta, ANGLE):
        pobsA_color = 'red'  # Assuming pobsA is an object with a Color attribute in your Python code
        b = dist - radius
        b_dot = ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta)) / dist
        LgLfb = ((y - ny) * speed * np.cos(theta) - (x - nx) * speed * np.sin(theta)) / dist
        LgLfb2 = ((x - nx) * np.cos(theta) + (y - ny) * np.sin(theta)) / dist
        Lf2b = (speed**2 * dist**2 - ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta))**2) / dist**3
        A_safe = -LgLfb
        A_safeu2 = -LgLfb2
        A_safe = [A_safe, A_safeu2, 0, 0]
        psi_1 = b_dot + p1 * b**q1
        b_safe = Lf2b + p1 * q1 * b**(q1 - 1) * b_dot + p2 * psi_1**q2
        set1, set2 = b, psi_1
    else:
        pobsA_color = 'cyan'  # Assuming pobsA is an object with a Color attribute in your Python code
        A_safe = []
        b_safe = []

    # Obs B avoidance
        
    nx, ny = OBSTACLE_POSITIONS[1]
    dist = np.sqrt((x - nx)**2 + (y - ny)**2)  

    if sensing(robot, [obstacles[1]], detect, theta, ANGLE):
        pobsB_color = 'red'  # Assuming pobsB is an object with a Color attribute
        radius = 6  # Overriding the radius value for obstacle B
        b = dist - radius
        b_dot = ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta)) / dist
        LgLfb = ((y - ny) * speed * np.cos(theta) - (x - nx) * speed * np.sin(theta)) / dist
        LgLfb2 = ((x - nx) * np.cos(theta) + (y - ny) * np.sin(theta)) / dist
        Lf2b = (speed**2 * dist**2 - ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta))**2) / dist**3
        A_safeB = -LgLfb
        A_safeu2B = -LgLfb2
        A_safeB = [A_safeB, A_safeu2B, 0, 0]
        psi_1 = b_dot + p1 * b**q1
        b_safeB = Lf2b + p1 * q1 * b**(q1 - 1) * b_dot + p2 * psi_1**q2
    else:
        pobsB_color = 'cyan'  # Assuming pobsB is an object with a Color attribute
        A_safeB = []
        b_safeB = []

    # Obs C avoidance

    nx, ny = OBSTACLE_POSITIONS[2]
    dist = np.sqrt((x - nx)**2 + (y - ny)**2)

    if sensing(robot, [obstacles[2]], detect, theta, ANGLE):
        radius = 7  # Specific radius for obstacle C
        pobsC_color = 'red'  # Assuming pobsC is an object with a Color attribute
        b = dist - radius
        b_dot = ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta)) / dist
        LgLfb = ((y - ny) * speed * np.cos(theta) - (x - nx) * speed * np.sin(theta)) / dist
        LgLfb2 = ((x - nx) * np.cos(theta) + (y - ny) * np.sin(theta)) / dist
        Lf2b = (speed**2 * dist**2 - ((x - nx) * speed * np.cos(theta) + (y - ny) * speed * np.sin(theta))**2) / dist**3
        A_safeC = -LgLfb
        A_safeu2C = -LgLfb2
        A_safeC = [A_safeC, A_safeu2C, 0, 0]
        psi_1 = b_dot + p1 * b**q1
        b_safeC = Lf2b + p1 * q1 * b**(q1 - 1) * b_dot + p2 * psi_1**q2
    else:
        pobsC_color = 'cyan'  # Assuming pobsC is an object with a Color attribute
        A_safeC = []
        b_safeC = []

    # CLF for reaching desired speed
    
    vd = dist_dst * v_max / 10
    if dist_dst < 1:
        vd = 0
    V_speed = (speed - vd)**2
    LfV_speed = 0
    LgV_speed = 2 * (speed - vd)

    # CBF for min and max speed satisfactions

    b_vmax = v_max - speed
    Lgb_vmax = -1
    A_vmax = -Lgb_vmax
    b_vmin = speed - v_min
    Lgb_vmin = 1
    A_vmin = -Lgb_vmin

    # Combining CLF and CBF constraints in the form Au = b

    # SOLVE QP

    # Initialize A as an empty list
    A_list = []

    # Append the constraints
    A_list.append([LgV, 0, -1, 0])

    if pobsA_color == 'red':
        A_list.append(A_safe)
    else:
        A_list.append([0,0,0,0])
    if pobsB_color == 'red':
        A_list.append(A_safeB)
    else:
        A_list.append([0,0,0,0])
    if pobsC_color == 'red':
        A_list.append(A_safeC)
    else:
        A_list.append([0,0,0,0])

    A_list.append([1, 0, 0, 0])
    A_list.append([-1, 0, 0, 0])
    A_list.append([0, 1, 0, 0])
    A_list.append([0, A_vmax, 0, 0])
    A_list.append([0, A_vmin, 0, 0])
    A_list.append([0, LgV_speed, 0, -1])

    # Convert A_list to a NumPy array
    A = np.array(A_list)

    # Ensure b is of the correct length
    b = np.array([b_V] + [b_safe if pobsA_color == 'red' else 0,
                        b_safeB if pobsB_color == 'red' else 0,
                        b_safeC if pobsC_color == 'red' else 0] + 
                        [u_max, -u_min, a_max, b_vmax, b_vmin, -LfV_speed - eps * V_speed])

    # Setting Up the Cost of the QP in the Form u^THu + H^Tu
    H = np.array([[2, 0, 0, 0],
                [0, 2, 0, 0],
                [0, 0, 2 * psc, 0],
                [0, 0, 0, 2 * psc]])
    
    F = np.array([0, 0, 0, 0])


    # Define the objective function for the optimizer
    def objective(x):
        return 0.5 * np.dot(x.T, np.dot(H, x)) + np.dot(F, x)

    # Define the constraints for the optimizer
    constraints = ({'type': 'ineq', 'fun': lambda x: b - np.dot(A, x)})

    # Perform the optimization
    result = minimize(objective, np.zeros(4), method='trust-constr', constraints=constraints)

    u = result.x
    #fval = result.fun

    t = [0, 0.1]

    # Solving the differential equation
    solution = solve_ivp(dynamics, t, state, args=(u,), t_eval=[t[-1]])
    xx = solution.y.T
    rt = xx[-1]

    once = 1

    # Conditional logic
    if u[2] > 0.1 and once == 1:
        b_c = dist - radius
        once = 0

    return rt
