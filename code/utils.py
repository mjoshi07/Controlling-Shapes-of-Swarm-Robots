import config
from cvxopt import solvers, matrix
import numpy as np
from swarm_robot import SwarmRobot


def create_bots(num_of_bots=config.NUM_BOTS):
    """
    creates a list of SwarmRobot class object
    list size equal to num_of_bots specified by the user
    """
    bots = []
    for i in range(num_of_bots):
        bot = SwarmRobot()
        bots.append(bot)
    return bots


def check_for_separation_dist(x, y, bot_positions):
    """
    iterates through all the existing bot positions
    check if the current x and y are at a user specified separation distance from the existing positions
    """
    for set_pos in bot_positions:
        dist = ((set_pos[0] - x) ** 2 + (set_pos[1] - y) ** 2) ** 0.5
        if dist < config.SEPARATION_DIST:
            return False
    return True


def initialize_swarm(bots):
    """
    initialize position of swarm bots with a normal distribution
    """
    # container to store the bot positions
    bot_positions = []

    for i in range(len(bots)):
        # create a random x and y position for each bot with a uniform distribution
        pos_x, pos_y = np.random.normal(config.BOTS_POSITION_MEAN, config.BOTS_POSITION_SD, size=(1, 2))[0]

        # check if the generated position is at a certain distance from existing bot positions
        # we initialize bots in this manner so as to avoid collisions at the beginning
        if check_for_separation_dist(pos_x, pos_y, bot_positions):
            bot_positions.append([pos_x, pos_y])
        else:
            dist_pass = False
            while not dist_pass:
                # keep generating random values for x and y till all bots are at a certain distance from each other
                # to satisfy the collision condition at the beginning
                pos_x, pos_y = np.random.normal(config.BOTS_POSITION_MEAN, config.BOTS_POSITION_SD, size=(1, 2))[0]
                dist_pass = check_for_separation_dist(pos_x, pos_y, bot_positions)

            # store the positions that satisfy the collision avoidance condition
            bot_positions.append([pos_x, pos_y])

    for bot_, pos in zip(bots, bot_positions):
        # update each bot position with the normal distribution generated positions
        bot_.q = np.array([[pos[0], pos[1]]])


def goal_reached(current_state, desired_state, precision=3):
    """
    check if the current state is equal to the desired state with the user specified precision
    we define state as a list of centroid (array of size 2x1), theta (scalar), s1 (scalar), s2(scalar)
    state = [centroid, theta, s1, s2]
    """

    # retrieve the x coordinate of the centroid of current state
    curr_centroid_x = current_state[0][0].item()

    # retrieve the y coordinate of the centroid of  current state
    curr_centroid_y = current_state[0][1].item()

    # retrieve orientation of the current state
    curr_theta = current_state[1].item()

    # retrieve semi major axis of the ellipse of current state
    curr_s1 = current_state[2]

    # retrieve semi minor axis of the ellipse of the current state
    curr_s2 = current_state[3]

    # retrieve the x coordinate of the centroid of the desired state
    des_centroid_x = desired_state[0][0].item()

    # retrieve the y coordinate of the centroid of the desired state
    des_centroid_y = desired_state[0][1].item()

    # retrieve orientation of the desired state
    des_theta = desired_state[1]

    # retrieve semi major axis of the ellipse of the desired state
    des_s1 = desired_state[2]

    # retrieve semi minor axis of the ellipse of the desired state
    des_s2 = desired_state[3]

    # round of the difference in the values to the specified precision
    # if the difference after rounding off is non-ZERO in any case it means desired state is not reached yet
    if round(curr_centroid_x - des_centroid_x, ndigits=precision):
        return False
    if round(curr_centroid_y - des_centroid_y, ndigits=precision):
        return False
    if round(curr_theta - des_theta, ndigits=precision):
        return False
    if round(curr_s1 - des_s1, ndigits=precision):
        return False
    if round(curr_s2 - des_s2, ndigits=precision):
        return False

    return True


def convex_optimizer(C, d, A, b):
    """
    C - is m x n dense or sparse matrix
    d - is n x 1 dense matrix
    A - is p x n dense or sparse matrix
    b - is p x 1 dense matrix

    This is a wrapper function for python cvopxt - sovler quadratic program method

    Solves the equation ======>   min[(1/2)*{norm(C.x - d)}^2]   ---->  subject to  A*x <= b.

    """
    # we multiply by square root of 2 so as to satisfy the input(C, d) of convex optimization equation
    C = np.sqrt(2) * C
    d = np.sqrt(2) * d

    # convert to cvxopt matrix
    C = matrix(C, C.shape, 'd')
    d = matrix(d, d.shape, 'd')

    #   A  is p x n dense or sparse matrix
    A = matrix(A, A.shape, 'd')
    b = matrix(b, b.shape, 'd')

    """
    P is a n x n dense or sparse 'd' matrix with the lower triangular
    part of P stored in the lower triangle.  Must be positive semi-definite
    """
    P = C.T * C

    """
    q is an n x 1 dense 'd' matrix.
    """
    q = -d.T * C
    solvers.options['show_progress'] = False
    solution = solvers.qp(P, q.T, A, b)

    if 'x' in solution.keys():
        return solution['x']
    else:
        return np.array([[0], [0]])


def construct_formation_variables(theta):
    """
    constructs the formation variables as mentioned in [2] equation (24)
    """
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    H1 = np.eye(2) + np.matmul(np.matmul(R, R), config.E2)
    H2 = np.eye(2) - np.matmul(np.matmul(R, R), config.E2)
    H3 = np.matmul(np.matmul(R, R), config.E1)

    return R, H1, H2, H3


def get_state_tilde(current_state, desired_state):
    """
    calculates the state space error given by the difference of current state and the desired state
    """
    return np.vstack((np.subtract(desired_state[0], current_state[0]),
               desired_state[1] - current_state[1],
               desired_state[2] - current_state[2],
               desired_state[3] - current_state[3]))


def get_control_vector_fields(state_tilde):
    """
    calculate control vector fields as mentioned in [2] equation (42)
    """
    centroid_derivative = np.matmul(config.KU, np.vstack((state_tilde[0], state_tilde[1])))
    theta_derivative = config.KT*state_tilde[2].item()
    s1_derivative = config.KS1*state_tilde[3].item()
    s2_derivative = config.KS2*state_tilde[4].item()

    return centroid_derivative, theta_derivative, s1_derivative, s2_derivative


def get_optimal_control_law(position, control_vector_fields, current_state, H1, H2, H3):
    """
    calculate optimal control law given by the minimum energy solution
    optimal control law is defined as the projections of the minimum norm vector
    mentioned in [2] equation (41)
    """
    # extract individual control field
    centroid_derivative, theta_derivative, s1_derivative, s2_derivative = control_vector_fields[0],\
                                                                          control_vector_fields[1],\
                                                                          control_vector_fields[2],\
                                                                          control_vector_fields[3]
    # extract individual current state variable
    u_curr, s1_curr, s2_curr = current_state[0], current_state[2], current_state[3]

    # implement equation (41)
    optimal_control_law = np.add(np.add(centroid_derivative, (
                (s1_curr - s2_curr) * np.matmul(H3, np.subtract(position, u_curr)) * theta_derivative / (s1_curr + s2_curr))),
                      np.add((np.matmul(H1, np.subtract(position, u_curr)) * s1_derivative / 4 * s1_curr),
                             (np.matmul(H2, np.subtract(position, u_curr)) * s2_derivative / 4 * s2_curr)))

    return optimal_control_law


def check_collision_avoidance(idx1, bots, R, u_curr, convergence_condition, convergence_constraint):
    """
    check distance between idx1 index bot and its neighborhood (all other bots)
    if the distance between idx1 index bot and any of the bot is less than separation distance
    add a constraint as mentioned in [1] equation (10)
    """
    # calculate bot position in moving frame
    bot_pos_moving_frame = np.matmul(R.transpose(), np.subtract(bots[idx1].q.transpose(), u_curr))

    # iterate through all the bots
    for idx2 in range(len(bots)):
        if idx2 == idx1:
            continue
        else:
            # calculate other bot position in moving frame
            bot2_pos_moving_frame = np.matmul(R.transpose(), np.subtract(bots[idx2].q.transpose(), u_curr))

            # calculate the difference in the 2 bots position
            diff_pos = np.subtract(bot_pos_moving_frame, bot2_pos_moving_frame)

            # calculate delta as mentioned in [1]
            delta = np.linalg.norm(diff_pos, 2)

            # collision avoidance condition
            if delta <= config.SEPARATION_DIST:

                # calculate bot idx1 velocity in moving frame
                bot_vel_moving_frame = np.matmul(R.transpose(), bots[idx1].vel.transpose())

                # calculate bot idx2 velocity in moving frame
                bot2_vel_moving_frame = np.matmul(R.transpose(), bots[idx2].vel.transpose())

                # calculate difference in 2 bots velocity
                diff_vel = np.subtract(bot_vel_moving_frame, bot2_vel_moving_frame)

                # formulate the collision avoidance condition as mentioned in [1] equation (10)
                collision_avoidance_condition = np.matmul(diff_pos, diff_vel.transpose())

                # add the formulated collision avoidance condition to the existing conditions for monotonic convergence
                convergence_condition = np.vstack((convergence_condition, -collision_avoidance_condition))
                convergence_constraint = np.vstack((convergence_constraint, np.array([[0.0], [0.0]])))

    return convergence_condition, convergence_constraint
