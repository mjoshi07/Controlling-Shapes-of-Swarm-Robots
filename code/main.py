import config
from abstract_space import AbstractSpace
import utils
import plotting
import numpy as np


def start_algorithm(num_bots=None, num_iterations=None):
    if num_bots is None:
        num_bots = config.NUM_BOTS
    if num_iterations is None:
        num_iterations = config.NUM_ITERATIONS

    # Initialize empty abstract space
    abstract_space = AbstractSpace()
    u_curr, theta_curr, s1_curr, s2_curr = abstract_space.update(None)

    # create a list containing N bots
    bots = utils.create_bots(num_bots)

    # initialize bot position
    utils.initialize_swarm(bots)

    # initialize containers to store data
    counter = 0
    vel_cap_pts = []
    vel_star_pts = []
    bot_last_vel_pts_lin = []
    bot_last_vel_pts_ang = []
    state_tilde_plot_pts = []

    desired_state_index = 0

    # used in the case when there are multiple desired state spaces and we have to draw a tunnel like structure
    x_points = [[8, 14], [8, 14]]
    y_points = [[4, 4], [7.5, 7.5]]
    draw_lines = [x_points, y_points]
    draw_lines = None

    while True:
        # udpate abstract space according to current bots formation
        u_curr, theta_curr, s1_curr, s2_curr = abstract_space.update(bots)
        current_state = [u_curr, theta_curr, s1_curr, s2_curr]

        # specify desired state (in case there are multiple desired states)
        desired_state = config.DESIRED_ABSTRACT_STATES[desired_state_index]

        if counter % 1000 == 0:
            print("u: ", u_curr, "theta: ", theta_curr, "s1: ", s1_curr, "s2: ", s2_curr)
            plotting.plot_swarm(bots, current_state,   desired_state, str(desired_state_index) +"_" + str(counter), draw_lines)

        # check if desired state is attained
        if utils.goal_reached(current_state, desired_state, config.PRECISION) \
                or counter > num_iterations:
            if len(config.DESIRED_ABSTRACT_STATES) > 1 and desired_state_index < len(config.DESIRED_ABSTRACT_STATES) - 1:
                desired_state_index += 1
                counter = 0
            else:
                break

        # calculate state space error
        state_tilde = utils.get_state_tilde(current_state, desired_state)

        # calcualte control vector fields
        control_vector_fields = utils.get_control_vector_fields(state_tilde)

        # calculate formation variables
        R, H1, H2, H3 = utils.construct_formation_variables(theta_curr)

        # calculate Lie Group, g
        g = np.vstack(((np.hstack((R, u_curr))), np.array([0, 0, 1])))

        # calculate Gamma
        Gamma = np.vstack((np.hstack((g, np.zeros(shape=(3, 2)))),
                           np.hstack((np.zeros(shape=(2, 3)), config.IDENTITY_MAT))))

        vel_cap_all_bots = []
        vel_star_all_bots = []

        # looping over each robot
        for idx1 in range(len(bots)):

            bot = bots[idx1]

            # calculate optimal control law using minimum energy solution
            vel_star = utils.get_optimal_control_law(bot.q.transpose(), control_vector_fields, current_state, H1, H2, H3)

            bot_pos_moving_frame = np.matmul(R.transpose(), np.subtract(bot.q.transpose(), u_curr))

            # calculate differential of surjective submersion
            d_phi = np.vstack((config.IDENTITY_MAT,
                             (1/s1_curr-s2_curr)*np.matmul(bot_pos_moving_frame.transpose(), config.E1),
                              np.matmul(bot_pos_moving_frame.transpose(), np.add(config.IDENTITY_MAT, config.E2)),
                              np.matmul(bot_pos_moving_frame.transpose(), np.subtract(config.IDENTITY_MAT, config.E2))))

            # formulate the constraint for monotonic convergence
            convergence_condition = np.matmul(np.matmul(np.matmul(state_tilde.transpose(),
                                                                  config.GAIN_MAT), Gamma), d_phi)

            # flip the sign to maintain the inequality given at (14) of [1] for convex optimization
            convergence_condition = -convergence_condition
            convergence_constraint = np.array([[0.0]])

            # variables to be used in case of convex optimization failure during collision avoidance condition
            temp_a = convergence_condition
            temp_b = convergence_constraint

            # check for collision avoidance with every team member and impose additional constraint if neccessary
            convergence_condition, convergence_constraint = utils.check_collision_avoidance(idx1, bots, R, u_curr,
                                                                                            convergence_condition,
                                                                                            convergence_constraint)

            # calculate the best possible solution for velocity
            # closest to optimal solution which satisfies the constraints
            # handle the failure cases by reverting to the non collision avoidance constraints
            try:
                vel_convex_opt = utils.convex_optimizer(config.IDENTITY_MAT, np.matmul(R.transpose(), vel_star),
                                                        convergence_condition, convergence_constraint)
            except Exception as e:
                vel_convex_opt = utils.convex_optimizer(config.IDENTITY_MAT, np.matmul(R.transpose(), vel_star),
                                                        temp_a, temp_b)
            # book keeping for plotting
            if counter == 0 or counter % 1000 == 0:
                vel_cap_all_bots.append(vel_convex_opt)
                vel_star_all_bots.append(vel_star)

            # convert from moving frame to inertial frame
            vel_inertial_frame = np.matmul(R, vel_convex_opt)

            # update each bot position and orientation
            bots[idx1].move_robot(vel_inertial_frame_cvxopt=vel_inertial_frame,
                                  vel_inertial_frame_optimal=vel_star.transpose())

        idx1 = 0
        if counter % 50 == 0:
            state_tilde_plot_pts.append(state_tilde)
            bot_last_vel_pts_lin.append(bots[idx1].linear_vel)
            bot_last_vel_pts_ang.append(bots[idx1].angular_vel)
        if counter % 1000 == 0:
            vel_cap_pts.append(vel_cap_all_bots)
            vel_star_pts.append(vel_star_all_bots)

        counter += 1

    print("PROCESS COMPLETE")
    print("X: ", u_curr[0], " Y: ", u_curr[1], " theta: ", theta_curr, " s1: ", s1_curr, " s2: ", s2_curr)
    plotting.plot_state_tilde(state_tilde_plot_pts, counter)
    plotting.plot_vel(vel_cap_pts, vel_star_pts, bot_last_vel_pts_lin, bot_last_vel_pts_ang)


if __name__ == "__main__":
    start_algorithm()
    print('DONE')
