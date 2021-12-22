import numpy as np
import config
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import os


def plot_swarm(bots, current_state, desired_state, count, draw_lines=None):
    centroid = current_state[0]
    orientation = current_state[1]
    s1 = current_state[2]
    s2 = current_state[3]
    centroid_des = desired_state[0]
    orientation_des = desired_state[1]
    s1_des = desired_state[2]
    s2_des = desired_state[3]

    plot_dir = "./plots"
    ensemble_plot_dir = os.path.join(plot_dir, "ensemble_plots")

    if not os.path.exists(plot_dir):
        os.mkdir(plot_dir)
    if not os.path.exists(ensemble_plot_dir):
        os.mkdir(ensemble_plot_dir)
    # Plot Current and desired ellipses
    plt.figure()
    ax = plt.gca()
    ellipse1 = Ellipse(xy=(centroid[0], centroid[1]), width=config.CONC_ELLIPSE * s1, height=config.CONC_ELLIPSE * s2, angle=orientation * 180/np.pi, edgecolor='r', fill=False)
    ellipse2 = Ellipse(xy=(centroid_des[0], centroid_des[1]), width=config.CONC_ELLIPSE * s1_des, height=config.CONC_ELLIPSE * s2_des, angle=orientation_des * 180/np.pi, edgecolor='g', fill=False)
    ax.add_patch(ellipse1)
    ax.add_patch(ellipse2)
    plt.xlim((0, 20))
    plt.ylim((0, 20))

    for bot in bots:
        bot_pos = bot.q.T
        circle = plt.Circle((bot_pos[0], bot_pos[1]), (config.BOT_AXEL_LENGTH+config.BOT_RADIUS), color='b', fill=False)
        ax.add_patch(circle)

        x, y = bot_pos[0], bot_pos[1]
        length = (config.BOT_AXEL_LENGTH+config.BOT_RADIUS)
        orientation = bot.theta
        endy = y + length * np.sin(orientation)
        endx = x + length * np.cos(orientation)

        plt.plot([x, endx], [y, endy])

    if draw_lines is not None:
        plt.plot(draw_lines[0][0], draw_lines[1][0], color='r', markersize=3)
        plt.plot(draw_lines[0][1], draw_lines[1][1], color='r', markersize=3)

    file_path = os.path.join(ensemble_plot_dir, str(count) + ".png")

    plt.savefig(file_path)


def plot_state_tilde(state_tld_pts, counter):

    state_tld_pts_x = [state_tld[0].item() for state_tld in state_tld_pts]
    state_tld_pts_y = [state_tld[1].item() for state_tld in state_tld_pts]
    state_tld_pts_theta = [state_tld[2].item() for state_tld in state_tld_pts]
    state_tld_pts_s1 = [state_tld[3].item() for state_tld in state_tld_pts]
    state_tld_pts_s2 = [state_tld[4].item() for state_tld in state_tld_pts]
    plt.figure()
    plt.plot(state_tld_pts_x, label='~x')
    plt.plot(state_tld_pts_y, label='~y')
    plt.plot(state_tld_pts_theta, label='~theta')
    plt.plot(state_tld_pts_s1, label='~s1')
    plt.plot(state_tld_pts_s2, label='~s2')
    plt.legend(loc='upper right')
    plt.savefig('plots/state_tilde_plot.png')


def plot_vel(vel_cap_pts, vel_star_pts, last_lin_vel, last_ang_vel):
    vel_star_all_x = []
    vel_star_all_y = []
    for i in range(len(vel_star_pts[0])):
        vel_star_all_x.append([])
        vel_star_all_y.append([])
    for i in range(len(vel_star_pts)):
        vel_all_bots = vel_star_pts[i]
        for j in range(len(vel_all_bots)):
            vel_star_all_x[j].append(vel_all_bots[j][0])
            vel_star_all_y[j].append(vel_all_bots[j][1])

    vel_cap_all_x = []
    vel_cap_all_y = []
    for i in range(len(vel_cap_pts[0])):
        vel_cap_all_x.append([])
        vel_cap_all_y.append([])
    for i in range(len(vel_cap_pts)):
        vel_all_bots = vel_cap_pts[i]
        for j in range(len(vel_all_bots)):
            vel_cap_all_x[j].append(vel_all_bots[j][0])
            vel_cap_all_y[j].append(vel_all_bots[j][1])

    plt.figure()

    # plt.xlim((0, 10))
    plt.ylim((-10, 10))
    for i in range(len(vel_star_all_x)):
        plt.plot(vel_star_all_x[i], '-')
        plt.plot(vel_cap_all_x[i], '--')
    plt.legend(loc='upper right')
    plt.savefig('plots/optimal_computed_velocity_x.png')

    plt.figure()

    # plt.xlim((0, 10))

    plt.ylim((-10, 10))
    for i in range(len(vel_star_all_y)):
        plt.plot(vel_star_all_y[i], '-')
        plt.plot(vel_cap_all_y[i], '--')
    plt.legend(loc='upper right')
    plt.savefig('plots/optimal_computed_velocity_y.png')

    plt.figure()
    # plt.legend(loc='upper right')
    # plt.xlim((0, 10))
    plt.ylim((-0.5, 0.5))
    plt.plot(last_lin_vel)
    plt.savefig('plots/linear_velocity.png')

    plt.figure()
    # plt.legend(loc='upper right')
    # plt.xlim((0, 10))
    plt.ylim((-2, 2))
    plt.plot(last_ang_vel)
    plt.savefig('plots/angular_velocity.png')