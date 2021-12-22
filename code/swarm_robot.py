import config
import numpy as np


class SwarmRobot:
    def __init__(self):
        """
        initialize empty SwarmRobot with all ZERO values
        """
        shape_ = [1, 2]
        self.q = np.zeros(shape=shape_, dtype=np.float32)
        self.vel = np.zeros(shape=shape_, dtype=np.float32)
        self.vel_star = np.zeros(shape=shape_, dtype=np.float32)
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def update(self, q_=None, vel_=None, vel_star_=None, theta_=None):
        if q_ is not None:
            self.q = q_
        if vel_ is not None:
            self.vel = vel_
        if vel_star_ is not None:
            self.vel_star = vel_star_
        if theta_ is not None:
            self.theta = theta_

    def move_robot(self, vel_inertial_frame_cvxopt, vel_inertial_frame_optimal):

        # calculate transformation matrix to calculate linear and angular velocity of a bot
        transformation_matrix = np.vstack(([np.cos(self.theta), np.sin(self.theta)],
                                 [-np.sin(self.theta)/config.BOT_AXEL_LENGTH, np.cos(self.theta)/config.BOT_AXEL_LENGTH]))

        # convert inertial frame velocity to robot frame velocity
        vel_moving_frame = np.matmul(transformation_matrix, vel_inertial_frame_cvxopt)

        # calculate linear velocity, as mentioned in [1], max linear velocity is set to 0.3 m/s
        self.linear_vel = min(vel_moving_frame[0].item(), config.MAX_LINEAR_VEL)

        # calculate angular velocity, as mentioned in [1], max angular velocity is set to 1 rad/s
        self.angular_vel = min(vel_moving_frame[1].item(), config.MAX_ANGULAR_VEL)

        # calculate linear displacement for time step dT
        linear_disp = config.dT * self.linear_vel * np.array([[np.cos(self.theta), np.sin(self.theta)]])

        # calculate linear displacement for time step dT
        angular_disp = config.dT * self.angular_vel

        # update all bot parameters to calculate abstract space for next time step
        # we also update vel_ and vel_star_ for plotting purposes
        self.update(q_=np.add(self.q, linear_disp),
                    vel_=vel_inertial_frame_cvxopt.transpose(),
                    vel_star_=vel_inertial_frame_optimal,
                    theta_=self.theta + angular_disp)

