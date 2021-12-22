import numpy as np
import config
import utils


class AbstractSpace:

    def __init__(self):
        """
        initialize empty state space with all ZERO values
        """
        self.u_curr = np.zeros(shape=[2, 1], dtype=np.float32)
        self.theta_curr = 0.0
        self.s1_curr = 0.0
        self.s2_curr = 0.0

    def get_centroid(self, swarm_bots=None):
        """
        calculate centroid of the formation as mentioned in [1]
        """
        if swarm_bots is None:
            return self.u_curr
        bot_centroids = 0.0
        for bot in swarm_bots:
            bot_centroids += bot.q.transpose()

        return bot_centroids / len(swarm_bots)

    def get_theta(self, swarm_bots, centroid):
        """
        calculate orientation of the formation as mentioned in [2] equation (29)
        """
        if swarm_bots is None:
            return self.theta_curr
        abstract_space_x = 0.0
        abstract_space_y = 0.0
        for bot in swarm_bots:
            bot_position = bot.q.transpose()
            abstract_space_y += np.matmul(np.matmul((bot_position - centroid).transpose(), config.E1),
                                     (bot_position - centroid)).item()
            abstract_space_x += np.matmul(np.matmul((bot_position - centroid).transpose(), config.E2),
                                     (bot_position - centroid)).item()

        return np.arctan2(abstract_space_y, abstract_space_x) / 2.0

    def get_semi_axes(self, swarm_bots, centroid, H1, H2):
        """
        calculate shape variables, s1 and s2
        mentioned in [2] equation (28)
        """
        if swarm_bots is None:
            return self.s1_curr, self.s2_curr
        abstract_space_s1 = 0.0
        abstract_space_s2 = 0.0
        for bot in swarm_bots:
            abstract_space_s1 += np.matmul(np.matmul((bot.q.transpose() - centroid).transpose(), H1),
                                (bot.q.transpose() - centroid)).item()
            abstract_space_s2 += np.matmul(np.matmul((bot.q.transpose() - centroid).transpose(), H2),
                                (bot.q.transpose() - centroid)).item()

        return abstract_space_s1 / (2 * (len(swarm_bots) - 1)), abstract_space_s2 / (2 * (len(swarm_bots)  - 1))

    def update(self, swarm_bots):
        """
        update the current state space using the current positions of the agents in the swarm
        """
        self.u_curr = self.get_centroid(swarm_bots)
        self.theta_curr = self.get_theta(swarm_bots,  self.u_curr)
        _, H1, H2, _ = utils.construct_formation_variables(self.theta_curr)
        self.s1_curr, self.s2_curr = self.get_semi_axes(swarm_bots, self.u_curr, H1, H2)

        return self.u_curr, self.theta_curr, self.s1_curr, self.s2_curr
