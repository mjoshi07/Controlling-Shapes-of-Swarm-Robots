import numpy as np
from math import log


# set total number of iterations for which control algorithm will run if it doesn't converge
NUM_ITERATIONS = 10000

# set total number of agents in the swarm
NUM_BOTS = 3

# set mean of the normal distribution to initialize the bot positions
BOTS_POSITION_MEAN = 6

# set standard deviation of the normal distribution to initialize the bot positions
BOTS_POSITION_SD = 1

# set precision value, to be used when calculating if the desired state space has been reached
PRECISION = 3

# set a discrete time step value used to update the position of individual agent in the swarm
dT = 0.01

# gain matrices for the control vector field as mentioned in [2] under section VII
KU = np.array([[2, 0], [0, 2]])
KS1 = 2
KS2 = 2
KT = 2

# matrices to define formation variables as mentioned in [2] equation (23)
E1 = np.array([[0.0, 1.0], [1.0, 0.0]])
E2 = np.array([[1.0, 0.0], [0.0, -1.0]])


# robot specifications as mentioned in [1] under section VII
BOT_RADIUS = 0.15
BOT_AXEL_LENGTH = 0.1
SAFE_DISTANCE = 0.1
MAX_LINEAR_VEL = 0.3
MAX_ANGULAR_VEL = 1

# separation distance as mentioned in [1] equation (9)
SEPARATION_DIST = 2*(BOT_RADIUS + BOT_AXEL_LENGTH) + SAFE_DISTANCE

# calculate concentration ellipse as mentioned in [2] equation (22)
CONST_PROBABILITY = 0.99
CONC_ELLIPSE = -2*log(1-CONST_PROBABILITY)

# Define the desired state space
# set desired centroid
U_DES = np.array([[10], [6]], dtype=np.float32)

# set desired orientation
THETA_DES = 0.0

# set desired semi major axis of the ellipse
S1_DES = 0.5

# set desired semi minor axis of the ellipse
S2_DES = 0.3


# define more desired state if neccessary
U_DES2 = np.array([[12], [6]], dtype=np.float32)
U_DES3 = np.array([[14], [6]], dtype=np.float32)
# DESIRED_ABSTRACT_STATES = [[U_DES, THETA_DES, S1_DES, S2_DES], [U_DES2, 0.0, 0.5, 0.3], [U_DES3, 0.0, 0.5, 0.3]]

# by default we converge to a single desired state space
DESIRED_ABSTRACT_STATES = [[U_DES, THETA_DES, S1_DES, S2_DES]]

# Gain matrix K, as mentioned in [1] under section VII
GAIN_MAT = np.vstack((np.array([1, 0, 0, 0, 0]),
                     np.array([0, 1, 0, 0, 0]),
                     np.array([0, 0, 0.8, 0, 0]),
                     np.array([0, 0, 0, 0.8, 0]),
                     np.array([0, 0, 0, 0, 0.8])))

# an identity matrix of size 2x2
IDENTITY_MAT = np.eye(2)


