# Controlling-Shapes-of-Swarm-Robots
Python based implementation of the paper "Controlling Shapes of Ensembles of Robots". This implementation satisfies the project 1 requirement for the course ENPM 667: Control for Robotic Systems. The paper focuses on developing energy based laws for controlling a team of robots to assemble in a desired formation.

# Introduction
1. Seeking the advantages provided by collective behavioral patterns to solve complex tasks has intrigued roboticists for a long time.
2. When in teams robots can interact with each other and augment their understanding of the environments they need to operate in.
3. The challenges faced by researchers however are to develop appropriate strategies to control a team of large robots in scalable efficient way.
4. Most control methods require identification of each individual member of robot team. As the number of robots in a team, the computations become more intensive to handle.
5. The project is our interpretation of the research work done by Michael and Kumar in [1] which utilizes transformation of a higher dimensional agent state space to a lower dimensional abstract space which depends on scalable characteristics.

# Approach 

   ![Example Abstract Space enclosing a team of robot [1]](https://user-images.githubusercontent.com/41729963/147318251-494b3d40-9d10-4e89-bc29-8ff1e2cef7fe.png)

Abstract space is characterized by the overall enclosing boundary of formation of the robot team. Authors in referenced paper argued that a concentration ellipsoid is most suitable to define boundary of team formation. Hence state variables of proposed abstract spacebecome : {u_x, u_y, θ, s1, s2}, where u_x, u_y & θ define position and orientation of ellipsoid and s1, s2 define the size of ellipsoid.

1. Most straightforward way to converge to the desired state is use minimum energy solution to drive state error to zero through exponential convergence.
2. When working in teams, robots have to constantly compute the separation distance with other members. If separation distance decreases below a threshold,
the chances of collision increases.
3. To accommodate for the collision avoidance requirement, authors adopt the idea of using solution closest to the minimum energy solution and use it to satisfy the
monotonic convergence inequality.
4. This becomes a quadratic problem using a constrained convex optimization [3].

 ![Convex Optimization](https://user-images.githubusercontent.com/41729963/147318574-b78af85a-2017-460c-adad-d6324a4206e8.png)

5. Constrained convex optimization allows us to impose constraints of the form Ax<b.
6. This is where our 2 constraints for monotonic convergence and collision avoidance will be utilized.

# Simulation and Results
A team of 5 robots converges to a desired formation. We observe that for some particular collision constraints, the convex optimization solver fails
and is not able to find any solution.

But the overall system converges to the desired state as shown in the figure.
![output](https://user-images.githubusercontent.com/41729963/147318728-73a30106-b310-4b85-96b8-6f7e6c04b0bd.gif)

# Conclusion
We have presented our interpretation of the techniques used by authors to develop laws to control an ensemble of robots. The saliency of these laws lie in the fact that they provide a way for controlling team of robots regardless of number of team members and robustness to their failure. This is done by converting the high dimensional robot space to a low dimensional abstract space. To achieve the desired state, this abstract space evolution is used to calculate individual robot velocity subject to constraints of monotonic convergence and collision avoidance. We implemented the algorithm in Python using cvxopt library to solve constrained convex optimization and it was observed that the results were consistent to the arguments made by the author.

# References
1. Michael, N. and Kumar, V. (2008). Controlling shapes of ensembles of robots of finite size with nonholonomic constraints. Robotics: Science and Systems, Zurich, Switzerland.
2. Belta and V. Kumar, “Abstraction and control for groups of robots,” IEEE Transactions on Robotics, vol. 20, no. 5, pp. 865–875, Oct. 2004.
3. Python library for convex optimization : https://cvxopt.org/


