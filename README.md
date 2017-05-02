General outline of problem set

The problem set will focus primarily on the creation and simulation of an LQR solver. 
Students will visualize changes to the under-actuated system via variations of several parameters such as:
Control trajectories, Cost functions, System dynamics as well as the A, B, Q and R matrices.
We implement a simple inverted pendulum on a cart as our under-actuacted system and 
walk students through controlling it to reach an equilibrium point by implementing some helper functions and 
adjusting the control parameters.

The mini-pset is structured as follows:
- 
- Introduction to LQR
- API documentation
- Implement some helper functions for LQR
- Run LQR solver with some preset conditions -> simulate
- Tune the cost matrices (Q and R) -> simulate 
- Adjust dynamics matrices (A and B) -> simulate -> ocw.pset.2.c (effect of 2x gravity, torque...)



This problem set will explore the feedback control technique we discussed in lecture: the Linear Quadratic Regulator

## LQR 
. In this problem, we will develop an end-to-end controller capable of swinging up the pendulum over the cart and balancing it at the upright configuration.
. First, let's use LQR to design a stabilizing controller for the upright configuration q0=[[pi],[0]]. The first step is to linearize the system about the upright point. Find the matrices A and B such that the linearization is
d/dt [[q],[q_dot]] ~= A*[[(q-q_0)], [q_dot]] + B*u


Next we're going to develop the continuous time LQR controller. *** let's not ask them to implement LQR. Instead, ask for example Q and R cont matrices***
- scipy.linalg has several useful functions (link to scipy.linalg https://docs.scipy.org/doc/scipy-0.19.0/reference/linalg.html), 
lookup scipy.linalg solution to the continuous-time algebraic Riccati equation
- now compute the LQR gain as follows inv(R)*B.T*X


- Come up with some symmetric positive definite cost matrices Q and R and pass these into the LQR function that we implemented to get 
[K, S, E] = lqr(A, B, Q, R) Where K (2d array) represents the state feedback gains, S (2d array) is the solution to the Riccati equations, and E (1d array) represents the eigenvalues of the closed loop system
- LQR is just that easy! Now you have a controller that will stabilize the equilibrium at q0. 

## Implement constrain(theta) --
    theta = theta % (2*pi); if theta > pi: theta = -2*pi+theta; return theta
## Implement swing_up(u) controller ---
The last piece of the puzzle is to implement a swing-up controller for the pendulum.
## Implement control(u) ---
For now, the LQR cost-to-go function, (x−x0)' * S(x−x0) gives a good notion of how "far" away any state is from the equilibrium x0. Implement a case statement to switch from the swing-up controller to the LQR controller, whenever this cost-to-go is within some threshold.

Now, you are good to go! There are a few values to tune: Q,R,k1,k2,k3 and the switching threshold. You may have to play around with these values to get a stabilizing controller, but once you have something reasonable, the solution should be quite robust.

## Simulation
- Try simulating your LQR controller from initial states far away from the upright. 2*pi is upright. pi is bottom.
- What is the typical behavior?
https://danielpiedrahita.wordpress.com/portfolio/cart-pole-control/

Simulate this control law for a few trajectories, starting from near the downward position, trying a few values for k1,k2 and k3. Unlike with the pendulum, we do not have a proof that we will reach an orbit that passes near to the upright position. However, if you've chosen reasonable gains, you should find that the acrobot does, in fact, swing up--but that it then quickly falls back down. We're still missing stabilization!
This is where the LQR controller from earlier comes into play. The tough question here is when to switch from the swingup controller to the stabilization controller? Ideally, we would like to switch whenever the current state is in the region of attraction created by the LQR solution.

Basin of Attraction analysis
https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf
controllability




How does the time taken to get within 0.05 of the goal in x and x_dot change for the LQR policy change as Q is increased? e.g. When Q is set to (i) 0.25 times and (ii) 100 times the identity matrix and R is kept the same. 
=> 6 seconds when using Q=1000*I, also 6 seconds when Q=0.25*1
=>  The trajectory followed by the min-time policy is much sharper and reaches higher speeds than those achieved
by the LQR. The LQR follows a much smoother path to the goal, and accelerates much less due to the cost put
on actuation, while the min-time policy is constantly using as much of its actuator as it can. 
=> The LQR solution required 9.89 seconds to reach the goal using Q = .25I. The min-time policy required 4.43
seconds to reach the goal. Using Q = 100I, however, resulted in a time of 4.04 seconds. This is due to the LQR
policy not having bounds on actuation, and thus being able to outperform the min-time solution through the use
of much larger actuator magnitudes than the min-time was permitted. 

- Manual initial conditions for B

- Questions about stabilization. Controllability.

# LQR-Trees: Feedback Motion Planning
Connection to more advanced controls such as funnel libraries which use LQR.
 Plot Basin of attraction from ocw.pset1.2
LQR-tree for the simple pendulum
 Tradeoffs, Limitations, Dimensionality
how do these compare

 Comparison between the Basin of Attraction of a Normal LQR Controller (red) and an LQR-Trees Control Policy (blue)
https://danielpiedrahita.wordpress.com/portfolio/cart-pole-control/

