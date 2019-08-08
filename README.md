# Quadrotor-Trajectory-Navigation
Implementation of a trajectory generation algorithm in a quadrotor simulation
![trajectory generation](./media/path.png)

# Introduction
Hello. This is my project for my internship at the Robotics, Aerospace, and Information Networks (RAIN) lab at the University of Washington. 

I built a trajectory generation algorithm that is able to autonomously navigate a quadrotor through an obstacle-populated environment. I also built an accurate quadrotor simulation, however I also implemented this script on a live quadrotor in C. 

![finalized academic poster](./media/academic_poster.png)
![quadrotor used](./media/quad.png)

# Results
The finalized algorithm is able to quickly generate a safe and optimal trajectory that allows a quadrotor to fly at speeds up to 3 meters per second.

![recording of live flight test with implemented algorithm](./media/live_flight_test.gif)
![pathh of quadrotor during live flight test](./media/flight_test_photo.png)


# Navigation Functions
The algorithm utilized navigation functions as a method for trajectory generation. Navigation functions map quadrotor positions to costs, where favorable positions have low costs and unfavorable positions have high costs. For all other states, the navigation function varies smoothly. The algorithm used is able to safely and quickly navigate a live quadrotor through an obstacle-populated environment in real time. Navigation functions are also quick to compute as they only require the centers and radii of the spheres which encapsulate the obstacles. This algorithm can also generalize to non-spherical obstacles and environments involving multiple agents. 
![contour plot of cost function](./media/cost_function_paper.png)

# Quadrotor Simulation
Quad.m is the quadrotor class. This class provides an accurate simulation of a quadrotor flying around, including discretized controllers and sensors. I found that there was not a whole lot of difference between real life and simulation.

![simulation with implemented algorithm](./media/flight_test.gif)

# Algorithm
The algorithm is located in RoussosTrajectory.m. This function is mainly an optimizer that converges to the global minimum of a cost function provided by RoussosNavigationFunction.m. 