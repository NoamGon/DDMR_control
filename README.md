# DDMR_control
This repository shows our work on the trajectory tracking control of a differential drive mobile robot, that is designed to drive autonomously in a greenhouse. The robot had 2 driving wheels and 2 steering wheels (4 actuators overall).
The control method that was used is Backstepping Kinematic Control for the driving wheels, while the steering wheels needed to help the robot converge and follow the desired trajectory.
The control algorithm was ran on a Nvidia Jetson Nano that was mounted on the robot, and the state estimation (position and orientation) was recieved via UDP communication through an Optitrack camera system.
The actuators were Dynamixel xm430 servo motors. 
First, the control algorithm was simulated in Simulink. After convergence was recieved, it was tested on the real robot. The algorithm was written in Python.
The trajectories that were teseted are linear and circular trajectories, as expected to be in a greenhouse. 
