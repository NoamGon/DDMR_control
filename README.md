# DDMR_control
This repository shows our work on the trajectory tracking control of a differential drive mobile robot, that is designed to drive autonomously in a greenhouse. The robot had 2 driving wheels and 2 steering wheels (4 actuators overall). The control method that was used is Backstepping Kinematic Control for the driving wheels, while the steering wheels needed to help the robot converge and follow the desired trajectory. The control algorithm was ran on a Nvidia Jetson Nano that was mounted on the robot, and the state estimation (position and orientation) was received via UDP communication through an Optitrack camera system. The actuators were used are Dynamixel xm430 servo motors. First, the control algorithm was simulated in Simulink:


![Capture4](https://github.com/user-attachments/assets/dec8c9df-b622-4971-b956-307513c64467)




The dynamic model and pid controller block:
![Capture5](https://github.com/user-attachments/assets/fd8bc88b-7987-41a6-b0a8-cc3eb954c398)

The trajectory tracking with non zero initial conditions:
![1](https://github.com/user-attachments/assets/b5211f3d-b62f-410c-8d8a-694e956f835c)

The trajectories that were tested are linear and circular trajectories, as expected to be in a greenhouse.
After convergence was received, it was tested on the real robot. The algorithm was written in Python. The results for the circular trajectory are shown in the following video:



https://github.com/user-attachments/assets/6cde1240-d091-48ec-a549-76f691cbcc12



