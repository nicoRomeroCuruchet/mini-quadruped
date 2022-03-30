# mini-quadruped

This is an open source project for anyone who wants to research low-cost quadruped robotic platforms, prresents 
a mathematical description, simulation and implementation of a three-dimensional quadruped robot, with six degrees 
of freedom on torso and multi-joint legs. The equations to express rotations in 
combination with the inverse kinematics, allowed to derive an algorithm to find out the position of the 
multi-joint legs as a function of the torso’s orientation. The goal is to proof that the stabilization
of the robot can be reached in real shifting environments by incorporating a gyroscope and accelerometer, 
merging both measures in order to obtain the orientation of the body in real time. 
Those magnitudes were the input of a closed PID feedback-loop algorithm which estimated the positions of the
each leg to keep the quadruped’s body balanced.

   ![Demo](https://media.giphy.com/media/i7qEreajtPtJEAiz5Z/giphy-downsized-large.gif)
