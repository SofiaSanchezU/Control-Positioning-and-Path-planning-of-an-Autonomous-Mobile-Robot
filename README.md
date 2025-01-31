# Control-Positioning-and-Path-planning-of-an-Autonomous-Mobile-Robot
Code developed for "S. Sanchez, - Control- Positioning- Path planning for Feedback Linearized Differential-Drive".  
Master's graduate "Calabria University".  
For any questions or suggestions write to sofysan.1993@gmail.com

# Sumary.
This repository provides a detailed framework for mobile robotics, covering key topics such as path planning, controller implementation, mobile robot modeling, sensor integration, and the use of the Extended Kalman Filter (EKF) for accurate positioning in dynamic environments. The entire framework is designed and implemented within the MATLAB/Simulink environment, offering a unified platform for the exploration, analysis, and advancement of mobile robotics principles and applications. It serves as a valuable resource for researchers, engineers, and enthusiasts, offering in-depth insights into both fundamental and advanced techniques in mobile robotics.  
<p align="center">
  <img src="https://github.com/fercho-0109/Mobile-robotics/assets/40362695/23696346-064c-4809-91ac-6b2186e58d7a" alt="Texto alternativo" width="600">
</p>

# Problem Statement
Design a autonomus mobile robot framework capable of navigates a structured environment along a specified trajectory using a robust control strategy. The robot's positional data is gathered from an array of sensors and processed using multilateration algorithms. This raw data is then filtered to ensure accuracy and reliability. The entire control strategy operates within a closed-loop control system, as illustrated in Figure 
<p align="center">
  <img src="https://github.com/fercho-0109/Mobile-robotics/assets/40362695/898f4cfd-08c3-46a4-8ad3-529e24647de3" alt="Texto alternativo" width="400">
</p>

## Main Assumptions
- The robot base movement is performed by wheels.  
- Assume a differential drive robot model  
- The robot moves in a structured environment (The position of the obstacles is knowing in advance)
- The environment is equipped with beacons location sensors  

## Differential Drive Model 
<p align="center">
  <img src="https://github.com/fercho-0109/Mobile-robotics/assets/40362695/e4ec6f7c-1631-4a29-ae84-eefdb1700f1a" alt="Texto alternativo" width="600">
</p>


##  PATH PLANNING 
Define the environment  
<p align="center">
  <img src="https://github.com/SofiaSanchezU/Control-Positioning-and-Path-planning-of-an-Autonomous-Mobile-Robot/assets/155551302/2dafa97a-449d-43bc-a264-839e2839c0ce" alt="Texto alternativo" width="300">
</p>

The target and starting positions are selected from the environment so that the path planning can find the solution for all possible configurations.  
The position of all obstacles is a-priory know  
The obstacles are static   
**The strategy used to develop the path planning is visibility graphs** The goal is to compute the shortest Collision-Free path.  
**example** 

<p align="center">
  <img src="Images/path.png" alt="Texto alternativo" width="300">
</p>


## CONTROL
The objective is to design proper control input for the robot so that it drives its pose to a given trajectory or target state. Due to the characteristics of path planning, the Input/Output linearization control law was selected to drive the robot on the path at the correct time. Because the input-output linearization control would drive point B on the target, the Posture Regulation  control is considered to drive the robot to the desired pose.  
The idea is to remove the nonlinearities of the model by choosing a proper control law so that the closed-loop system results has a linear relationship with the inputs and outputs. Then, standard control techniques can be used to design the control law.  
<p align="center">
  <img src="https://github.com/fercho-0109/Mobile-robotics/assets/40362695/2aa65a5c-04be-4aaa-a429-f6b81f5d5da9" alt="Texto alternativo" width="400">
</p>

##  POSITIONING
The objective is to Locate (position and attitude) the robot in a given environment and in a given reference frame.  
For this task, a set of Beacons located as shown in the figure is assumed as sensors. The position of each sensor is apriori known. For simplicity is supposed that the sensors can cover all the space of the environment. Also, it is assumed that the sensor measurements are noise-affected.

<p align="center">
  <img src="Images/positioning.png" alt="Texto alternativo" width="300">
</p>

 
As we can see the sensors, because of the noise, do not represent reliable data of the position of the robot. Therefore, the application of a filter that improves the reliability of the data is necessary. To deal with this problem, the **Extended Kalman Filter** is taken into account since we are working with a non-linear model. Additionally, the system has noise in the model and in the measurements. 

 
# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment

# File description
The repository contains the main file
1. **code**: This folder contains all the necessary programs and functions to run the model



# Example to run the experiment  

### Matlab/Simulink simulation **"Mobile Robot"** 
1. Download the files or clone the repository 
2. Run the Matlab file "**main_script**".
4. The plots should start to show the results  

