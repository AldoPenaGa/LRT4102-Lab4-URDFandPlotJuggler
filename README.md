<p align="center">
  <h2 align="center">Unified Robotics Description Format (URDF) and the interaction between ROS and PlotJuggler </h2>

  <p align="justify">
  This is the fourth laboratory report for the course titled Robotic Systems Design (LRT4102). This report will focus on the generation of a robotic cartesian model, in addition, PlotJuggler is explored while looking at its relation with Robot Operating System and ROSbag.
	  
  <br>Universidad de las Américas Puebla (UDLAP) - Guided by professor Dr. César Martínez Torres. "https://www.linkedin.com/in/c%C3%A9sar-martinez-torres-617b5347/?originalSubdomain=mx>" 
  </p>
</p>
<be>

## Table of contents
- [Introduction](#introduction)
- [Problems](#problems)
- [Codes](#codes)
- [Conclusion](#conclusion)
- [Contributors](#codes)

<div align= "justify">

### Introduction

In this report, we explore two essential structures: using URDF to construct a graphically depicted Cartesian robot with configurable dimensions and assessing ROS functionality with Plot Juggler. When developing the URDF model, we create a graphical representation of a Cartesian robot and use web-based tools to see and test it. Plot Juggler is then used to visually portray and analyze the most recent ROS practice (atg and dtg controller: https://github.com/AldoPenaGa/LRT4102-Lab3-ROSEuclidean), with the goal of depicting the error correction while charting the progression of key metrics like ATG and DTG. The ROS program is used then alongside ROSBag to verify its reliability to reproduce the actions related to the topics listened to, this was examined with Plot Juggler. This research emphasizes the efficient use of URDF for robotic design and Plot Juggler for accurate ROS analysis, resulting in improved comprehension of the design of robotic systems.

**URDF** 

URDF (Unified Robot Description Format) is an XML file format used in robotics to distinguish between robots and robotic systems, acting as a standard in simulation and robotic environments. It thoroughly explains the robot's geometry, joints, linkages, sensors, and other critical structural and kinematic features. A URDF file contains elements such as links that represent robot parts, joints that define kinematic relationships between links, collision models for collision calculations, visual models for graphical representation, inertias for mass and inertia properties, and sensors mounted on the robot such as cameras and lidars.


**PlotJuggler**

PlotJuggler is a data visualization tool created within ROS for use in robotics and other telemetry-related applications, featuring real-time data processing capabilities. PlotJuggler's key features and tools include an intuitive user interface for efficient data loading, visualization, and manipulation; support for various data types, including numerical values, vectors, matrices, images, and ROS messages; real-time visualization for monitoring and debugging running robotic systems; customization options for graph appearance and behavior, such as axis configuration, colors, and labels; and seamless integration with ROS for direct visualization


**ROSbag**

ROSBag is a flexible tool in the Robot Operating System (ROS) ecosystem that can capture, store, and play back ROS message data. It is a precious tool for data collection and analysis in robotics and related subjects. ROSBag enables users to record and preserve data streams from many ROS topics including as sensor readings, control instructions, and system states, allowing for offline analysis, troubleshooting, and experimentation. Its versatility and interoperability with ROS make it a vital component for robotics development and study based on data.

### Problems
There were multiple tasks to be addressed, for the URDF:

1- Create a Cartesian robot with free dimensions, the robot will be purely visual, use the website to verify its design.

Meanwhile for the PlotJuggler:

1- Plot and analyze the operation of the last laboratory.
2- Demonstrate that the final error is 0 or very close to 0. 
3- Plot the evolution of ATG and DTG. 
4- Run your program in a loop, at least 4 times, create a ROSBAG and then plot the entire experiment in Plot Juggler.


### Codes

The codes and files related to this practice can be found in two separate folders regarding its domain.

****-

The code dtg_atg asks for a desired position within the turtlesim workspace, the values are saved in a vector which will serve to calculate the distance to go (dtg). This is calculated by using the Pythagorean theorem (d = sqrt((x2-x1)² + (y2-y1)²)) between the actual pose, obtained by reading the topic `/turtle1/pose` and the desired location introduced by the user. The angle to go (atg) is calculated using atan2(y2-y1, x2-1) doing the operation in radians and displaying the result in degrees. Both values are printed once the program has finished its execution.

**spawn**

The spawn script has a similar input than the dtg_atg, in which the values are asked and saved in a vector. Then the `rospy.ServiceProxy` called `/kill` is called in order to erase the current turtle. Once this has done, the service `spawn` is also called so it spawns the turtle in the desired x, y and theta.

*Important to import the services `Spawn` and `Kill` from `turtle.srv` or the program won't work as intended.

**velocities**

The velocities program will print continuously the linear and angular velocities based on the error and the controller adjustments. These velocities are obtained with the product of those characteristics.

This script just demonstrates the relation between these structures and how they affect the movement of the turtlesim, in future releases, the movement in y axis and a plotting graphic tool will be added.

**dtg_atg_withcontroller**

This script aims not only to calculate the distance to go and the angle to go but to move the robot to the desired pose introduced by the user, first does the same than the dtg_atg script and then, by using a P controller (iterating the proportional control constant for angular velocity to the error produced by the actual pose and the desired pose), rotates to achieve the position asked (this is done by setting the linear velocity to zero meanwhile it reaches a threshold for the rotation) and then moves forward to that point (comparing the actual pose to the desired pose and utilizing the proportional control constant for linear velocity). `Kp_linear` and `Kp_angular` can be adjusted to meet the expactations depending on the purpose, nevertheless, the default values are: 0.5 and 1.1 respectively. Finally, the current pose, the desired pose and the error are displayed continuously. Once it has achieved the desired position, it asks again for a new pose to reach.

### Conclusion
In this study, we tested the Euclidean strategy for handling the Turtlesim in the Robot Operating System (ROS). We accomplished precise movement by calculating distances and angles while employing a P controller. Throughout this educational journey, we connected theory and practice, gaining significant insights into robotic system design. Plus, new structures like services were explored, setting another tool for basic programs in ROS.

### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle  |

