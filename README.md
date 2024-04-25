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
- [Results](#results)
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

**URDF_Cartesian**

This XML file defines the framework of the Cartesian robot for simulation, including physical linkages, linear motion joints, movement restrictions, and visual features. It includes the materials for the parts' appearance, links indicating physical components such as the base and movement axes, joints that allow movement between links, and visual geometry for each component.

**dtg_atg_withcontrollerPublishing**

This Python code works similarly to the one developed in the last practice https://github.com/AldoPenaGa/LRT4102-Lab3-ROSEuclidean/blob/main/lab03/src/dtg_atg_withcontroller.py . The key difference is that it publishes the linear and angular errors and the distance and angle to go in the topics `/error_linear`, `/error_angular`, `/distance_to_go` and `/angle_to_go` using Float32 data type:

```Python
    error_linear_publisher = rospy.Publisher('/error_linear', Float32, queue_size=10)
    error_angular_publisher = rospy.Publisher('/error_angular', Float32, queue_size=10)

    distance_to_go_publisher = rospy.Publisher('/distance_to_go', Float32, queue_size=10)
    angle_to_go_publisher = rospy.Publisher('/angle_to_go', Float32, queue_size=10)
```
This will help in the plotting for PlotJuggler analysis.

### Results

The first image (figure 1) shows the built URDF model displayed in the following page https://mymodelrobot.appspot.com/5629499534213120 , the joints `joint_x`, `joint_y`, `joint_z` permits the moving of the model. 

![Figure 1. URDF model loaded in mymodelrobot](https://github.com/AldoPenaGa/LRT4102-Lab4-URDFandPlotJuggler/.jpg)


Meanwhile, the next series of pictures (figures illustrate the results for the PlotJuggler and ROSbag section.




### Conclusion
In this study, we tested the Euclidean strategy for handling the Turtlesim in the Robot Operating System (ROS). We accomplished precise movement by calculating distances and angles while employing a P controller. Throughout this educational journey, we connected theory and practice, gaining significant insights into robotic system design. Plus, new structures like services were explored, setting another tool for basic programs in ROS.

### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle  |

