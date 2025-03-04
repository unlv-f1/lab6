# Lab 6: Interactive Markers

## I. Learning Goals

- Learn to place and manipulate markers on the F1Tenth Simulator
- Introduction to C++ ROS2
- Visualize Scan Matching in rivz

## II. Overview

The provided example implementations are coded in C++. Students have the option to provide hand-in code in either language (C++ or Python). Students will use the provided code to help understand Markers, before creating a raceline with them, and finally changing those markers to be interactive. (wording)

## Scan Matching
This lab implements the approach from (INS LINK) Andrea Censi’s PLICP paper, specifically within *transform.cpp* and *correspond.cpp*.

###  The Point-to-Line Metric
PLICP is a variant of an ICP algorithm utilizing a point-to-line metric. The use of the point-to-line metric
results in quadratically faster conversions, compared to ICP’s linear convergence. The designated point
may also be projected onto an extension of the line segment, allowing for a simpler derivation of distance
to the surface.

### PLICP
The algorithm takes 3 inputs:
• A reference scan.
• A current scan.
• An initial guess of the roto-transform, calculated by a previous iteration.
The algorithm outputs a transformation, which is the converged solution representing the roto-transform
between the current and reference scan. This calculated transformation becomes the next iteration’s initial
guess, and will then be used, along with a formulated point-to-line error, to transform coordinates of the
current scan onto the frame of the previous scan. These steps will be repeated until convergence with the
surface point set.
There are two components of this approach, a transformation (implemented within transform.cpp), and
correspondence (implemented within correspond.cpp). For this project, you will implement the PLICP
correspondance method. The transformation and naive correspondance has been provided to you.


### Markers
Markers in ROS2 help us to visualize locations on the map in rviz. Markers can be used for a variety of purposes- but for now, we will stick to projecting vehicle heading and visualization of other vehicle algos(wording...)

#### *transform.cpp*

Within the Scan Matching package provided, there exists PLICP transformations from the *transform.cpp* file. 

### Interactive Markers
Interactive Markers are similar to regular Markers, but they can be manipulated within rviz. Users can interact with them and freely change their positions and rotations dynamically. This will be extremely helpful going forwards, when students need to use Markers for a vehicle's heading in Pure Pursuit. 

###  Implementation
###  Deliverables and Demonstrations

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `____________` package and directly commit and push to a repo shared with your TA. Your commited code should start and run in simulation smoothly.

**Shared Demonstration Requirements**:

**Simulator Demonstration**:





###  Grading Rubric

- Compilation: **10** Points


###  Extra Resources

Interactive Markers Documentation ROS: [LINK](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
