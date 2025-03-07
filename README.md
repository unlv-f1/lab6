# Lab 6: Interactive Markers

## I. Learning Goals

- Learn to place and manipulate markers on the F1Tenth Simulator
- Introduction to C++ ROS2
- Visualize Scan Matching in rivz

## II. Overview

The provided example implementations in the package "Scan_Matching" are coded in C++. This means students 
will need to rebuild the package (using colcon build) in between every change to the C++ files. 
Students have the option to provide hand-in code in either language (C++ or Python). 
Students will use the provided code to help understand Markers, before placing a raceline 
with them, and finally changing those markers to be interactive. (wording)

## III. Scan Matching
This lab implements the approach from  [Andrea Censi’s PLICP paper](/2008-irca-plicp.pdf), specifically 
within *transform.cpp* and *correspond.cpp*. 

###  The Point-to-Line Metric
Point-to-Line Itterative Closest Point (PLICP) is a variant of an ICP algorithm utilizing a point-to-line metric. 
The use of the point-to-line metric results in quadratically faster conversions, compared to ICP’s linear convergence. 
The designated point may also be projected onto an extension of the line segment, allowing for a simpler derivation of distance to the surface.

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
correspondence (implemented within correspond.cpp). For part one of this project, you will visualize the naive correspondance and the PLICP
correspondance methods and answer questions about the implementation.

### Markers
Markers in ROS2 help us to visualize points of interest in rviz. Markers in this class will be used in path trajectory planning and visualization of our iterative algorithms. It is best to get accustomed to their use now!  

To visualize Markers, you will need to add them by topic in rviz. The Scan_Match node implements them under the topic /scan_match_debug. You may need to change the Laser Scan colors to help you visualize the markers. You can do this in the dropdown menu in rviz.
![Markers by Topic](/img/transform_markers.png)

Marker visualization for the Scan_Match package is configured in *vizualization.h* and *visualization.cpp*. The density of markers is directly related to the number of iterations (number_iter within transform.cpp). Increase the number of iterations to increase the visibility of the matching. Record what changes as you increase the number of iterations.

#### *correspond.cpp*

Within the Scan Matching package provided, there exist 2 implementations of a correspondance algorithm. getNaiveCorrespondance() and getCorrespondance(). PLICP transformations from the *correspond.cpp* file. getCorrespondance() uses a faster implementation over the entire scan, while the naive approach only uses a single scan.


To run the Scan_Match node, build your workspace, source your install, and run:
```bash
~/sim_ws$ ros2 run lab5_pkg scanmatch_node 
Started
```
Remember, you need to rebuild and resource every change!

### Interactive Markers
Interactive Markers are similar to regular Markers, but they can be manipulated *within* rviz! Users can interact with them and 
freely change their positions and rotations dynamically. This will be extremely helpful going forwards, when students need to 
use Markers for a vehicle's heading in Pure Pursuit. 

- provide a marker csv file for track
- provide marker implementation (?) start of one

###  Implementation
###  Deliverables and Demonstrations

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `____________` package and 
directly commit and push to a repo shared with your TA. Your commited code should start and run in simulation smoothly.

**Shared Demonstration Requirements**:

**Simulator Demonstration**:





###  Grading Rubric

- Compilation: **10** Points


###  Extra Resources

Interactive Markers Documentation ROS: [LINK](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
Interactive Marker Tutorials: [LINK](https://github.com/ros-visualization/visualization_tutorials)