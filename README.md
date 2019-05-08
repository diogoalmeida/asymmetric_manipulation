Asymmetric Manipulation
===
This ROS package includes the necessary code to replicate the results in our paper
[Asymmetric Dual-Arm Task Execution using an Extended Relative Jacobian](https://arxiv.org/abs/1905.01248).
In particular, it defines generic velocity controllers that implement different strategies for modeling
the differential Kinematics of a dual-arm robotic manipulator, and the necessary configuration files to
setup a numerical simulation of the different methods.

We implement controllers that make use of:
1. The [Cooperative Task Space](https://dynamicsystems.asmedigitalcollection.asme.org/pdfaccess.ashx?url=/data/journals/jdsmaa/26230/691_1.pdf) (CTS)
* The [Extended CTS](https://ieeexplore.ieee.org/abstract/document/7759161)
* A relative Jacobian formulation
* Our extended relative Jacobian

Installation
==
Our code requires a working ROS installation (tested on full ROS Kinetic and Melodic installations).
In addition, we have dependencies on the following packages:
* [generic_control_toolbox](https://github.com/diogoalmeida/generic_control_toolbox)
* [baxter_description](https://github.com/RethinkRobotics/baxter_common) (required only to replicate our article's results. You should be able to configure this package to use your own robot's URDF).
* [robot_kinematic_simulation](https://github.com/diogoalmeida/robot_kinematic_simulation)
* [object_server](https://github.com/diogoalmeida/ros_object_server)
* [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools)

Running the numerical simulations
==
The script `run_simulations.py` runs the simulations from which the article's results were obtained,
```
$ rosrun asymmetric_manipulation run_simulations.py
```

**TODO** Pass arguments for plotting or saving results.
