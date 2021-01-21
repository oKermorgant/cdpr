Packages related to simulation and control of cable-driven parallel robots.

The robot geometry is defined in a YAML file (see sdf folder), then generates a SDF file through the call to `gen_cdpr.py <file>.yaml`.

The same robot can be simulated by calling `roslaunch cdpr cdpr.launch model:=<robot model>` where `robot model` corresponds to the YAML and SDF files.

A very basic PID controller can be tested using `rosrun cdpr pid_control`. The controller does not take into account the positive-only tensions and is just here to show the use of the CDPR class that interfaces with Gazebo.

It is also possible to run both the simulation and the controller using the lunch files in the cdpr_controllers package for instance `roslaunch cdpr_controllers cdpr_pid_qp.launch`.


### Model generation

An example is given through cdpr/sdf/caroca.yaml. The sim_cables field leads to two behaviors: 
* if True then Gazebo wil simulate the cables as rigid bodies and subscribe for cable tensions
* if False then Gazebo will simulate a free-floating platform and subscribe for cdpr::Tensions which are the tensions + unit vector of all cables.


## Installation

_tested with ROS 1 noetic under Ubuntu 20.04_

### Dependencies

[Log2plot] (https://github.com/oKermorgant/log2plot) and [ViSP] (https://visp.inria.fr) should be installed for these packages to work.

### Building from source in a catkin workspace

```
mkdir catkin_ws
cd catkin_ws
git clone https://github.com/remipannequin/cdpr.git src
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```


### Some works
The improvement with using CTC control algorithm and trajectory generator

The control algorithm is CTC which integrates the quadratic programming optimization method in order to get the feasible tension in cables.

The trajectory parameter is defined by `trajectory.yaml` in sdf folder.

The SDF file is loaded by the launch file directly when using `roslaunch` command to launch one controller.

The 5 orders polynomial is implemented to generate one trajectory with updating time 0.01. It is set the same initial pose with initialization of cdpr, and the desired position is [2,2,1].

## References

Please use the following reference when citing this work:

[Franklin Okoli, Yuchuan Lang, Olivier Kermorgant, Stéphane Caro, "Cable-Driven Parallel Robot simulation using Gazebo and ROS", The 22nd CISM IFToMM Symposium on Robot Design, Dynamics and Control, Jun 2018, Rennes, France ](https://hal.archives-ouvertes.fr/hal-01757531v1)
