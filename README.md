Packages related to simulation and control of cable-driven parallel robots.

The robot geometry is defined in a YAML file (see sdf folder), then generates a SDF file through the call to `gen_cdpr.py <file>.yaml`.

The same robot can be simulated by calling `roslaunch cdpr.launch model:=<robot model>` where `robot model` corresponds to the YAML and SDF files.

A very basic PID controller can be tested using `rosrun cdpr pid_control`. The controller does not take into account the positive-only tensions and is just here to show the use of the CDPR class that interfaces with Gazebo.
