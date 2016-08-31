# affw_control
An adaptive feed-forward controller for mobile platforms. The controller works on top of existing motor models and builds a compensation model during runtime, to reduce the error between target velocity and actual measured velocity.

The following diagram shows the package structure and message flow:

![alt tag](https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/affw_control/master/doc/ros_pkg_affw.png)

The green boxes are part of this repository. The affw-wrapper contains the robot specific part. The diagram contains two examples. The code for the hector tracker can be found in a separate repository: https://github.com/tu-darmstadt-ros-pkg/hector_tracker_affw
