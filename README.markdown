camera_aravis

This is a [ROS](http://ros.org) package for the [Aravis GigEVision
library](http://live.gnome.org/Aravis). It is open source, under the
LGPL (like Aravis itself).


------------------------
This ROS node publishes messages image_raw and camera_info for a specified camera.  It supports 
a variety of camera features that can be set through ROS parameters, including the following:
* autoexposure (off, once, continuous)
* autogain (off, once, continuous)
* exposure
* gain
* acquisition mode (freerunning, singleframe)
* framerate
* trigger source (software, line1, line2)
* software trigger rate

The basic command to run it, for example:
$ rosrun camera_aravis camnode

To run it in a given namespace, which is the better way to do it:
$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode


------------------------
It supports multiple cameras, each of which may be specified on the command-line, or via parameter.
Runs one node per camera.  For example, specifying the camera via the command-line:

$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode Basler-21237813


And specifying the camera via a parameter:

$ rosparam set cam1/guid Basler-21237813

$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode


------------------------
It support the dynamic_reconfigure protocol, and once the node is running, you may adjust 
its parameters by running the following and then manipulating the GUI:

$ rosrun dynamic_reconfigure reconfigure_gui


------------------------
There is an additional nice feature related to timestamps.  We want a stable timestamp on the
images that the camera delivers, giving a nice smooth time delta from frame to frame.  If we were 
to use the ROS clock on the PC, by the time we get the image packets from the camera a variable
amount of time has passed on the PC's clock due to variable network and system delays.  The camera's 
onboard clock is stable but it doesn't match with the ROS clock on the PC, and furthermore since it
comes from a different piece of hardware, the two clock's rates are slightly different.

The solution we've implemented is to start with a base of ROS time, and to accumulate the dt's from 
the camera clock.  To accomodate the difference in clock rates, a PID controller gently pulls the
result toward ROS time.  This system works nicely as much as we've tested it, but you may want to
verify that the timestamps on your system look good for your situation.  The camera_aravis package 
may be compiled with "#define TUNING" to let you adjust the PID gains to your liking, if you feel
the need.  Look in camnode.cpp for those sections, and you'll see how to set the PID gains via 
the parameter server.




