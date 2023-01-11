# ros_vrpn_client

[![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=ros_vrpn_client)](https://jenkins.asl.ethz.ch/job/ros_vrpn_client/)

This is a ROS interface for VRPN ("Virtual-Reality Peripheral Network"). More info about VRPN can be found in their original [publication](https://dl.acm.org/doi/10.1145/505008.505019) and on their [github](https://github.com/vrpn/vrpn).

## Installation

For both Ubuntu and macOS:

1. build [vrpn_catkin](https://github.com/ethz-asl/vrpn_catkin) package (catkinized version of [VRPN](https://github.com/vrpn/vrpn))
2. build this package

## Usage

You have to start a ROS node per tracked object and the ROS node name has to be the name of the trackable object.

`rosrun ros_vrpn_client ros_vrpn_client _object_name:=object_name _vrpn_server_ip:=192.168.1.1`

Or in a launch file:

```XML
<arg name="object_name" default="some_object_name" />
<node name="some_object_name_vrpn_client" type="ros_vrpn_client" pkg="ros_vrpn_client" output="screen">
  <param name="vrpn_server_ip" value="192.168.1.100" />
  <param name="object_name" value="$(arg object_name)" />
  <!-- or directly:
  <param name="object_name" value="some_object_name" /> -->
</node>
```

## Info about coord frames

### TF coord frames

1. /optitrak
        - world frame that we will use.
        - X axis is along the x axis of the calibration pattern.
        - Z axis is vertically up.

2. Every tracked object has a coord frame whose TF name is the name of
   the ros node (given from the launch file or command line).

   Hitting "Reset To Current Orientation" in the TrackingTools
   software (Trackable properties) aligns the object coord frame with
   the /optitrak frame.

### Coord frames vodoo

The TrackingTools software outputs the position and orientation in a
funky coord frame which has the Y axis pointing vertically up, the X
axis along the x axis of the calibration square and Z axis along the
-ve z axis of the calibration square.

We perform some rotations to get rid of this funky frame and use the
/optitrak frame described above as our fixed world coord frame. The
code is in the "VRPN_CALLBACK track_target" function in
ros_vrpn_client.cpp
