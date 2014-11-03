ros_vrpn_client
===============

Ros interface for http://www.cs.unc.edu/Research/vrpn/

Installation
------------
Use install_vrpn.sh to download, compile, and install VRPN.
Please consult the VRPN website if your run into some trouble.



TF coord frames
----------------

1. /optitrak 
        - world frame that we will use.
        - X axis is along the x axis of the clibration pattern.
        - Z axis is vertically up.

2. Every tracked object has a coord frame whose TF name is the name of
   the ros node (given from the launch file or command line).

   Hitting "Reset To Current Orientation" in the TrackingTools
   software (Trackable properties) aligns the object coord frame with
   the /optitrak frame.

Running the code
----------------

1. Example to run node from command line:
     ./bin/ros_vrpn_client __name:=torso_trackable _vrpn_server_ip:=192.168.2.110


Coord frames vodoo
------------------
The TrackingTools software outputs the position and orientation in a
funky coord frame which has the Y axis pointing vertically up, the X
axis along the x axis of the calibration square and Z axis along the
-ve z axis of the calibration square.

We perform some rotations to get rid of this funky frame and use the
/optitrak frame described above as our fixed world coord frame. The
code is in the "VRPN_CALLBACK track_target" function in
ros_vrpn_client.cpp


