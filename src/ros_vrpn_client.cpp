/*
 # Copyright (c) 2011, Georgia Tech Research Corporation
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Georgia Tech Research Corporation nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 # OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 # OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 # ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #

 ## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
 ## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
 */

// This application listens for a rigid body named 'Tracker' on a remote machine.
// The raw data is input to a Extended Kalman Filter (EKF) based estimator estimating
// the target position, velocity, orientation and angular velocity. The estimated
// quantities and raw data are then published through ROS.

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <glog/logging.h>

#include "vicon_odometry_estimator.h"

void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB tracker);

// NOTE(millanea@ethz.ch): The following callbacks should be implemented in the case that
//                         the tracking system supports them. In this case a flag should be
//                         added to the code indicating the whether or not the vicon estimator
//                         should be run.
//void VRPN_CALLBACK track_target_velocity(void *, const vrpn_TRACKERVELCB tv);
//void VRPN_CALLBACK track_target_acceleration(void *, const vrpn_TRACKERACCCB ta);

// A class representing the state of the tracked target.
struct TargetState
{
  geometry_msgs::TransformStamped measured_transform;
  geometry_msgs::TransformStamped estimated_transform;
  nav_msgs::Odometry estimated_odometry;
};

// Available coordinate systems.
enum CoordinateSystem
{
  vicon,
  optitrack
} coordinate_system;

// Global target descriptions.
TargetState* target_state;
std::string object_name;
std::string coordinate_system_string;

// Global indicating the availability of new VRPN callback function.
bool fresh_data = false;
vrpn_TRACKERCB prev_tracker;

// Pointer to the vicon estimator. Global such that it can be accessed from the callback.
vicon_estimator::ViconOdometryEstimator* vicon_odometry_estimator = NULL;

class Rigid_Body {

  public:
    // Constructor
    Rigid_Body(ros::NodeHandle& nh, std::string server_ip, int port, const std::string& object_name)
    {
      // Advertising published topics.
      measured_target_transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>("raw_transform", 10);
      estimated_target_transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>("estimated_transform", 10);
      estimated_target_odometry_pub_ = nh.advertise<nav_msgs::Odometry>("estimated_odometry", 10);
      // Connecting to the vprn device and creating an associated tracker.
      std::stringstream connection_name;
      connection_name << server_ip << ":" << port;
      connection = vrpn_get_connection_by_name(connection_name.str().c_str());
      tracker = new vrpn_Tracker_Remote(object_name.c_str(), connection);
      tracker->print_latest_report();
      // Registering a callback to be called when a new data is made available by the vrpn sever.
      this->tracker->register_change_handler(NULL, track_target);
      tracker->print_latest_report();
      // NOTE(millanea@ethz.ch): The following callbacks should be added if they're available.
      //                         See detailed note above.
      //this->tracker->register_change_handler(NULL, track_target_velocity);
      //this->tracker->register_change_handler(NULL, track_target_acceleration);
    }

    // Publishes the raw measured target state to the transform message.
    void publish_measured_transform(TargetState *target_state)
    {
      measured_target_transform_pub_.publish(target_state->measured_transform);
    }

    // Publishes the estimated target state to the transform message and sends tranform.
    void publish_estimated_transform(TargetState *target_state)
    {
      br.sendTransform(target_state->estimated_transform);
      estimated_target_transform_pub_.publish(target_state->estimated_transform);
    }

    // Publishes the estimated target state to the odometry message.
    void publish_estimated_odometry(TargetState *target_state)
    {
      estimated_target_odometry_pub_.publish(target_state->estimated_odometry);
    }

    // Passes contol to the vrpn client.
    void step_vrpn()
    {
      this->tracker->mainloop();
      this->connection->mainloop();
    }

  private:
    // Publishers
    ros::Publisher measured_target_transform_pub_;
    ros::Publisher estimated_target_transform_pub_;
    ros::Publisher estimated_target_odometry_pub_;
    tf::TransformBroadcaster br;
    // Vprn object pointers
    vrpn_Connection* connection;
    vrpn_Tracker_Remote* tracker;
};


// TODO(millanea@ethz.ch): The following callbacks should be implemented if they're required.
//                         See detailed note above.
//void VRPN_CALLBACK track_target_acceleration(void *, const vrpn_TRACKERACCCB ta) {
//  std::cout<<"acceleration_callback"<<std::endl;
//  std::cout<<"ta.vel[0]"<<ta.acc[0]<<std::endl;
//}
//
//void VRPN_CALLBACK track_target_velocity(void *, const vrpn_TRACKERVELCB tv) {
//  std::cout<<"velocity_callback"<<std::endl;
//  std::cout<<"tv.vel[0]"<<tv.vel[0]<<std::endl;
//}

// Corrects measured target orientation and position for differing frame definitions.
void inline correctForCoordinateSystem(const Eigen::Quaterniond& orientation_in,
                                       const Eigen::Vector3d& position_in,
                                       CoordinateSystem coordinate_system,
                                       Eigen::Quaterniond* orientation_measured_B_W,
                                       Eigen::Vector3d* position_measured_W)
{
  // Rotation to correct between optitrack and vicon
  const Eigen::Quaterniond kqOptitrackFix(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()));
  // Correcting measurements based on coordinate system
  switch (coordinate_system)
  {
    case optitrack:
    {
      // Here we rotate the Optitrack measured quaternion by qFix, a
      // Pi/2 rotation around the x-axis. By doing so we convert from
      // NED to ENU.
      *orientation_measured_B_W = kqOptitrackFix * orientation_in * kqOptitrackFix.inverse();
      *position_measured_W = Eigen::Vector3d(position_in.x(), -position_in.z(), position_in.y());
      break;
    }
    case vicon:
    {
      *orientation_measured_B_W = orientation_in;
      *position_measured_W = position_in;
      break;
    }
    default:
    {
      ROS_FATAL("Coordinate system not defined!");
      break;
    }
  }
}

// Compares two instances of tracker data for equality
bool inline tracker_is_equal(const vrpn_TRACKERCB& vprn_data_1, const vrpn_TRACKERCB& vprn_data_2)
{
  return( vprn_data_1.quat[0] == vprn_data_2.quat[0]
          and vprn_data_1.quat[1] == vprn_data_2.quat[1]
          and vprn_data_1.quat[2] == vprn_data_2.quat[2]
          and vprn_data_1.quat[3] == vprn_data_2.quat[3]
          and vprn_data_1.pos[0] == vprn_data_2.pos[0] 
          and vprn_data_1.pos[1] == vprn_data_2.pos[1]
          and vprn_data_1.pos[2] == vprn_data_2.pos[2] );
}

// Tracker Position/Orientation Callback
void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB tracker)
{
  // Constructing the raw measured target pose variables.
  Eigen::Quaterniond orientation_in(tracker.quat[3], tracker.quat[0], tracker.quat[1], tracker.quat[2]);
  Eigen::Vector3d position_in(tracker.pos[0], tracker.pos[1], tracker.pos[2]);
  // Constructing the measured pose variables corrected for frame definitions
  Eigen::Quaterniond orientation_measured_B_W;
  Eigen::Vector3d position_measured_W;
  correctForCoordinateSystem(orientation_in, position_in, coordinate_system,
                             &orientation_measured_B_W, &position_measured_W);

  // Verifying that each callback indeed gives fresh data.
  if (tracker_is_equal(prev_tracker, tracker)) {
    ROS_WARN("Repeated Values");
  }
  prev_tracker = tracker;

  // Somehow the vrpn msgs are in a different time zone.
  const int kMicroSecToNanoSec = 1000;
  const double kHoursToSec = 3600;
  ros::Time timestamp_local = ros::Time::now();
  int timediff_sec = std::round(double(timestamp_local.sec - tracker.msg_time.tv_sec) / kHoursToSec) * kHoursToSec;

  int timestamp_nsec = tracker.msg_time.tv_usec * kMicroSecToNanoSec;
  ros::Time timestamp = ros::Time(tracker.msg_time.tv_sec + timediff_sec, timestamp_nsec);

  ros::Duration time_diff = ros::Time::now() - timestamp;
  if (std::abs(time_diff.toSec()) > 0.1) {
    ROS_WARN_STREAM_THROTTLE(1, "Time delay: " << time_diff.toSec());
  }

  // Updating the estimates with the new measurements.
  vicon_odometry_estimator->updateEstimate(position_measured_W, orientation_measured_B_W);
  Eigen::Vector3d position_estimate_W = vicon_odometry_estimator->getEstimatedPosition();
  Eigen::Vector3d velocity_estimate_W = vicon_odometry_estimator->getEstimatedVelocity();
  Eigen::Quaterniond orientation_estimate_B_W = vicon_odometry_estimator->getEstimatedOrientation();
  Eigen::Vector3d rate_estimate_B = vicon_odometry_estimator->getEstimatedAngularVelocity();
  // Publishing the estimator intermediate results
  // TODO(millanea): This is really only useful for estimator debugging and should be removed
  //                 once things reach a stable state.
  vicon_odometry_estimator->publishIntermediateResults(timestamp);

  // Rotating the estimated global frame velocity into the body frame.
  Eigen::Vector3d velocity_estimate_B = orientation_estimate_B_W.toRotationMatrix() * velocity_estimate_W;

  // Populate the raw measured transform message. Published in main loop.
  target_state->measured_transform.header.stamp = timestamp;
  target_state->measured_transform.header.frame_id = coordinate_system_string;
  target_state->measured_transform.child_frame_id = object_name;
  tf::vectorEigenToMsg(position_measured_W, target_state->measured_transform.transform.translation);
  tf::quaternionEigenToMsg(orientation_measured_B_W, target_state->measured_transform.transform.rotation);

  // Populate the estimated transform message. Published in main loop.
  target_state->estimated_transform.header.stamp = timestamp;
  target_state->estimated_transform.header.frame_id = coordinate_system_string;
  target_state->estimated_transform.child_frame_id = object_name;
  tf::vectorEigenToMsg(position_estimate_W, target_state->estimated_transform.transform.translation);
  tf::quaternionEigenToMsg(orientation_estimate_B_W, target_state->estimated_transform.transform.rotation);

  // Populate the estimated odometry message. Published in main loop.
  target_state->estimated_odometry.header.stamp = timestamp;
  target_state->estimated_odometry.header.frame_id = coordinate_system_string;
  target_state->estimated_odometry.child_frame_id = object_name;
  tf::pointEigenToMsg(position_estimate_W, target_state->estimated_odometry.pose.pose.position);
  tf::quaternionEigenToMsg(orientation_estimate_B_W, target_state->estimated_odometry.pose.pose.orientation);
  tf::vectorEigenToMsg(velocity_estimate_B, target_state->estimated_odometry.twist.twist.linear);
  tf::vectorEigenToMsg(rate_estimate_B, target_state->estimated_odometry.twist.twist.angular);

  // Indicating to the main loop the data is ready for publishing.
  fresh_data = true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vrpn_client", ros::init_options::AnonymousName);
  ros::NodeHandle nh("");
  ros::NodeHandle private_nh("~");

  target_state = new TargetState;

  std::string vrpn_server_ip;
  int vrpn_port;
  std::string trackedObjectName;

  private_nh.param<std::string>("vrpn_server_ip", vrpn_server_ip, std::string());
  private_nh.param<int>("vrpn_port", vrpn_port, 3883);
  private_nh.param<std::string>("vrpn_coordinate_system", coordinate_system_string, "vicon");
  private_nh.param<std::string>("object_name", object_name, "auk");

  // Debug output
  std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;
  std::cout << "vrpn_port:" << vrpn_port << std::endl;
  std::cout << "vrpn_coordinate_system:" << coordinate_system_string << std::endl;
  std::cout << "object_name:" << object_name << std::endl;

  // Checking for valid coordinate specification
  CHECK(coordinate_system_string.compare("vicon")
        or coordinate_system_string.compare("optitrack"))
        << "ROS param vrpn_coordinate_system should be either 'vicon' or 'optitrack'!";

  // Creating the estimator
  vicon_odometry_estimator = new vicon_estimator::ViconOdometryEstimator(private_nh);
  vicon_odometry_estimator->initializeParameters(private_nh);
  vicon_odometry_estimator->reset();

  // Creating object which handles data publishing
  Rigid_Body tool(private_nh, vrpn_server_ip, vrpn_port, object_name);

  ros::Rate loop_rate(1000);  //TODO(gohlp): fix this

  while (ros::ok())
  {
    tool.step_vrpn();

    // Publishing newly received data.
    if (fresh_data == true)
    {
      tool.publish_measured_transform(target_state);
      tool.publish_estimated_transform(target_state);
      tool.publish_estimated_odometry(target_state);
      fresh_data = false;
    }
    loop_rate.sleep();
  }
  return 0;
}
