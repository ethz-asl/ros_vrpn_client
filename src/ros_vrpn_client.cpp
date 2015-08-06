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

//== This application listens for a rigid body named 'Tracker' on a remote machine
//== and publishes & tf it's position and orientation through ROS.
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <Eigen/Geometry>

#include "vicon_odometry_estimator.h"

void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB t);

// TODO(millanea@ethz.ch): The following callbacks should be implemented in the case that
//                         the tracking system supports them. In this case a flag should be
//                         added to the code indicating the whether or not the vicon estimator
//                         should be run.
//void VRPN_CALLBACK track_target_velocity(void *, const vrpn_TRACKERVELCB tv);
//void VRPN_CALLBACK track_target_acceleration(void *, const vrpn_TRACKERACCCB ta);

class TargetState
{
 public:
  geometry_msgs::TransformStamped target;
  nav_msgs::Odometry odometry;
};

TargetState *target_state;
std::string object_name;
std::string coordinate_system_string;

enum CoordinateSystem
{
  vicon,
  optitrack
} coordinate_system;

// Global indicating the availability of new VRPN callback function.
bool fresh_data = false;
vrpn_TRACKERCB prev_vrpn_data;

// Pointer to the vicon estimator. Global such that it can be accessed from the callback
vicon_estimator::ViconOdometryEstimator* vicon_odometry_estimator = NULL;

class Rigid_Body
{
 private:
  ros::Publisher target_pub;
  ros::Publisher odometry_pub;
  tf::TransformBroadcaster br;
  vrpn_Connection *connection;
  vrpn_Tracker_Remote *tracker;

 public:
  Rigid_Body(ros::NodeHandle& nh, std::string server_ip, int port, const std::string& object_name)
  {
    target_pub = nh.advertise<geometry_msgs::TransformStamped>("pose", 10);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 10);
    std::stringstream connection_name;
    connection_name << server_ip << ":" << port;
    connection = vrpn_get_connection_by_name(connection_name.str().c_str());
    tracker = new vrpn_Tracker_Remote(object_name.c_str(), connection);

    tracker->print_latest_report();
    this->tracker->register_change_handler(NULL, track_target);

    // TODO(millanea@ethz.ch): The following callbacks should be added if they're available.
    //                         See detailed note above.
    //this->tracker->register_change_handler(NULL, track_target_velocity);
    //this->tracker->register_change_handler(NULL, track_target_acceleration);

    tracker->print_latest_report();
  }

  void publish_target_state(TargetState *target_state)
  {
    br.sendTransform(target_state->target);
    target_pub.publish(target_state->target);
  }

  void publish_odometry(TargetState *target_state)
  {
    odometry_pub.publish(target_state->odometry);
  }

  void step_vrpn()
  {
    this->tracker->mainloop();
    this->connection->mainloop();
  }
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

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB t)
{
  Eigen::Quaterniond qOrig(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
  Eigen::Quaterniond qFix(0.70710678, 0.70710678, 0., 0.);

  Eigen::Quaterniond orientation_measured_B_W;
  Eigen::Vector3d position_measured_W;
  switch (coordinate_system)
  {
    case optitrack:
    {
      // Here we rotate the Optitrack measured quaternion by qFix, a
      // Pi/2 rotation around the x-axis. By doing so we convert from
      // NED to ENU (I think).
      orientation_measured_B_W = qFix * qOrig * qFix.inverse();
      position_measured_W = Eigen::Vector3d(t.pos[0], -t.pos[2], t.pos[1]);
      break;
    }
    case vicon:
    {
      orientation_measured_B_W = qOrig;
      position_measured_W = Eigen::Vector3d(t.pos[0], t.pos[1], t.pos[2]);
      break;
    }
    default:
    {
      ROS_FATAL("Coordinate system not defined!");
      break;
    }
  }

  // verifying that each callback indeed gives fresh data.
  if (prev_vrpn_data.quat[0] == t.quat[0] and prev_vrpn_data.quat[1] == t.quat[1]
      and prev_vrpn_data.quat[2] == t.quat[2] and prev_vrpn_data.quat[3] == t.quat[3]
      and prev_vrpn_data.pos[0] == t.pos[0] and prev_vrpn_data.pos[1] == t.pos[1]
      and prev_vrpn_data.pos[2] == t.pos[2])
    ROS_WARN("Repeated Values");
  prev_vrpn_data = t;

  // Somehow the vrpn msgs are in a different time zone.
  const int kMicroSecToNanoSec = 1000;
  ros::Time timestamp_local = ros::Time::now();
  int timediff_sec = std::round(double(timestamp_local.sec - t.msg_time.tv_sec) / 3600) * 3600;

  int timestamp_nsec = t.msg_time.tv_usec * kMicroSecToNanoSec;
  ros::Time timestamp = ros::Time(t.msg_time.tv_sec + timediff_sec, timestamp_nsec);

  ros::Duration time_diff = ros::Time::now() - timestamp;
  if (std::abs(time_diff.toSec()) > 0.1) {
    ROS_WARN_STREAM_THROTTLE(1, "Time delay: " << time_diff.toSec());
  }

  // Updating the estimates with the new measurements
  vicon_odometry_estimator->updateEstimate(position_measured_W, orientation_measured_B_W);
  vicon_odometry_estimator->publishResults(timestamp);
  Eigen::Vector3d position_estimate_W = vicon_odometry_estimator->getEstimatedPosition();
  Eigen::Vector3d velocity_estimate_W = vicon_odometry_estimator->getEstimatedVelocity();
  Eigen::Quaterniond orientation_estimate_B_W = vicon_odometry_estimator->getEstimatedOrientation();
  Eigen::Vector3d rate_estimate_B = vicon_odometry_estimator->getEstimatedAngularVelocity();

  // Rotating the estimated global frame velocity into the body frame
  Eigen::Vector3d velocity_estimate_B = orientation_estimate_B_W.toRotationMatrix() * velocity_estimate_W;

  // Populating topic contents. Published in main loop
  target_state->target.header.stamp = timestamp;
  target_state->target.header.frame_id = coordinate_system_string;
  target_state->target.child_frame_id = object_name;
  target_state->target.transform.translation.x = position_estimate_W.x();
  target_state->target.transform.translation.y = position_estimate_W.y();
  target_state->target.transform.translation.z = position_estimate_W.z();
  target_state->target.transform.rotation.x = orientation_estimate_B_W.x();
  target_state->target.transform.rotation.y = orientation_estimate_B_W.y();
  target_state->target.transform.rotation.z = orientation_estimate_B_W.z();
  target_state->target.transform.rotation.w = orientation_estimate_B_W.w();

  // Assemble odometry message.
  target_state->odometry.header.stamp = timestamp;
  target_state->odometry.header.frame_id = coordinate_system_string;
  target_state->odometry.child_frame_id = object_name;
  target_state->odometry.pose.pose.position.x = position_estimate_W.x();
  target_state->odometry.pose.pose.position.y = position_estimate_W.y();
  target_state->odometry.pose.pose.position.z = position_estimate_W.z();
  target_state->odometry.pose.pose.orientation.w = orientation_estimate_B_W.w();
  target_state->odometry.pose.pose.orientation.x = orientation_estimate_B_W.x();
  target_state->odometry.pose.pose.orientation.y = orientation_estimate_B_W.y();
  target_state->odometry.pose.pose.orientation.z = orientation_estimate_B_W.z();
  target_state->odometry.twist.twist.linear.x = velocity_estimate_B.x();
  target_state->odometry.twist.twist.linear.y = velocity_estimate_B.y();
  target_state->odometry.twist.twist.linear.z = velocity_estimate_B.z();
  target_state->odometry.twist.twist.angular.x = rate_estimate_B.x();
  target_state->odometry.twist.twist.angular.y = rate_estimate_B.y();
  target_state->odometry.twist.twist.angular.z = rate_estimate_B.z();

  // Indicating to the main loop the data is ready for publishing 
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
  private_nh.param<std::string>("object_name", object_name, "bluebird");

  std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;
  std::cout << "vrpn_port:" << vrpn_port << std::endl;
  std::cout << "vrpn_coordinate_system:" << coordinate_system_string << std::endl;
  std::cout << "object_name:" << object_name << std::endl;

  if (coordinate_system_string == std::string("vicon"))
  {

  }
  else if (coordinate_system_string == std::string("optitrack"))
  {

  }
  else
  {
    ROS_FATAL("ROS param vrpn_coordinate_system should be either 'vicon' or 'optitrack'!");
  }

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

    // only publish when receive data over VRPN.
    if (fresh_data == true)
    {
      tool.publish_target_state(target_state);
      tool.publish_odometry(target_state);
      fresh_data = false;
    }
    //ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
