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

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#include <Eigen/Geometry>

#include "vicon_odometry_estimator.h"

void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB t);
//void VRPN_CALLBACK track_target_velocity(void *, const vrpn_TRACKERVELCB tv);
//void VRPN_CALLBACK track_target_acceleration(void *, const vrpn_TRACKERACCCB ta);

class TargetState {
 public:
  geometry_msgs::TransformStamped target;
  nav_msgs::Odometry odometry;
};


TargetState *target_state;
std::string frame_id;
std::string coordinate_system_string;

enum CoordinateSystem {
  vicon,
  optitrack
} corrdinate_system;

// set to true in the VRPN callback function.
bool fresh_data = false;
vrpn_TRACKERCB prev_vrpn_data;
//vrpn_TRACKERVELCB prev_vrpn_velocity_data;

// Pointer to the vicon estimator
ViconOdometryEstimator* viconOdometryEstimator =  NULL; //vicon_estimation::


class Rigid_Body {
 private:
  ros::Publisher target_pub;
  ros::Publisher odometry_pub;
  tf::TransformBroadcaster br;
  vrpn_Connection *connection;
  vrpn_Tracker_Remote *tracker;

 public:
  Rigid_Body(ros::NodeHandle& nh, std::string server_ip, int port) {
    target_pub = nh.advertise<geometry_msgs::TransformStamped>("pose", 100);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 100);
    std::string connec_nm = server_ip + ":" + boost::lexical_cast<std::string>(port);
    connection = vrpn_get_connection_by_name(connec_nm.c_str());
    std::string target_name = nh.getNamespace().substr(1);
    tracker = new vrpn_Tracker_Remote(target_name.c_str(), connection);

	tracker->print_latest_report();
    //std::cout<<"vel_id: "<<tracker->velocity_m_id<<std::endl;
    this->tracker->register_change_handler(NULL, track_target);
//    this->tracker->register_change_handler(NULL, track_target_velocity);
//    this->tracker->register_change_handler(NULL, track_target_acceleration);
    tracker->print_latest_report();
  }

  void publish_target_state(TargetState *target_state) {
    br.sendTransform(target_state->target);
    target_pub.publish(target_state->target);
  }

  void publish_odometry(TargetState *target_state) {
    odometry_pub.publish(target_state->odometry);
  }

  void step_vrpn() {
    this->tracker->mainloop();
    this->connection->mainloop();
  }
};

//void VRPN_CALLBACK track_target_acceleration(void *, const vrpn_TRACKERACCCB ta) {
//	std::cout<<"acceleration_callback"<<std::endl;
//	std::cout<<"ta.vel[0]"<<ta.acc[0]<<std::endl;
//}
//
//void VRPN_CALLBACK track_target_velocity(void *, const vrpn_TRACKERVELCB tv) {
//	std::cout<<"velocity_callback"<<std::endl;
//	std::cout<<"tv.vel[0]"<<tv.vel[0]<<std::endl;
//}

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB t) {
  Eigen::Quaterniond q_orig(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
  Eigen::Quaterniond q_fix(0.70710678, 0.70710678, 0., 0.);

  Eigen::Quaterniond q_rot;
  Eigen::Vector3d pos;
  switch (corrdinate_system) {
    case optitrack: {
      // optitrak <-- funky <-- object
      // the q_fix.inverse() esures that when optitrak_funky says 0 0 0
      // for roll pitch yaw, there is still a rotation that aligns the
      // object frame with the /optitrak frame (and not /optitrak_funky)
      q_rot = q_fix * q_orig * q_fix.inverse();
      pos = Eigen::Vector3d(t.pos[0], -t.pos[2], t.pos[1]);
      break;
    }
    case vicon: {
      q_rot = q_orig;
      pos = Eigen::Vector3d(t.pos[0], t.pos[1], t.pos[2]);
      break;
    }
    default: {
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

  // Extracting the delta time between callbacks
  //std::cout << "delta time (s): " << (t.msg_time.tv_usec - prev_vrpn_data.msg_time.tv_usec) / 1000000.0 << std::endl;
  prev_vrpn_data = t;  

  // Somehow the vrpn msgs are in a different time zone.
  const int kMicroSecToNanoSec = 1000;
  ros::Time timestamp_local = ros::Time::now();
  int timediff_sec = std::round(double(timestamp_local.sec - t.msg_time.tv_sec) / 3600) * 3600;

  int timestamp_nsec = t.msg_time.tv_usec * kMicroSecToNanoSec;
  ros::Time timestamp = ros::Time(t.msg_time.tv_sec + timediff_sec, timestamp_nsec);

  ros::Duration time_diff = ros::Time::now() - timestamp;
  if(std::abs(time_diff.toSec()) > 0.1) {
    ROS_WARN_STREAM_THROTTLE(1, "Time delay: " << time_diff.toSec());
  }

  // Updating the estimates with the new measurements
  viconOdometryEstimator->updateEstimate(pos, q_rot);
  viconOdometryEstimator->publishResults(timestamp);
  Eigen::Vector3d pos_hat = viconOdometryEstimator->getEstimatedPosition();
  Eigen::Vector3d vel_hat = viconOdometryEstimator->getEstimatedVelocity();
  Eigen::Quaterniond quat_hat = viconOdometryEstimator->getEstimatedOrientation();
  Eigen::Vector3d omega_hat = viconOdometryEstimator->getEstimatedAngularVelocity();

  // Populating topic contents. Published in main loop
  target_state->target.header.stamp = timestamp;
  target_state->target.header.frame_id = coordinate_system_string;
  target_state->target.child_frame_id = frame_id;
  target_state->target.transform.translation.x = pos_hat.x();
  target_state->target.transform.translation.y = pos_hat.y();
  target_state->target.transform.translation.z = pos_hat.z();
  target_state->target.transform.rotation.x = quat_hat.x();
  target_state->target.transform.rotation.y = quat_hat.y();
  target_state->target.transform.rotation.z = quat_hat.z();
  target_state->target.transform.rotation.w = quat_hat.w();

  // Assemble odometry message.
  target_state->odometry.header.stamp = timestamp;
  target_state->odometry.header.frame_id = coordinate_system_string;
  target_state->odometry.child_frame_id = frame_id;
  target_state->odometry.pose.pose.position.x = pos_hat.x();
  target_state->odometry.pose.pose.position.y = pos_hat.y();
  target_state->odometry.pose.pose.position.z = pos_hat.z();
  target_state->odometry.pose.pose.orientation.w = quat_hat.w();
  target_state->odometry.pose.pose.orientation.x = quat_hat.x();
  target_state->odometry.pose.pose.orientation.y = quat_hat.y();
  target_state->odometry.pose.pose.orientation.z = quat_hat.z();
  target_state->odometry.twist.twist.linear.x = vel_hat.x() ;
  target_state->odometry.twist.twist.linear.y = vel_hat.y() ;
  target_state->odometry.twist.twist.linear.z = vel_hat.z() ;
  target_state->odometry.twist.twist.angular.x = omega_hat.x() ;
  target_state->odometry.twist.twist.angular.y = omega_hat.y() ;
  target_state->odometry.twist.twist.angular.z = omega_hat.z() ;

  // Indicating to the main loop the data is ready for publishing 
  fresh_data = true;
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ros_vrpn_client");
  ros::NodeHandle nh("~");

  target_state = new TargetState;
  //frame_id = nh.getNamespace().substr(1);
  frame_id = nh.getNamespace();

  std::string vrpn_server_ip;
  int vrpn_port;
  std::string tracked_object_name;

  nh.param<std::string>("vrpn_server_ip", vrpn_server_ip, std::string());
  nh.param<int>("vrpn_port", vrpn_port, 3883);
  nh.param<std::string>("vrpn_coordinate_system", coordinate_system_string, "vicon");

  std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;
  std::cout << "vrpn_port:" << vrpn_port << std::endl;
  std::cout << "vrpn_coordinate_system:" << coordinate_system_string << std::endl;

  if (coordinate_system_string == std::string("vicon")) {

  } else if (coordinate_system_string == std::string("optitrack")) {

  } else {
    ROS_FATAL("ROS param vrpn_coordinate_system should be either 'vicon' or 'optitrack'!");
  }

  // Creating the estimator
  viconOdometryEstimator = new ViconOdometryEstimator(nh);
  viconOdometryEstimator->initializeParameters(nh);
  viconOdometryEstimator->reset();

  // Creating object which handles data publishing
  Rigid_Body tool(nh, vrpn_server_ip, vrpn_port);

  ros::Rate loop_rate(1000);  //TODO(gohlp): fix this

  while (ros::ok()) {
    tool.step_vrpn();
    //vrpn_SleepMsecs(10);
    if (fresh_data == true) {  // only publish when receive data over VRPN.
      tool.publish_target_state(target_state);
      tool.publish_odometry(target_state);
      fresh_data = false;
    }
    //ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
