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

class TargetState
{
 public:
  geometry_msgs::TransformStamped target;
  nav_msgs::Odometry odometry;
};

TargetState *targetState;
std::string frameId;
std::string coordinateSystemString;

enum CoordinateSystem
{
  vicon,
  optitrack
} corrdinate_system;

// Global indicating the availability of new VRPN callback function.
bool freshData = false;
vrpn_TRACKERCB prevVrpnData;

// Pointer to the vicon estimator. Global such that it can be accessed from the callback
viconEstimator::ViconOdometryEstimator* viconOdometryEstimator = NULL;

class Rigid_Body
{
 private:
  ros::Publisher targetPub;
  ros::Publisher odometryPub;
  tf::TransformBroadcaster br;
  vrpn_Connection *connection;
  vrpn_Tracker_Remote *tracker;

 public:
  Rigid_Body(ros::NodeHandle& nh, std::string server_ip, int port)
  {
    targetPub = nh.advertise<geometry_msgs::TransformStamped>("pose", 100);
    odometryPub = nh.advertise<nav_msgs::Odometry>("odometry", 100);
    std::string connecNm = server_ip + ":" + boost::lexical_cast<std::string>(port);
    connection = vrpn_get_connection_by_name(connecNm.c_str());
    std::string targetName = nh.getNamespace().substr(1);
    tracker = new vrpn_Tracker_Remote(targetName.c_str(), connection);

    tracker->print_latest_report();
    //std::cout<<"vel_id: "<<tracker->velocity_m_id<<std::endl;
    this->tracker->register_change_handler(NULL, track_target);
    //this->tracker->register_change_handler(NULL, track_target_velocity);
    //this->tracker->register_change_handler(NULL, track_target_acceleration);
    tracker->print_latest_report();
  }

  void publish_target_state(TargetState *target_state)
  {
    br.sendTransform(target_state->target);
    targetPub.publish(target_state->target);
  }

  void publish_odometry(TargetState *target_state)
  {
    odometryPub.publish(target_state->odometry);
  }

  void step_vrpn()
  {
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
void VRPN_CALLBACK track_target(void *, const vrpn_TRACKERCB t)
{
  Eigen::Quaterniond qOrig(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
  Eigen::Quaterniond qFix(0.70710678, 0.70710678, 0., 0.);

  Eigen::Quaterniond qRot;
  Eigen::Vector3d pos;
  switch (corrdinate_system)
  {
    case optitrack:
    {
      // optitrak <-- funky <-- object
      // the q_fix.inverse() esures that when optitrak_funky says 0 0 0
      // for roll pitch yaw, there is still a rotation that aligns the
      // object frame with the /optitrak frame (and not /optitrak_funky)
      qRot = qFix * qOrig * qFix.inverse();
      pos = Eigen::Vector3d(t.pos[0], -t.pos[2], t.pos[1]);
      break;
    }
    case vicon:
    {
      qRot = qOrig;
      pos = Eigen::Vector3d(t.pos[0], t.pos[1], t.pos[2]);
      break;
    }
    default:
    {
      ROS_FATAL("Coordinate system not defined!");
      break;
    }
  }

  // verifying that each callback indeed gives fresh data.
  if (prevVrpnData.quat[0] == t.quat[0] and prevVrpnData.quat[1] == t.quat[1]
      and prevVrpnData.quat[2] == t.quat[2] and prevVrpnData.quat[3] == t.quat[3]
      and prevVrpnData.pos[0] == t.pos[0] and prevVrpnData.pos[1] == t.pos[1]
      and prevVrpnData.pos[2] == t.pos[2])
    ROS_WARN("Repeated Values");

  // Extracting the delta time between callbacks
  //std::cout << "delta time (s): " << (t.msg_time.tv_usec - prev_vrpn_data.msg_time.tv_usec) / 1000000.0 << std::endl;
  prevVrpnData = t;

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
  viconOdometryEstimator->updateEstimate(pos, qRot);
  viconOdometryEstimator->publishResults(timestamp);
  Eigen::Vector3d pos_hat = viconOdometryEstimator->getEstimatedPosition();
  Eigen::Vector3d vel_hat = viconOdometryEstimator->getEstimatedVelocity();
  Eigen::Quaterniond quat_hat = viconOdometryEstimator->getEstimatedOrientation();
  Eigen::Vector3d omega_hat = viconOdometryEstimator->getEstimatedAngularVelocity();

  // Populating topic contents. Published in main loop
  targetState->target.header.stamp = timestamp;
  targetState->target.header.frame_id = coordinateSystemString;
  targetState->target.child_frame_id = frameId;
  targetState->target.transform.translation.x = pos_hat.x();
  targetState->target.transform.translation.y = pos_hat.y();
  targetState->target.transform.translation.z = pos_hat.z();
  targetState->target.transform.rotation.x = quat_hat.x();
  targetState->target.transform.rotation.y = quat_hat.y();
  targetState->target.transform.rotation.z = quat_hat.z();
  targetState->target.transform.rotation.w = quat_hat.w();

  // Assemble odometry message.
  targetState->odometry.header.stamp = timestamp;
  targetState->odometry.header.frame_id = coordinateSystemString;
  targetState->odometry.child_frame_id = frameId;
  targetState->odometry.pose.pose.position.x = pos_hat.x();
  targetState->odometry.pose.pose.position.y = pos_hat.y();
  targetState->odometry.pose.pose.position.z = pos_hat.z();
  targetState->odometry.pose.pose.orientation.w = quat_hat.w();
  targetState->odometry.pose.pose.orientation.x = quat_hat.x();
  targetState->odometry.pose.pose.orientation.y = quat_hat.y();
  targetState->odometry.pose.pose.orientation.z = quat_hat.z();
  targetState->odometry.twist.twist.linear.x = vel_hat.x();
  targetState->odometry.twist.twist.linear.y = vel_hat.y();
  targetState->odometry.twist.twist.linear.z = vel_hat.z();
  targetState->odometry.twist.twist.angular.x = omega_hat.x();
  targetState->odometry.twist.twist.angular.y = omega_hat.y();
  targetState->odometry.twist.twist.angular.z = omega_hat.z();

  // Indicating to the main loop the data is ready for publishing 
  freshData = true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_vrpn_client");
  ros::NodeHandle nh("~");

  targetState = new TargetState;
  //frame_id = nh.getNamespace().substr(1);
  frameId = nh.getNamespace();

  std::string vrpnServerIp;
  int vrpn_port;
  std::string trackedObjectName;

  nh.param<std::string>("vrpn_server_ip", vrpnServerIp, std::string());
  nh.param<int>("vrpn_port", vrpn_port, 3883);
  nh.param<std::string>("vrpn_coordinate_system", coordinateSystemString, "vicon");

  std::cout << "vrpn_server_ip:" << vrpnServerIp << std::endl;
  std::cout << "vrpn_port:" << vrpn_port << std::endl;
  std::cout << "vrpn_coordinate_system:" << coordinateSystemString << std::endl;

  if (coordinateSystemString == std::string("vicon"))
  {

  }
  else if (coordinateSystemString == std::string("optitrack"))
  {

  }
  else
  {
    ROS_FATAL("ROS param vrpn_coordinate_system should be either 'vicon' or 'optitrack'!");
  }

  // Creating the estimator
  viconOdometryEstimator = new viconEstimator::ViconOdometryEstimator(nh);
  viconOdometryEstimator->initializeParameters(nh);
  viconOdometryEstimator->reset();

  // Creating object which handles data publishing
  Rigid_Body tool(nh, vrpnServerIp, vrpn_port);

  ros::Rate loopRate(1000);  //TODO(gohlp): fix this

  while (ros::ok())
  {
    tool.step_vrpn();
    //vrpn_SleepMsecs(10);

    // only publish when receive data over VRPN.
    if (freshData == true)
    {
      tool.publish_target_state(targetState);
      tool.publish_odometry(targetState);
      freshData = false;
    }
    //ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}
