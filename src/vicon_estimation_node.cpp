/*
 * Copyright 2015 Alex Millane, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This file implements a node which subscribes to published raw vicon
// position and orientation measurements and, the vicon estimator and
// finally publishes position, velcocity, orientation and angular
// velocity estimates.

#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "vicon_odometry_estimator.h"

// Class for collecting data and passing it to the underlying estimator
class ViconDataListener {
 public:
  // Constructor
  ViconDataListener(
      ros::NodeHandle nh, ros::NodeHandle nh_private,
      vicon_estimator::ViconOdometryEstimator* vicon_odometry_estimator)
      : vicon_odometry_estimator_(vicon_odometry_estimator) {
    // Subscribing to the raw vicon data
    raw_transform_sub_ =
        nh.subscribe("vrpn_client/raw_transform", 10,
                     &ViconDataListener::transformStampedCallback, this);
    // Advertising the estimated target state components
    estimated_transform_pub_ =
        nh_private.advertise<geometry_msgs::TransformStamped>(
            "estimated_transform", 10);
    estimated_odometry_pub_ =
        nh_private.advertise<nav_msgs::Odometry>("estimated_odometry", 10);
    // Getting the object name
    nh_private.param<std::string>("object_name", object_name_, "auk");
  }

  // Raw vicon data callback.
  void transformStampedCallback(
      const geometry_msgs::TransformStampedConstPtr& msg) {
    // Extracting the relavent data from the message
    Eigen::Vector3d position_measured_W;
    Eigen::Quaterniond orientation_measured_B_W;
    tf::vectorMsgToEigen(msg->transform.translation, position_measured_W);
    tf::quaternionMsgToEigen(msg->transform.rotation, orientation_measured_B_W);
    ros::Time timestamp = msg->header.stamp;
    // Passing the received data to the estimator
    vicon_odometry_estimator_->updateEstimate(position_measured_W,
                                              orientation_measured_B_W,
                                              timestamp);
    // Retreiving the estimates
    Eigen::Vector3d position_estimate_W =
        vicon_odometry_estimator_->getEstimatedPosition();
    Eigen::Vector3d velocity_estimate_W =
        vicon_odometry_estimator_->getEstimatedVelocity();
    Eigen::Quaterniond orientation_estimate_B_W =
        vicon_odometry_estimator_->getEstimatedOrientation();
    Eigen::Vector3d rate_estimate_B =
        vicon_odometry_estimator_->getEstimatedAngularVelocity();
    // Rotating the estimated global frame velocity into the body frame.
    Eigen::Vector3d velocity_estimate_B =
        orientation_estimate_B_W.toRotationMatrix() * velocity_estimate_W;
    // Creating estimated transform message
    geometry_msgs::TransformStamped estimated_transform;
    estimated_transform.header = msg->header;
    tf::vectorEigenToMsg(position_estimate_W,
                         estimated_transform.transform.translation);
    tf::quaternionEigenToMsg(orientation_estimate_B_W,
                             estimated_transform.transform.rotation);
    // Creating estimated odometry message
    nav_msgs::Odometry estimated_odometry;
    estimated_odometry.header = msg->header;
    estimated_odometry.child_frame_id = object_name_;
    tf::pointEigenToMsg(position_estimate_W,
                        estimated_odometry.pose.pose.position);
    tf::quaternionEigenToMsg(orientation_estimate_B_W,
                             estimated_odometry.pose.pose.orientation);
    tf::vectorEigenToMsg(velocity_estimate_B,
                         estimated_odometry.twist.twist.linear);
    tf::vectorEigenToMsg(rate_estimate_B,
                         estimated_odometry.twist.twist.angular);
    // Publishing the estimates
    estimated_transform_pub_.publish(estimated_transform);
    estimated_odometry_pub_.publish(estimated_odometry);
    // Publishing the estimator intermediate results
    vicon_odometry_estimator_->publishIntermediateResults(msg->header.stamp);
  }

 private:
  // Raw vicon data subscriber.
  ros::Subscriber raw_transform_sub_;
  // Estimate publishers
  ros::Publisher estimated_transform_pub_;
  ros::Publisher estimated_odometry_pub_;
  // Name of the tracked object
  std::string object_name_;
  // Vicon-based estimator
  std::unique_ptr<vicon_estimator::ViconOdometryEstimator>
      vicon_odometry_estimator_;
};

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "vicon_estimator");

  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Creating a Vicon-based estimator to do the estimation
  vicon_estimator::ViconOdometryEstimator vicon_odometry_estimator(nh_private);
  vicon_odometry_estimator.initializeParameters(nh_private);
  vicon_odometry_estimator.reset();

  // Creating a Vicon Data Listener to direct vicon data to the estimator
  ViconDataListener vicon_data_listener(nh, nh_private,
                                        &vicon_odometry_estimator);

  // Spinng forever pumping callbacks
  ros::spin();
  // Exit tranquilly
  return 0;
}
