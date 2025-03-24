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

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "vicon_odometry_estimator.hpp"

using std::placeholders::_1;

// Class for collecting data and passing it to the underlying estimator
class ViconDataListener : public rclcpp::Node
{
public:
  // Constructor
  ViconDataListener(
    const rclcpp::NodeOptions & options,
    const std::shared_ptr<vicon_estimator::ViconOdometryEstimator> vicon_odometry_estimator)
  : Node("vicon_data_listener", options), vicon_odometry_estimator_(vicon_odometry_estimator)
  {
    // Getting the object name
    this->declare_parameter<std::string>("object_name", "auk");
    this->get_parameter<std::string>("object_name", object_name_);
    // Subscribing to the raw vicon data
    raw_transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "vrpn_client/raw_transform", 10,
      std::bind(&ViconDataListener::transformStampedCallback, this, _1));
    // Advertising the estimated target state components
    estimated_transform_pub_ =
      create_publisher<geometry_msgs::msg::TransformStamped>("estimated_transform", 10);
    estimated_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("estimated_odometry", 10);
  }

  // Raw vicon data callback.
  void transformStampedCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Extracting the relavent data from the message
    Eigen::Vector3d position_measured_W;
    Eigen::Quaterniond orientation_measured_B_W;
    tf2::fromMsg(msg->transform.translation, position_measured_W);
    tf2::fromMsg(msg->transform.rotation, orientation_measured_B_W);
    rclcpp::Time timestamp = msg->header.stamp;
    // Passing the received data to the estimator
    vicon_odometry_estimator_->updateEstimate(
      position_measured_W, orientation_measured_B_W, timestamp);
    // Retreiving the estimates
    Eigen::Vector3d position_estimate_W = vicon_odometry_estimator_->getEstimatedPosition();
    Eigen::Vector3d velocity_estimate_W = vicon_odometry_estimator_->getEstimatedVelocity();
    Eigen::Quaterniond orientation_estimate_B_W =
      vicon_odometry_estimator_->getEstimatedOrientation();
    Eigen::Vector3d rate_estimate_B = vicon_odometry_estimator_->getEstimatedAngularVelocity();
    // Rotating the estimated global frame velocity into the body frame.
    Eigen::Vector3d velocity_estimate_B =
      orientation_estimate_B_W.toRotationMatrix() * velocity_estimate_W;
    // Creating estimated transform message
    geometry_msgs::msg::TransformStamped estimated_transform;
    estimated_transform.header = msg->header;
    tf2::toMsg(position_estimate_W, estimated_transform.transform.translation);
    estimated_transform.transform.rotation = tf2::toMsg(orientation_estimate_B_W);
    // Creating estimated odometry message
    nav_msgs::msg::Odometry estimated_odometry;
    estimated_odometry.header = msg->header;
    estimated_odometry.child_frame_id = object_name_;
    estimated_odometry.pose.pose.position = tf2::toMsg(position_estimate_W);
    estimated_odometry.pose.pose.orientation = tf2::toMsg(orientation_estimate_B_W);
    tf2::toMsg(velocity_estimate_B, estimated_odometry.twist.twist.linear);
    tf2::toMsg(rate_estimate_B, estimated_odometry.twist.twist.angular);
    // Publishing the estimates
    estimated_transform_pub_->publish(estimated_transform);
    estimated_odometry_pub_->publish(estimated_odometry);
    // Publishing the estimator intermediate results
    vicon_odometry_estimator_->publishIntermediateResults(msg->header.stamp);
  }

private:
  // Raw vicon data subscriber.
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr raw_transform_sub_;
  // Estimate publishers
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr estimated_transform_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odometry_pub_;
  // Name of the tracked object
  std::string object_name_;
  // Vicon-based estimator
  std::shared_ptr<vicon_estimator::ViconOdometryEstimator> vicon_odometry_estimator_;
};

// Standard C++ entry point
int main(int argc, char ** argv)
{
  // Announce this program to the ROS master
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // Creating a Vicon-based estimator to do the estimation
  auto vicon_odometry_estimator =
    std::make_shared<vicon_estimator::ViconOdometryEstimator>(options);

  vicon_odometry_estimator->initializeParameters();
  vicon_odometry_estimator->reset();

  // Creating a Vicon Data Listener to direct vicon data to the estimator
  auto vicon_data_listener = std::make_shared<ViconDataListener>(options, vicon_odometry_estimator);

  // Spinng forever pumping callbacks
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(vicon_odometry_estimator);
  exec.add_node(vicon_data_listener);

  exec.spin();

  rclcpp::shutdown();
  // Exit tranquilly
  return 0;
}
