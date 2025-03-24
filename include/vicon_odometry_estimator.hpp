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

#ifndef VICON_ODOMETRY_ESTIMATOR_H
#define VICON_ODOMETRY_ESTIMATOR_H

#include <math.h>
#include <stdio.h>

#include <Eigen/Geometry>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_vrpn/msg/vicon_estimator.hpp"
#include "vicon_estimator.hpp"

namespace vicon_estimator
{

static const bool kDefaultVerboseFlag = true;

class ViconOdometryEstimator : public rclcpp::Node
{
public:
  // Constructor
  ViconOdometryEstimator(
    const rclcpp::NodeOptions & options,
    const std::string & node_name = "vicon_odometry_estimator");

  // Initialize the estimator parameters
  void initializeParameters();
  // Reset the estimator
  void reset();
  // Publishing the intermediate results
  void publishIntermediateResults(rclcpp::Time timestamp);

  // Calls the underlying estimator, updating the estimate with the latest
  // measurement
  void updateEstimate(
    const Eigen::Vector3d & position_measured_W,
    const Eigen::Quaterniond & orientation_measured_B_W, rclcpp::Time timestamp);

  // Getter methods for estimates values
  Eigen::Vector3d getEstimatedPosition() const { return vicon_estimator_.getEstimatedPosition(); }

  Eigen::Vector3d getEstimatedVelocity() const { return vicon_estimator_.getEstimatedVelocity(); }

  Eigen::Quaterniond getEstimatedOrientation() const
  {
    return vicon_estimator_.getEstimatedOrientation();
  }

  Eigen::Vector3d getEstimatedAngularVelocity() const
  {
    return vicon_estimator_.getEstimatedAngularVelocity();
  }

private:
  // Underlying estimator
  vicon_estimator::ViconEstimator vicon_estimator_;
  // Publisher
  rclcpp::Publisher<ros_vrpn::msg::ViconEstimator>::SharedPtr publisher_;
  // Flag for verbose output
  bool verbose_;
};
}  // namespace vicon_estimator

#endif  // VICON_ODOMETRY_ESTIMATOR_H
