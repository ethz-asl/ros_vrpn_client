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

#include <eigen_conversions/eigen_msg.h>

#include "vicon_odometry_estimator.h"

namespace vicon_estimator {

ViconOdometryEstimator::ViconOdometryEstimator(ros::NodeHandle& nh)
    : vicon_estimator_(), verbose_(kDefaultVerboseFlag) {
  // Creating publisher for intermediate estimator values
  publisher_ = nh.advertise<ros_vrpn_client::viconEstimator>(
      "vicon_intermediate_results", 100);
}

void ViconOdometryEstimator::initializeParameters(ros::NodeHandle& nh) {
  // Recovering translational estimator parameters values from the parameter
  // server
  vicon_estimator::TranslationalEstimatorParameters
      translationalEstimatorParameters;
  nh.getParam("translational_estimator/kp",
              translationalEstimatorParameters.kp_);
  nh.getParam("translational_estimator/kv",
              translationalEstimatorParameters.kv_);
  // Recovering rotational estimator parameters values from the parameter server
  vicon_estimator::RotationalEstimatorParameters rotationalEstimatorParameters;
  nh.getParam(
      "rotational_estimator/orientation_estimate_initial_covariance",
      rotationalEstimatorParameters.dorientation_estimate_initial_covariance_);
  nh.getParam("rotational_estimator/rate_estimate_initial_covariance",
              rotationalEstimatorParameters.drate_estimate_initial_covariance_);
  nh.getParam("rotational_estimator/orientation_process_covariance",
              rotationalEstimatorParameters.dorientation_process_covariance_);
  nh.getParam("rotational_estimator/rate_process_covariance",
              rotationalEstimatorParameters.drate_process_covariance_);
  nh.getParam(
      "rotational_estimator/orientation_measurementCovariance",
      rotationalEstimatorParameters.orientation_measurement_covariance_);

  // Getting outlier rejection type
  std::string outlier_rejection_method_string;
  nh.getParam("rotational_estimator/outlier_rejection_method",
              outlier_rejection_method_string);

  if (!outlier_rejection_method_string.compare("mahalanobis_distance")) {
    rotationalEstimatorParameters.outlier_rejection_method_ =
        OutlierRejectionMethod::MAHALANOBIS_DISTANCE;
  } else if (!outlier_rejection_method_string.compare(
                 "subsequent_measurements")) {
    rotationalEstimatorParameters.outlier_rejection_method_ =
        OutlierRejectionMethod::SUBSEQUENT_MEASUREMENTS;
  } else if (!outlier_rejection_method_string.compare("none")) {
    rotationalEstimatorParameters.outlier_rejection_method_ =
        OutlierRejectionMethod::NONE;
  } else {
    ROS_WARN_STREAM(
        "Outlier rejection method requested ("
        << outlier_rejection_method_string
        << ") Not recognized. Please select one of "
           "\"mahalanobis_distance\", \"subsequent_measurements\", \"none\". "
           "Using default value.");
  }

  nh.getParam(
      "rotational_estimator/outlier_rejection_mahalanobis_threshold",
      rotationalEstimatorParameters.outlier_rejection_mahalanobis_threshold_);

  nh.getParam(
      "rotational_estimator/outlier_rejection_subsequent_threshold_degrees",
      rotationalEstimatorParameters
          .outlier_rejection_subsequent_threshold_degrees_);
  nh.getParam("rotational_estimator/outlier_rejection_subsequent_maximum_count",
              rotationalEstimatorParameters
                  .outlier_rejection_subsequent_maximum_count_);

  nh.getParam("rotational_estimator/output_minimal_quaternions",
              rotationalEstimatorParameters.output_minimal_quaternions_);

  nh.getParam("verbose", verbose_);

  // Setting parameters in estimator
  vicon_estimator_.setParameters(translationalEstimatorParameters,
                                 rotationalEstimatorParameters);
}

void ViconOdometryEstimator::reset() { vicon_estimator_.reset(); }

void ViconOdometryEstimator::publishIntermediateResults(ros::Time timestamp) {
  // TODO(millanea): Publishing of the intermediate results was useful when
  //                constructing the estimator however is not likely to still
  //                be useful. Should be removed.

  vicon_estimator::TranslationalEstimatorResults
      translational_estimator_results;
  vicon_estimator::RotationalEstimatorResults rotational_estimator_results;
  vicon_estimator_.getIntermediateResults(&translational_estimator_results,
                                          &rotational_estimator_results);

  // Creating estimator message
  ros_vrpn_client::viconEstimator msg;
  // Attaching the vprn timestamp
  msg.header.stamp = timestamp;

  // Writing the measurement to the message object
  tf::vectorEigenToMsg(translational_estimator_results.position_measured_,
                       msg.pos_measured);
  // Writing the old estimates to the message object
  tf::vectorEigenToMsg(translational_estimator_results.position_old_,
                       msg.pos_old);
  tf::vectorEigenToMsg(translational_estimator_results.velocity_old_,
                       msg.vel_old);
  // Posteriori results
  tf::vectorEigenToMsg(translational_estimator_results.position_estimate_,
                       msg.pos_est);
  tf::vectorEigenToMsg(translational_estimator_results.velocity_estimate_,
                       msg.vel_est);

  // Writing the measurement to the message object
  tf::quaternionEigenToMsg(rotational_estimator_results.orientation_measured_,
                           msg.quat_measured);
  // Writing the old estimates to the message object
  tf::quaternionEigenToMsg(rotational_estimator_results.orientation_old_,
                           msg.quat_old);
  tf::vectorEigenToMsg(rotational_estimator_results.rate_old_, msg.omega_old);
  // Posteriori results
  tf::quaternionEigenToMsg(rotational_estimator_results.orientation_estimate_,
                           msg.quat_est);
  tf::vectorEigenToMsg(rotational_estimator_results.rate_estimate_,
                       msg.omega_est);

  // Data to do with the orientation measurement outlier detection
  msg.outlier_flag.data =
      rotational_estimator_results.measurement_outlier_flag_;
  msg.measurement_flip_flag.data =
      rotational_estimator_results.measurement_flip_flag_;
  msg.q_Z_Z1_magnitude.data = rotational_estimator_results.q_Z_Z1_magnitude_;
  msg.q_Z_B_mahalanobis_distance.data =
      rotational_estimator_results.q_Z_B_mahalanobis_distance_;
  msg.q_covariance_trace.data =
      rotational_estimator_results.q_covariance_trace_;

  // Writing the covariance matrix
  msg.covariance.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.covariance.layout.dim[0].size = 6;
  msg.covariance.layout.dim[0].stride = 36;
  msg.covariance.layout.dim[0].label = "cov_x";
  msg.covariance.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.covariance.layout.dim[1].size = 6;
  msg.covariance.layout.dim[1].stride = 1;
  msg.covariance.layout.dim[1].label = "cov_y";
  msg.covariance.data =
      std::vector<double>(rotational_estimator_results.covariance_.data(),
                          rotational_estimator_results.covariance_.data() +
                              rotational_estimator_results.covariance_.size());

  // Publishing estimator message
  publisher_.publish(msg);
}

void ViconOdometryEstimator::updateEstimate(
    const Eigen::Vector3d& position_measured_W,
    const Eigen::Quaterniond& orientation_measured_B_W, ros::Time timestamp) {
  // Converting the ros time stamp to double
  double timestamp_double = timestamp.toSec();
  // Updating the estimates
  vicon_estimator_.updateEstimate(position_measured_W, orientation_measured_B_W,
                                  timestamp_double);
  // Getting the estimator statuses
  EstimatorStatus rotational_estimator_status;
  EstimatorStatus translational_estimator_status;
  vicon_estimator_.getEstimatorStatuses(&translational_estimator_status,
                                        &rotational_estimator_status);
  if (verbose_) {
    if (translational_estimator_status == EstimatorStatus::RESET) {
      ROS_WARN("Estimator crashed and restarted: translational estimator");
    } else if (translational_estimator_status == EstimatorStatus::OUTLIER) {
      ROS_WARN("Outlier detected: translational estimator");
    }
    if (rotational_estimator_status == EstimatorStatus::RESET) {
      ROS_WARN("Estimator crashed and restarted: rotational estimator");
    } else if (rotational_estimator_status == EstimatorStatus::OUTLIER) {
      ROS_WARN("Outlier detected: rotational estimator");
    }
  }
}
}
