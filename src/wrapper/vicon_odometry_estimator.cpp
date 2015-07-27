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

#include "vicon_odometry_estimator.h"

namespace viconEstimator {

ViconOdometryEstimator::ViconOdometryEstimator(ros::NodeHandle& nh)
    : viconEstimator_()
{
  // Creating publisher for intermediate estimator values
  publisher_ = nh.advertise<ros_vrpn_client::viconEstimator>("viconEstimator", 100);
}

void ViconOdometryEstimator::initializeParameters(ros::NodeHandle& nh)
{
  // Recovering translational estimator parameters values from the parameter server
  viconEstimator::TranslationalEstimatorParameters translationalEstimatorParameters;
  nh.getParam("translational_estimator/dt", translationalEstimatorParameters.dt);
  nh.getParam("translational_estimator/kp", translationalEstimatorParameters.kp);
  nh.getParam("translational_estimator/kv", translationalEstimatorParameters.kv);
  // Recovering rotational estimator parameters values from the parameter server
  viconEstimator::RotationalEstimatorParameters rotationalEstimatorParameters;
  nh.getParam("rotational_estimator/dt", rotationalEstimatorParameters.dt);
  nh.getParam("rotational_estimator/orientation_estimate_initial_covariance", rotationalEstimatorParameters.dQuatHatInitialCovariance);
  nh.getParam("rotational_estimator/roll_rate_estimate_initial_covariance", rotationalEstimatorParameters.dOmegaHatInitialCovariance);
  nh.getParam("rotational_estimator/orientation_process_covariance",rotationalEstimatorParameters.dQuatProcessCovariance);
  nh.getParam("rotational_estimator/roll_rate_process_covariance",rotationalEstimatorParameters.dOmegaProcessCovariance);
  nh.getParam("rotational_estimator/orientation_measurementCovariance", rotationalEstimatorParameters.quatMeasurementCovariance);
  // Setting parameters in estimator
  viconEstimator_.setParameters(translationalEstimatorParameters, rotationalEstimatorParameters);
}

void ViconOdometryEstimator::reset()
{
  viconEstimator_.reset();
}

void ViconOdometryEstimator::publishResults(ros::Time timestamp)
{

  viconEstimator::TranslationalEstimatorResults translationalEstimatorResults;
  viconEstimator::RotationalEstimatorResults rotationalEstimatorResults;
  viconEstimator_.getIntermediateResults(&translationalEstimatorResults,
                                         &rotationalEstimatorResults);

  // Creating estimator message
  ros_vrpn_client::viconEstimator msg;
  // Attaching the vprn timestamp
  msg.header.stamp = timestamp;

  // Writing the measurement to the message object
  msg.pos_measured.x = translationalEstimatorResults.posMeasured.x();
  msg.pos_measured.y = translationalEstimatorResults.posMeasured.y();
  msg.pos_measured.z = translationalEstimatorResults.posMeasured.z();
  // Writing the old estimates to the message object
  msg.pos_old.x = translationalEstimatorResults.posOld.x();
  msg.pos_old.y = translationalEstimatorResults.posOld.y();
  msg.pos_old.z = translationalEstimatorResults.posOld.z();
  msg.vel_old.x = translationalEstimatorResults.velOld.x();
  msg.vel_old.y = translationalEstimatorResults.velOld.y();
  msg.vel_old.z = translationalEstimatorResults.velOld.z();
  // Posteriori results
  msg.pos_est.x = translationalEstimatorResults.posEst.x();
  msg.pos_est.y = translationalEstimatorResults.posEst.y();
  msg.pos_est.z = translationalEstimatorResults.posEst.z();
  msg.vel_est.x = translationalEstimatorResults.velEst.x();
  msg.vel_est.y = translationalEstimatorResults.velEst.y();
  msg.vel_est.z = translationalEstimatorResults.velEst.z();

  // Writing the measurement to the message object
  msg.quat_measured.w = rotationalEstimatorResults.quatMeasured.w();
  msg.quat_measured.x = rotationalEstimatorResults.quatMeasured.x();
  msg.quat_measured.y = rotationalEstimatorResults.quatMeasured.y();
  msg.quat_measured.z = rotationalEstimatorResults.quatMeasured.z();
  // Writing the old estimates to the message object
  msg.quat_old.w = rotationalEstimatorResults.quatOld.w();
  msg.quat_old.x = rotationalEstimatorResults.quatOld.x();
  msg.quat_old.y = rotationalEstimatorResults.quatOld.y();
  msg.quat_old.z = rotationalEstimatorResults.quatOld.z();
  msg.omega_old.x = rotationalEstimatorResults.omegaOld.x();
  msg.omega_old.y = rotationalEstimatorResults.omegaOld.y();
  msg.omega_old.z = rotationalEstimatorResults.omegaOld.z();
  // Posteriori results
  msg.quat_est.w = rotationalEstimatorResults.quatEst.w();
  msg.quat_est.x = rotationalEstimatorResults.quatEst.x();
  msg.quat_est.y = rotationalEstimatorResults.quatEst.y();
  msg.quat_est.z = rotationalEstimatorResults.quatEst.z();
  msg.omega_est.x = rotationalEstimatorResults.omegaEst.x();
  msg.omega_est.y = rotationalEstimatorResults.omegaEst.y();
  msg.omega_est.z = rotationalEstimatorResults.omegaEst.z();

  // Publishing estimator message
  publisher_.publish(msg);
}

void ViconOdometryEstimator::updateEstimate(const Eigen::Vector3d& posMeasured,
                                            const Eigen::Quaterniond& quatMeasured)
{
  viconEstimator_.updateEstimate(posMeasured, quatMeasured);
}

}
