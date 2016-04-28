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

#include <iostream>
#include <stdio.h>
#include <math.h>

#include "vicon_estimator.h"

namespace vicon_estimator {

/*
 * --------------------------------------------------------------------
 * Vicon Odometry Estimator
 * --------------------------------------------------------------------
 */

ViconEstimator::ViconEstimator()
    : translational_estimator_(), rotational_estimator_() {}

void ViconEstimator::updateEstimate(
    const Eigen::Vector3d& position_measured_W,
    const Eigen::Quaterniond& orientation_measured_B_W) {
  // Updating the translational and rotation sub-estimates
  translational_estimator_.updateEstimate(position_measured_W);
  rotational_estimator_.updateEstimate(orientation_measured_B_W);
}

void ViconEstimator::reset() {
  // Resetting the translational and rotation
  translational_estimator_.reset();
  rotational_estimator_.reset();
}

void ViconEstimator::setParameters(
    const TranslationalEstimatorParameters& translational_estimator_parameters,
    const RotationalEstimatorParameters& rotational_estimator_parameters) {
  translational_estimator_.setParameters(translational_estimator_parameters);
  rotational_estimator_.setParameters(rotational_estimator_parameters);
}

void ViconEstimator::getIntermediateResults(
    TranslationalEstimatorResults* translational_estimator_results,
    RotationalEstimatorResults* rotational_estimator_results) const {
  *translational_estimator_results = translational_estimator_.getResults();
  *rotational_estimator_results = rotational_estimator_.getResults();
}

/*
 * --------------------------------------------------------------------
 * Translational Estimator
 * --------------------------------------------------------------------
 */

TranslationalEstimator::TranslationalEstimator()
    : estimator_parameters_(), estimator_results_() {
  // Resetting the estimator
  reset();
}

void TranslationalEstimator::updateEstimate(
    const Eigen::Vector3d& pos_measured_W) {
  // Saving the measurement to the intermediate results
  estimator_results_.position_measured_ = pos_measured_W;
  // Saving the old state to the intermediate results
  estimator_results_.position_old_ = position_estimate_W_;
  estimator_results_.velocity_old_ = velocity_estimate_W_;
  // Constructing the full state
  Eigen::Matrix<double, 6, 1> x_estimate;
  x_estimate << position_estimate_W_, velocity_estimate_W_;
  // Constructing the system matrix
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(0, 3) = estimator_parameters_.dt_ * Eigen::Matrix3d::Identity();
  // Constructing the measurement matrix
  Eigen::Matrix<double, 3, 6> C;
  C.setZero();
  C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Constructing the Luenberger gain matrix
  Eigen::Matrix<double, 6, 3> L_gain;
  L_gain << estimator_parameters_.kp_* Eigen::Matrix3d::Identity(),
      estimator_parameters_.kv_ * Eigen::Matrix3d::Identity();
  // Correction using the Luenberger equations + gain
  x_estimate = (A - L_gain * C * A) * x_estimate + L_gain * pos_measured_W;
  // Extracting state components
  position_estimate_W_ = x_estimate.block<3, 1>(0, 0);
  velocity_estimate_W_ = x_estimate.block<3, 1>(3, 0);
  // Saving estimate to intermediate results
  estimator_results_.position_estimate_ = position_estimate_W_;
  estimator_results_.velocity_estimate_ = velocity_estimate_W_;
}

void TranslationalEstimator::reset() {
  // Resetting the estimates to their initial values
  position_estimate_W_ = estimator_parameters_.initial_position_estimate_;
  velocity_estimate_W_ = estimator_parameters_.initial_velocity_estimate_;
}

void TranslationalEstimator::setParameters(
    const TranslationalEstimatorParameters& estimator_parameters) {
  estimator_parameters_ = estimator_parameters;
}

/*
 * --------------------------------------------------------------------
 * Rotational Estimator
 * --------------------------------------------------------------------
 */

RotationalEstimator::RotationalEstimator()
    : estimator_parameters_(), estimator_results_() {
  // Resetting the estimator
  reset();
}

void RotationalEstimator::reset() {
  // Resetting the estimates to thier initial values
  orientation_estimate_B_W_ =
      estimator_parameters_.initial_orientation_estimate_;
  rate_estimate_B_ = estimator_parameters_.initial_rate_estimate_;
  dorientation_estimate_ = estimator_parameters_.initial_rate_estimate_;
  drate_estimate_ = estimator_parameters_.initial_drate_estimate_;
  // Reseting to initial covariance
  covariance_
      << estimator_parameters_.dorientation_estimate_initial_covariance_*
         Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drate_estimate_initial_covariance_ *
          Eigen::Matrix3d::Identity();
  // Constructing process and measurement covariance matrices
  process_covariance_ << estimator_parameters_.dorientation_process_covariance_*
                         Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drate_process_covariance_ *
          Eigen::Matrix3d::Identity();
  measurement_covariance_
      << estimator_parameters_.orientation_measurement_covariance_*
         Eigen::Matrix4d::Identity();
  // Setting the old measurement to the intial position estimate
  orientation_measured_old_ =
      estimator_parameters_.initial_orientation_estimate_;
  first_measurement_flag_ = true;
  outlier_counter_ = 0;
}

void RotationalEstimator::setParameters(
    const RotationalEstimatorParameters& estimator_parameters) {
  estimator_parameters_ = estimator_parameters;
}

Eigen::Quaterniond RotationalEstimator::getEstimatedOrientation() const {
  Eigen::Quaterniond orientation_for_return;
  // Swapping to minimal (but redundant) representation if requested
  if (estimator_parameters_.output_minimal_quaternions_) {
    double correction_factor =
        orientation_estimate_B_W_.w() / std::abs(orientation_estimate_B_W_.w());
    orientation_for_return = Eigen::Quaterniond(
        correction_factor * orientation_estimate_B_W_.coeffs());
  } else {
    orientation_for_return = orientation_estimate_B_W_;
  }
  return orientation_for_return;
}

Eigen::Vector3d RotationalEstimator::getEstimatedRate() const {
  return rate_estimate_B_;
}

void RotationalEstimator::updateEstimate(
    const Eigen::Quaterniond& orientation_measured_B_W) {
  // Writing the raw measurement to the intermediate results structure
  estimator_results_.orientation_measured_ = orientation_measured_B_W;

  // Writing the old estimate to the intermediate results structure
  estimator_results_.orientation_old_ = orientation_estimate_B_W_;
  estimator_results_.rate_old_ = rate_estimate_B_;

  // Detecting outlier measurements
  bool measurement_update_flag;
  if (detectMeasurementOutlier(orientation_measured_B_W)) {
    measurement_update_flag = false;
  } else {
    measurement_update_flag = true;
  }

  // Propagating the global state estimate
  Eigen::Matrix<double, 7, 1> x_old;
  Eigen::Matrix<double, 7, 1> x_p;
  x_old << orientation_estimate_B_W_.coeffs(), rate_estimate_B_;
  updateEstimatePropagateGlobalEstimate(x_old, &x_p);

  // Propagating the error state estimate
  Eigen::Matrix<double, 6, 1> dx_old;
  Eigen::Matrix<double, 6, 1> dx_p;
  dx_old << dorientation_estimate_, drate_estimate_;
  updateEstimatePropagateErrorEstimate(dx_old, x_old, &dx_p);

  // Propagating the estimate covariance
  Eigen::Matrix<double, 6, 6> P_old;
  Eigen::Matrix<double, 6, 6> P_p;
  P_old = covariance_;
  updateEstimatePropagateErrorCovariance(P_old, x_old, &P_p);

  // Posteriori variables
  Eigen::Matrix<double, 7, 1> x_m;
  Eigen::Matrix<double, 6, 1> dx_m;
  Eigen::Matrix<double, 6, 6> P_m;

  // If no outlier detected do measurement update
  if (measurement_update_flag) {
    // Measurement Update
    updateEstimateUpdateErrorEstimate(orientation_measured_B_W, x_p, dx_p, P_p,
                                      &dx_m, &P_m);
    // Global state correction (combining priori global state estimate with
    // posteriori error state estimate)
    updateEstimateRecombineErrorGlobal(x_p, &x_m, &dx_m);
  }
  // If outlier detected just write priori estimate to posteriori
  else {
    // Global state correction (combining priori global state estimate with
    // priori error state estimate)
    updateEstimateRecombineErrorGlobal(x_p, &x_m, &dx_p);
    // Writing priori measurement covariance over posteriori covariance
    Eigen::Matrix<double, 6, 6> P_m = P_p;
  }

  // Extracting estimated quantities from the posteriori state
  orientation_estimate_B_W_ = Eigen::Quaterniond(x_m.block<4, 1>(0, 0));
  rate_estimate_B_ = x_m.block<3, 1>(4, 0);
  covariance_ = P_m;

  // Writing the new estimate to the intermediate results structure
  estimator_results_.orientation_estimate_ = orientation_estimate_B_W_;
  estimator_results_.rate_estimate_ = rate_estimate_B_;
}

Eigen::Matrix3d RotationalEstimator::skewMatrix(
    const Eigen::Vector3d& vec) const {
  Eigen::Matrix3d vec_cross;
  vec_cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return vec_cross;
}

void RotationalEstimator::updateEstimatePropagateGlobalEstimate(
    const Eigen::Matrix<double, 7, 1>& x_old,
    Eigen::Matrix<double, 7, 1>* x_p) {
  // Extracting components of the state
  Eigen::Quaterniond orienation_estimate_old =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_old = x_old.block<3, 1>(4, 0);
  // Converting the roll rate vector to quaternion representation
  Eigen::Quaterniond rate_estimate_quaternion = Eigen::Quaterniond(
      0, rate_estimate_old.x(), rate_estimate_old.y(), rate_estimate_old.z());
  // Performing orientation and roll rate propagation
  Eigen::Quaterniond orientation_estimate_priori = Eigen::Quaterniond(
      orienation_estimate_old.coeffs() +
      (0.5 * (orienation_estimate_old * rate_estimate_quaternion).coeffs()) *
          estimator_parameters_.dt_);
  Eigen::Vector3d rate_estimate_priori = rate_estimate_old;
  // Renormalizing the quaternion
  orientation_estimate_priori.normalize();
  // Writing to apriori state
  *x_p << orientation_estimate_priori.coeffs(), rate_estimate_priori;
}

void RotationalEstimator::updateEstimatePropagateErrorEstimate(
    const Eigen::Matrix<double, 6, 1>& dx_old,
    const Eigen::Matrix<double, 7, 1>& x_old,
    Eigen::Matrix<double, 6, 1>* dx_p) {
  // Extracting components of the states
  Eigen::Quaterniond orienation_estimate =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate = x_old.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate = dx_old.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate = dx_old.block<3, 1>(3, 0);
  // Performing propagation
  Eigen::Vector3d dorientation_estimate_priori_roc =
      -rate_estimate.cross(dorientation_estimate) +
      0.5 * drate_estimate;  // Appears to agree with equations in papers
  Eigen::Vector3d drate_estimate_priori_roc = Eigen::Vector3d::Zero();
  Eigen::Vector3d dorientation_estimate_priori =
      dorientation_estimate +
      dorientation_estimate_priori_roc * estimator_parameters_.dt_;
  Eigen::Vector3d drate_estimate_priori =
      drate_estimate + drate_estimate_priori_roc * estimator_parameters_.dt_;
  // Writing to apriori error state
  *dx_p << dorientation_estimate_priori, drate_estimate_priori;
}

void RotationalEstimator::updateEstimatePropagateErrorCovariance(
    Eigen::Matrix<double, 6, 6>& covariance_old,
    const Eigen::Matrix<double, 7, 1>& x_old,
    Eigen::Matrix<double, 6, 6>* covariance_priori) {
  // Extracting components of the state
  Eigen::Quaterniond orientation_estimate =
      Eigen::Quaterniond(x_old.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate = x_old.block<3, 1>(4, 0);
  // Constructing linearized system matrices
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 6> L;
  A << -1 * skewMatrix(rate_estimate), 0.5 * Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
  L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  // Performing propagation
  *covariance_priori = covariance_old +
                       (A * covariance_old + covariance_old * A.transpose() +
                        L * process_covariance_ * (L.transpose())) *
                           estimator_parameters_.dt_;
}

void RotationalEstimator::updateEstimateUpdateErrorEstimate(
    const Eigen::Quaterniond& orientation_measured,
    const Eigen::Matrix<double, 7, 1>& x_priori,
    const Eigen::Matrix<double, 6, 1>& dx_priori,
    const Eigen::Matrix<double, 6, 6>& covariance_priori,
    Eigen::Matrix<double, 6, 1>* dx_m,
    Eigen::Matrix<double, 6, 6>* covariance_measurement) {
  // Extracting components of the state
  Eigen::Quaterniond orientation_estimate_priori =
      Eigen::Quaterniond(x_priori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_priori = x_priori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate_priori =
      dx_priori.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate_priori =
      dx_priori.block<3, 1>(3, 0);
  // Constructing linearized measurement matrix
  Eigen::Matrix<double, 4, 3> Hdq;
  Eigen::Matrix<double, 4, 6> H;
  Eigen::Vector3d orientation_estimate_priori_vector =
      orientation_estimate_priori.vec();
  Hdq << orientation_estimate_priori.w() *
                 Eigen::Matrix<double, 3, 3>::Identity() +
             skewMatrix(orientation_estimate_priori_vector),
      -orientation_estimate_priori.vec().transpose();
  H << Hdq, Eigen::Matrix<double, 4, 3>::Zero();

  // Calculating the measured error quaternion
  Eigen::Quaterniond error_orientation =
      orientation_measured * orientation_estimate_priori.inverse();

  // Calculating the predicted measurement dependant on the sign of the measured
  // error quaternion
  Eigen::Quaterniond orientation_predicted;
  if (error_orientation.w() >= 0) {
    // Assigning the flag indicating that this measurment is not flipped
    estimator_results_.measurement_flip_flag_ = false;
    // Calculating the predicted measurement
    orientation_predicted =
        Eigen::Quaterniond(Hdq * dorientation_estimate_priori +
                           orientation_estimate_priori.coeffs());
  } else {
    // Assigning the flag indicating that this measurment is flipped
    estimator_results_.measurement_flip_flag_ = true;
    // Calculating the predicted measurement
    orientation_predicted =
        Eigen::Quaterniond(Hdq * dorientation_estimate_priori +
                           orientation_estimate_priori.coeffs());
  }

  // Calculating the measurement residual
  Eigen::Vector4d measurement_residual;
  measurement_residual =
      orientation_measured.coeffs() - orientation_predicted.coeffs();

  // Computing the Kalman gain
  Eigen::Matrix<double, 4, 4> S =
      H * covariance_priori * H.transpose() + measurement_covariance_;
  Eigen::Matrix<double, 6, 4> K;
  K = covariance_priori * H.transpose() * S.inverse();

  // Correcting the state
  *dx_m = dx_priori + K* measurement_residual;
  // Updating the covariance
  *covariance_measurement =
      (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covariance_priori;
  //*covariance_measurement = (Eigen::Matrix<double, 6, 6>::Identity() -
  //K*H)*P_p*((Eigen::Matrix<double, 6, 6>::Identity() - K*H).transpose()) +
  //K*measurementCovariance*K.transpose();
}

void RotationalEstimator::updateEstimateRecombineErrorGlobal(
    const Eigen::Matrix<double, 7, 1> x_priori,
    Eigen::Matrix<double, 7, 1>* x_measurement,
    Eigen::Matrix<double, 6, 1>* dx_measurement) {
  // Extracting components of the state
  Eigen::Quaterniond orientation_estimate_priori =
      Eigen::Quaterniond(x_priori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> rate_estimate_priori = x_priori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dorientation_estimate_measurement =
      dx_measurement->block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> drate_estimate_measurement =
      dx_measurement->block<3, 1>(3, 0);

  // Completing the error quaternion
  // The sign real part of the full quaternion was calculated in the error
  // quaternion measurement update step
  Eigen::Quaterniond dorientation_estimate_measurement_quaternion;
  if (estimator_results_.measurement_flip_flag_ == false) {
    dorientation_estimate_measurement_quaternion =
        Eigen::Quaterniond(1.0, dorientation_estimate_measurement.x(),
                           dorientation_estimate_measurement.y(),
                           dorientation_estimate_measurement.z());
  } else {
    dorientation_estimate_measurement_quaternion =
        Eigen::Quaterniond(-1.0, dorientation_estimate_measurement.x(),
                           dorientation_estimate_measurement.y(),
                           dorientation_estimate_measurement.z());
  }

  // Using estimated error states to correct global estimate states
  Eigen::Quaterniond orientation_estimate_measurement =
      orientation_estimate_priori *
      dorientation_estimate_measurement_quaternion;
  Eigen::Matrix<double, 3, 1> rate_estimate_measurement =
      rate_estimate_priori + drate_estimate_measurement;
  // Normalizing the posteriori quaternion
  orientation_estimate_measurement.normalize();
  // Writing to posteriori global state and error states
  *x_measurement << orientation_estimate_measurement.coeffs(),
      rate_estimate_measurement;
  *dx_measurement = Eigen::Matrix<double, 6, 1>::Zero();
}

bool RotationalEstimator::detectMeasurementOutlier(
    const Eigen::Quaterniond& orientation_measured) {
  /*
   *  Method 1 - Detecting the error between the current estimate and the
   *measurement.
   *
   *             Here we compute the rotation between the state estimate and the
   *current
   *             measurement. If the magnitude of this rotation is too large we
   *deem the
   *             the measurement an outlier and disgard it.
   *
   *             The problem with this method is that at initialization the
   *estimated
   *             state and measurement can be legitimate different very
   *different.
   *             TODO(millanea): The solution to this problem, and probably the
   *correct way
   *             to approach this outlier rejection problem, is to compute the
   *likelihood of
   *             the measurement given the current estimate of the state and its
   *variance.
   */
  // Constructing the quaternion representing the body to measurement rotation
  // Eigen::Quaterniond rotation_Z_W = orientation_measured *
  // orientation_estimate_B_W_.inverse();
  // Calculating a measure of the size of the error quaternion
  // double rotation_magnitude_radians =
  // quaternionRotationMagnitude(rotation_Z_W);
  /*
   *  Method 2 - Detecting the error between subsequent measurements.
   *
   *             Here we compute the rotation between the current measurement
   *and the measurement
   *             from the last time step. If the magnitude of this rotation is
   *too large we deem the
   *             the measurement an outlier and disgard it.
   */
  // Performing some initialization if this is the first measurement.
  // Assuming first measurement valid, saving it and returning.
  if (first_measurement_flag_) {
    first_measurement_flag_ = false;
    orientation_measured_old_ = orientation_measured;
    return false;
  }

  // Constructing the quaternion representing the rotation between subsquent
  // measurements
  Eigen::Quaterniond q_Z_Z1_ =
      orientation_measured * orientation_measured_old_.inverse();
  // Calculating the quaternion magnitude
  double q_Z_Z1_magnitude_ = quaternionRotationMagnitude(q_Z_Z1_);

  // Constructing the quaternion representing the rotation between the body a
  // measurement
  Eigen::Quaterniond q_Z_B_ =
      orientation_measured * orientation_estimate_B_W_.inverse();
  // Calculating the quaternion magnitude
  double q_Z_B_magnitude_ = quaternionRotationMagnitude(q_Z_B_);

  // Writing the error quaternions and their magnitude to the intermediate
  // results structure
  estimator_results_.q_Z_Z1_ = q_Z_Z1_;
  estimator_results_.q_Z_B_ = q_Z_B_;

  // Detecting if the measurement is an outlier
  bool measurement_outlier_flag =
      q_Z_Z1_magnitude_ >=
      estimator_parameters_.outlier_threshold_degrees_ * M_PI / 180.0;

  // After a certain number of measurements have been ignored in a row
  // we assume we've made a mistake and accept the measurement as valid.
  if (outlier_counter_ >= estimator_parameters_.maximum_outlier_count_) {
    measurement_outlier_flag = false;
  }

  // Saving the flag to the intermediate results structure
  estimator_results_.measurement_outlier_flag_ = measurement_outlier_flag;

  // If rotation too great indicate that measurement is corrupted
  if (measurement_outlier_flag) {
    ++outlier_counter_;
    return true;
  } else {
    // If measurement valid. Overwriting the old measurement.
    orientation_measured_old_ = orientation_measured;
    outlier_counter_ = 0;
    return false;
  }
}

double RotationalEstimator::quaternionRotationMagnitude(
    const Eigen::Quaterniond& rotation) {
  // Extracting the quaternion magnitude component
  double positive_rotation_return = 2 * acos(rotation.w());
  double negative_rotation_return = 2 * M_PI - 2 * acos(rotation.w());
  if (positive_rotation_return <= M_PI) {
    return positive_rotation_return;
  } else {
    return negative_rotation_return;
  }
}
}
