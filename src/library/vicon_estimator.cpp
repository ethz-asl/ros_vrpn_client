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

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <glog/logging.h>

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
    const Eigen::Quaterniond& orientation_measured_B_W,
    const double timestamp) {
  // Updating the translational and rotation sub-estimates
  translational_estimator_status_ =
      translational_estimator_.updateEstimate(position_measured_W, timestamp);
  rotational_estimator_status_ =
      rotational_estimator_.updateEstimate(orientation_measured_B_W, timestamp);
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

void ViconEstimator::getEstimatorStatuses(
    EstimatorStatus* translational_estimator_status,
    EstimatorStatus* rotational_estimator_status) const {
  *translational_estimator_status = translational_estimator_status_;
  *rotational_estimator_status = rotational_estimator_status_;
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

EstimatorStatus TranslationalEstimator::updateEstimate(
    const Eigen::Vector3d& pos_measured_W, const double timestamp) {
  // Performing some initialization if this is the first measurement.
  // Assuming first measurement valid, saving it and returning.
  if (first_measurement_flag_) {
    first_measurement_flag_ = false;
    last_timestamp_ = timestamp;
    pos_measured_old_ = pos_measured_W;
    position_estimate_W_ = pos_measured_W;
    return EstimatorStatus::OK;
  }
  // Calculating the time difference to the last measurement
  double dt = timestamp - last_timestamp_;
  last_timestamp_ = timestamp;

  // Detecting outlier measurements
  bool measurement_update_flag;
  if (detectMeasurementOutlierSubsequent(pos_measured_W)) {
    measurement_update_flag = false;
  } else {
    measurement_update_flag = true;
  }

  // If no outlier detected do measurement update
  if (measurement_update_flag) {
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
    A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
    // Constructing the measurement matrix
    Eigen::Matrix<double, 3, 6> C;
    C.setZero();
    C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    // Constructing the Luenberger gain matrix
    Eigen::Matrix<double, 6, 3> L_gain;
    L_gain << estimator_parameters_.kp_ * Eigen::Matrix3d::Identity(),
        estimator_parameters_.kv_ * Eigen::Matrix3d::Identity();
    // Correction using the Luenberger equations + gain
    x_estimate = (A - L_gain * C * A) * x_estimate + L_gain * pos_measured_W;
    // Extracting state components
    position_estimate_W_ = x_estimate.block<3, 1>(0, 0);
    velocity_estimate_W_ = x_estimate.block<3, 1>(3, 0);
    // Saving estimate to intermediate results
    estimator_results_.position_estimate_ = position_estimate_W_;
    estimator_results_.velocity_estimate_ = velocity_estimate_W_;
    // Returning status
    return EstimatorStatus::OK;
  }
  // If outlier detected just write priori estimate to posteriori
  else {
    // Global state correction (combining priori global state estimate with
    // priori error state estimate)
    estimator_results_.position_estimate_ = estimator_results_.position_old_;
    estimator_results_.velocity_estimate_ = estimator_results_.velocity_old_;
    // Return
    return EstimatorStatus::OUTLIER;
  }
}

void TranslationalEstimator::reset() {
  // Resetting the estimates to their initial values
  position_estimate_W_ = estimator_parameters_.initial_position_estimate_;
  velocity_estimate_W_ = estimator_parameters_.initial_velocity_estimate_;
  // Resetting the previous measurement info
  last_timestamp_ = -1.0;
  first_measurement_flag_ = true;
}

void TranslationalEstimator::setParameters(
    const TranslationalEstimatorParameters& estimator_parameters) {
  estimator_parameters_ = estimator_parameters;
}

bool TranslationalEstimator::detectMeasurementOutlierSubsequent(
    const Eigen::Vector3d& pos_measured) {
  // Performing some initialization if this is the first measurement.
  // Assuming first measurement valid, saving it and returning.
  if (first_measurement_flag_) {
    first_measurement_flag_ = false;
    pos_measured_old_ = pos_measured;
    return false;
  }

  // Compare new measurement with old measuerement and constructing error vector
  Eigen::Vector3d error = (pos_measured_old_ - pos_measured);
  // Detecting if the measurement is an outlier. If the
  bool measurement_outlier_flag =
      (error.norm() >= estimator_parameters_.outlier_threshold_meters_);

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
    pos_measured_old_ = pos_measured;
    outlier_counter_ = 0;
    return false;
  }
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
      << estimator_parameters_.dorientation_estimate_initial_covariance_ *
             Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drate_estimate_initial_covariance_ *
          Eigen::Matrix3d::Identity();
  // Constructing process and measurement covariance matrices
  process_covariance_
      << estimator_parameters_.dorientation_process_covariance_ *
             Eigen::Matrix3d::Identity(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      estimator_parameters_.drate_process_covariance_ *
          Eigen::Matrix3d::Identity();
  measurement_covariance_
      << estimator_parameters_.orientation_measurement_covariance_ *
             Eigen::Matrix4d::Identity();
  // Setting the old measurement to the intial position estimate
  last_timestamp_ = -1.0;
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

EstimatorStatus RotationalEstimator::updateEstimate(
    const Eigen::Quaterniond& orientation_measured_B_W,
    const double timestamp) {
  // Writing the raw measurement to the intermediate results structure
  estimator_results_.orientation_measured_ = orientation_measured_B_W;

  // Writing the old estimate to the intermediate results structure
  estimator_results_.orientation_old_ = orientation_estimate_B_W_;
  estimator_results_.rate_old_ = rate_estimate_B_;

  // Performing some initialization if this is the first measurement.
  if (first_measurement_flag_) {
    first_measurement_flag_ = false;
    orientation_measured_old_ = orientation_measured_B_W;
    last_timestamp_ = timestamp;
    orientation_estimate_B_W_ = orientation_measured_B_W;
    return EstimatorStatus::OK;
  }

  // Calculating the time difference to the last measurement
  double dt = timestamp - last_timestamp_;
  last_timestamp_ = timestamp;

  // Propagating the global state estimate
  Eigen::Matrix<double, 7, 1> x_old;
  Eigen::Matrix<double, 7, 1> x_p;
  x_old << orientation_estimate_B_W_.coeffs(), rate_estimate_B_;
  updateEstimatePropagateGlobalEstimate(x_old, dt, &x_p);

  // Propagating the error state estimate
  Eigen::Matrix<double, 6, 1> dx_old;
  Eigen::Matrix<double, 6, 1> dx_p;
  dx_old << dorientation_estimate_, drate_estimate_;
  updateEstimatePropagateErrorEstimate(dx_old, x_old, dt, &dx_p);

  // Propagating the estimate covariance
  Eigen::Matrix<double, 6, 6> P_old;
  Eigen::Matrix<double, 6, 6> P_p;
  P_old = covariance_;
  updateEstimatePropagateErrorCovariance(P_old, x_old, dt, &P_p);

  // Posteriori variables
  Eigen::Matrix<double, 7, 1> x_m;
  Eigen::Matrix<double, 6, 1> dx_m;
  Eigen::Matrix<double, 6, 6> P_m;

  // Detecting outlier measurements
  bool measurement_update_flag;
  if (estimator_parameters_.outlier_rejection_method_ ==
      OutlierRejectionMethod::MAHALANOBIS_DISTANCE) {
    measurement_update_flag =
        !detectMeasurementOutlierMahalanobis(orientation_measured_B_W, P_p);
  } else if (estimator_parameters_.outlier_rejection_method_ ==
             OutlierRejectionMethod::SUBSEQUENT_MEASUREMENTS) {
    measurement_update_flag =
        !detectMeasurementOutlierSubsequent(orientation_measured_B_W);
  } else {
    measurement_update_flag = true;
  }

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
    P_m = P_p;
  }

  // Extracting estimated quantities from the posteriori state
  orientation_estimate_B_W_ = Eigen::Quaterniond(x_m.block<4, 1>(0, 0));
  rate_estimate_B_ = x_m.block<3, 1>(4, 0);
  covariance_ = P_m;

  // Writing the new estimate to the intermediate results structure
  estimator_results_.orientation_estimate_ = orientation_estimate_B_W_;
  estimator_results_.rate_estimate_ = rate_estimate_B_;

  // Writing the Covariance to intermediate results structure
  // TODO(alexmillane): This should probably just be written in the pose
  // message.
  estimator_results_.covariance_ = covariance_;
  estimator_results_.q_covariance_trace_ = covariance_.trace();

  // Resetting estimator if it has crashed
  bool estimator_crash_flag = checkForEstimatorCrash();
  if (estimator_crash_flag) {
    reset();
  }

  // Returning the estimator status
  if (estimator_crash_flag) {
    return EstimatorStatus::RESET;
  } else {
    if (measurement_update_flag) {
      return EstimatorStatus::OK;
    } else {
      return EstimatorStatus::OUTLIER;
    }
  }
}

Eigen::Matrix3d RotationalEstimator::skewMatrix(
    const Eigen::Vector3d& vec) const {
  Eigen::Matrix3d vec_cross;
  vec_cross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return vec_cross;
}

void RotationalEstimator::updateEstimatePropagateGlobalEstimate(
    const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
    Eigen::Matrix<double, 7, 1>* x_p) {
  // Argument checks
  CHECK_NOTNULL(x_p);
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
          dt);
  Eigen::Vector3d rate_estimate_priori = rate_estimate_old;
  // Renormalizing the quaternion
  orientation_estimate_priori.normalize();
  // Writing to apriori state
  *x_p << orientation_estimate_priori.coeffs(), rate_estimate_priori;
}

void RotationalEstimator::updateEstimatePropagateErrorEstimate(
    const Eigen::Matrix<double, 6, 1>& dx_old,
    const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
    Eigen::Matrix<double, 6, 1>* dx_p) {
  // Argument checks
  CHECK_NOTNULL(dx_p);
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
      dorientation_estimate + dorientation_estimate_priori_roc * dt;
  Eigen::Vector3d drate_estimate_priori =
      drate_estimate + drate_estimate_priori_roc * dt;
  // Writing to apriori error state
  *dx_p << dorientation_estimate_priori, drate_estimate_priori;
}

void RotationalEstimator::updateEstimatePropagateErrorCovariance(
    Eigen::Matrix<double, 6, 6>& covariance_old,
    const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
    Eigen::Matrix<double, 6, 6>* covariance_priori) {
  // Argument checks
  CHECK_NOTNULL(covariance_priori);
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
  // Performing propagation ( P_kp1=A_d * P_k * A_d' + L * Qd * L' )
  Eigen::Matrix<double, 6, 6> covariance_priori_propagation;
  Eigen::Matrix<double, 6, 6> covariance_priori_addition;
  Eigen::Matrix<double, 6, 6> A_d =
      Eigen::Matrix<double, 6, 6>::Identity() + A * dt;
  covariance_priori_propagation = A_d * covariance_old * A_d.transpose();
  covariance_priori_addition = L * process_covariance_ * (L.transpose()) * dt;
  *covariance_priori =
      covariance_priori_propagation + covariance_priori_addition;
  // Ensuring symmetry and writing output
  makeCovarianceSymmetric(covariance_priori);
}

void RotationalEstimator::updateEstimateUpdateErrorEstimate(
    const Eigen::Quaterniond& orientation_measured,
    const Eigen::Matrix<double, 7, 1>& x_priori,
    const Eigen::Matrix<double, 6, 1>& dx_priori,
    const Eigen::Matrix<double, 6, 6>& covariance_priori,
    Eigen::Matrix<double, 6, 1>* dx_m,
    Eigen::Matrix<double, 6, 6>* covariance_measurement) {
  // Argument checks
  CHECK_NOTNULL(dx_m);
  CHECK_NOTNULL(covariance_measurement);
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
  Eigen::Matrix<double, 4, 6> b = H * covariance_priori;
  Eigen::Matrix<double, 4, 4> A = S.transpose();
  Eigen::Matrix<double, 4, 6> x = A.colPivHouseholderQr().solve(b);
  Eigen::Matrix<double, 6, 4> K = x.transpose();
  // Correcting the state
  *dx_m = dx_priori + K * measurement_residual;
  // Updating the covariance
  *covariance_measurement =
      (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covariance_priori;
  // Making the covariance matrix symmetric
  makeCovarianceSymmetric(covariance_measurement);
}

void RotationalEstimator::updateEstimateRecombineErrorGlobal(
    const Eigen::Matrix<double, 7, 1> x_priori,
    Eigen::Matrix<double, 7, 1>* x_measurement,
    Eigen::Matrix<double, 6, 1>* dx_measurement) {
  // Argument checks
  CHECK_NOTNULL(x_measurement);
  CHECK_NOTNULL(dx_measurement);
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

bool RotationalEstimator::checkForEstimatorCrash() {
  // Testing if covariance matrix member is nan.
  double nan_test = covariance_.sum();
  return std::isnan(nan_test);
}

bool RotationalEstimator::detectMeasurementOutlierMahalanobis(
    const Eigen::Quaterniond& orientation_measured,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  // Initializing the outlier flag
  bool measurement_outlier_flag = false;
  // Calculating the mahalanobis distance (3x3 matrix - Should be fine for
  // direct inverse).
  Eigen::Quaterniond q_Z_B =
      orientation_measured * orientation_estimate_B_W_.inverse();
  Eigen::Vector3d dq_Z_B_ = q_Z_B.vec();
  Eigen::Matrix<double, 3, 3> dq_covariance = covariance.block<3, 3>(0, 0);
  double q_Z_B_mahalanobis_distance =
      sqrt(dq_Z_B_.transpose() * dq_covariance.inverse() * dq_Z_B_);
  // Detecting outlier
  measurement_outlier_flag =
      q_Z_B_mahalanobis_distance >=
      estimator_parameters_.outlier_rejection_mahalanobis_threshold_;
  // Writing the intermediate results (for debug)
  estimator_results_.q_Z_Z1_magnitude_ = -1.0;
  estimator_results_.q_Z_B_mahalanobis_distance_ = q_Z_B_mahalanobis_distance;
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

bool RotationalEstimator::detectMeasurementOutlierSubsequent(
    const Eigen::Quaterniond& orientation_measured) {
  // Initializing the outlier flag
  bool measurement_outlier_flag = false;
  // Constructing the quaternion representing the rotation between subsquent
  // measurements
  Eigen::Quaterniond q_Z_Z1 =
      orientation_measured * orientation_measured_old_.inverse();
  // Calculating the quaternion magnitude
  double q_Z_Z1_magnitude = quaternionRotationMagnitude(q_Z_Z1);
  // Detecting outlier
  measurement_outlier_flag =
      q_Z_Z1_magnitude >=
      estimator_parameters_.outlier_rejection_subsequent_threshold_degrees_ *
          M_PI / 180.0;
  // After a certain number of measurements have been ignored in a row
  // we assume we've made a mistake and accept the measurement as valid.
  if (outlier_counter_ >=
      estimator_parameters_.outlier_rejection_subsequent_maximum_count_) {
    measurement_outlier_flag = false;
  }
  // Writing the intermediate results (for debug)
  estimator_results_.q_Z_Z1_magnitude_ = q_Z_Z1_magnitude;
  estimator_results_.q_Z_B_mahalanobis_distance_ = -1.0;
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

void RotationalEstimator::makeCovarianceSymmetric(
    Eigen::Matrix<double, 6, 6>* covariance) {
  // Argument checks
  CHECK_NOTNULL(covariance);
  *covariance = (*covariance + covariance->transpose()) / 2;
}
}
