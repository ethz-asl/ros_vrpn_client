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

#ifndef VICON_ESTIMATOR_H
#define VICON_ESTIMATOR_H

#include <Eigen/Geometry>

namespace vicon_estimator {

// Estimator Status enum
enum class EstimatorStatus { OK, OUTLIER, RESET };

enum class OutlierRejectionMethod {
  MAHALANOBIS_DISTANCE,
  SUBSEQUENT_MEASUREMENTS,
  NONE
};

// The parameter class for the translational estimator and parameter default
// values
static const double kDefaultTranslationalKp = 1.0;
static const double kDefaultTranslationalKv = 10.0;
static const double kDefaultOutlierThresholdMeters = 0.5;
static const int kDefaultMaximumOutlierCountTranslation = 10;

static const Eigen::Vector3d kDefaultInitialPositionEstimate =
    Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialVelocityEstimate =
    Eigen::Vector3d::Zero();

class TranslationalEstimatorParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimatorParameters()
      : kp_(kDefaultTranslationalKp),
        kv_(kDefaultTranslationalKv),
        outlier_threshold_meters_(kDefaultOutlierThresholdMeters),
        maximum_outlier_count_(kDefaultMaximumOutlierCountTranslation),
        initial_position_estimate_(kDefaultInitialPositionEstimate),
        initial_velocity_estimate_(kDefaultInitialVelocityEstimate) {}

  double kp_;
  double kv_;
  double outlier_threshold_meters_;
  int maximum_outlier_count_;
  Eigen::Vector3d initial_position_estimate_;
  Eigen::Vector3d initial_velocity_estimate_;
};

class TranslationalEstimatorResults {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimatorResults()
      : position_measured_(Eigen::Vector3d::Zero()),
        position_old_(Eigen::Vector3d::Zero()),
        velocity_old_(Eigen::Vector3d::Zero()),
        measurement_outlier_flag_(false),
        position_estimate_(Eigen::Vector3d::Zero()),
        velocity_estimate_(Eigen::Vector3d::Zero()) {}

  // Intermediate Estimator results
  Eigen::Vector3d position_measured_;
  Eigen::Vector3d position_old_;
  Eigen::Vector3d velocity_old_;
  bool measurement_outlier_flag_;
  Eigen::Vector3d position_estimate_;
  Eigen::Vector3d velocity_estimate_;
};

// Estimated object position and velocity from vicon data
class TranslationalEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimator();
  // Update estimated quantities with new measurement
  EstimatorStatus updateEstimate(const Eigen::Vector3d& pos_measured_W,
                                 const double timestamp);
  // Reset the estimator
  void reset();
  // Setting the estimator parameters
  void setParameters(
      const TranslationalEstimatorParameters& estimator_parameters);
  // Return intermediate results structure
  TranslationalEstimatorResults getResults() const {
    return estimator_results_;
  }
  // Return estimated position
  Eigen::Vector3d getEstimatedPosition() const { return position_estimate_W_; }
  // Return estimated velocity
  Eigen::Vector3d getEstimatedVelocity() const { return velocity_estimate_W_; }

 private:
  // Estimator parameters
  TranslationalEstimatorParameters estimator_parameters_;
  TranslationalEstimatorResults estimator_results_;

  // Last measurement
  Eigen::Vector3d pos_measured_old_;
  bool first_measurement_flag_;
  int outlier_counter_;
  double last_timestamp_;

  // Estimates
  Eigen::Vector3d position_estimate_W_;
  Eigen::Vector3d velocity_estimate_W_;

  // Detects if the passed measurement is an outlier
  bool detectMeasurementOutlierSubsequent(const Eigen::Vector3d& pos_measured);
};

// The parameter class for the translational estimator and parameter default
// values
static const double kDefaultdOrientationEstimateInitialCovariance = 1;
static const double kDefaultdRateEstimateInitialCovariance = 1;
static const double kDefaultdOrientationProcessCovariance = 0.01;
static const double kDefaultdRateProcessCovariance = 1;
static const double kDefaultOrientationMeasurementCovariance = 0.0005;
static const Eigen::Quaterniond kDefaultInitialOrientationEstimate =
    Eigen::Quaterniond::Identity();
static const Eigen::Vector3d kDefaultInitialRateEstimate =
    Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialDorientationEstimate =
    Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialDrateEstimate =
    Eigen::Vector3d::Zero();
static const OutlierRejectionMethod kDefaultOutlierRejectionMethod =
    OutlierRejectionMethod::MAHALANOBIS_DISTANCE;

static const double kDefaultOutlierRejectionMahalanobisThreshold = 5.0;

static const double kDefaultOutlierRejectionSubsequentThresholdDegrees = 30.0;
static const int kDefaultOutlierRejectionSubsequentMaximumCountRotation = 10.0;

static const bool kOutputMinimalQuaternions = false;

class RotationalEstimatorParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimatorParameters()
      : dorientation_estimate_initial_covariance_(
            kDefaultdOrientationEstimateInitialCovariance),
        drate_estimate_initial_covariance_(
            kDefaultdRateEstimateInitialCovariance),
        dorientation_process_covariance_(kDefaultdOrientationProcessCovariance),
        drate_process_covariance_(kDefaultdRateProcessCovariance),
        orientation_measurement_covariance_(
            kDefaultOrientationMeasurementCovariance),
        initial_orientation_estimate_(kDefaultInitialOrientationEstimate),
        initial_rate_estimate_(kDefaultInitialRateEstimate),
        initial_dorientation_estimate_(kDefaultInitialDorientationEstimate),
        initial_drate_estimate_(kDefaultInitialDrateEstimate),
        outlier_rejection_method_(kDefaultOutlierRejectionMethod),
        outlier_rejection_mahalanobis_threshold_(
            kDefaultOutlierRejectionMahalanobisThreshold),
        outlier_rejection_subsequent_threshold_degrees_(
            kDefaultOutlierRejectionSubsequentThresholdDegrees),
        outlier_rejection_subsequent_maximum_count_(
            kDefaultOutlierRejectionSubsequentMaximumCountRotation),
        output_minimal_quaternions_(kOutputMinimalQuaternions){};

  double dorientation_estimate_initial_covariance_;
  double drate_estimate_initial_covariance_;
  double dorientation_process_covariance_;
  double drate_process_covariance_;
  double orientation_measurement_covariance_;
  Eigen::Quaterniond initial_orientation_estimate_;
  Eigen::Vector3d initial_rate_estimate_;
  Eigen::Vector3d initial_dorientation_estimate_;
  Eigen::Vector3d initial_drate_estimate_;
  OutlierRejectionMethod outlier_rejection_method_;
  double outlier_rejection_mahalanobis_threshold_;
  double outlier_rejection_subsequent_threshold_degrees_;
  int outlier_rejection_subsequent_maximum_count_;
  bool output_minimal_quaternions_;
};

class RotationalEstimatorResults {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimatorResults()
      : orientation_measured_(Eigen::Quaterniond::Identity()),
        orientation_old_(Eigen::Quaterniond::Identity()),
        rate_old_(Eigen::Vector3d::Zero()),
        orientation_estimate_(Eigen::Quaterniond::Identity()),
        rate_estimate_(Eigen::Vector3d::Zero()),
        covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        measurement_outlier_flag_(false),
        measurement_flip_flag_(false),
        q_Z_Z1_magnitude_(0.0),
        q_Z_B_mahalanobis_distance_(0.0),
        q_covariance_trace_(0.0){};

  // Intermediate Estimator results
  Eigen::Quaterniond orientation_measured_;
  Eigen::Quaterniond orientation_old_;
  Eigen::Vector3d rate_old_;
  Eigen::Quaterniond orientation_estimate_;
  Eigen::Vector3d rate_estimate_;
  Eigen::Matrix<double, 6, 6> covariance_;
  bool measurement_outlier_flag_;
  bool measurement_flip_flag_;
  double q_Z_Z1_magnitude_;
  double q_Z_B_mahalanobis_distance_;
  double q_covariance_trace_;
};

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimator();
  // Update estimated quantities with new measurement
  EstimatorStatus updateEstimate(
      const Eigen::Quaterniond& orientation_measured_B_W,
      const double timestamp);
  // Reset the estimator
  void reset();
  // Setting the estimator parameters
  void setParameters(const RotationalEstimatorParameters& estimator_parameters);
  // Return intermediate results structure
  RotationalEstimatorResults getResults() const { return estimator_results_; }
  // Return estimated orientation
  Eigen::Quaterniond getEstimatedOrientation() const;
  // Return estimated angular velocity
  Eigen::Vector3d getEstimatedRate() const;

 private:
  // Estimator parameters
  RotationalEstimatorParameters estimator_parameters_;
  // Estimator (intermediate) results
  RotationalEstimatorResults estimator_results_;

  // Global estimates
  Eigen::Quaterniond orientation_estimate_B_W_;
  Eigen::Vector3d rate_estimate_B_;
  // Error estimates
  Eigen::Vector3d dorientation_estimate_;
  Eigen::Vector3d drate_estimate_;
  // Covariance Estimates
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_covariance_;
  Eigen::Matrix<double, 4, 4> measurement_covariance_;

  // Last measurement
  double last_timestamp_;
  Eigen::Quaterniond orientation_measured_old_;
  bool first_measurement_flag_;
  int outlier_counter_;

  // Function to generate a skew symmetric matrix from a vector
  Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec) const;
  // Serial of functions performing the estimate update steps
  void updateEstimatePropagateGlobalEstimate(
      const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
      Eigen::Matrix<double, 7, 1>* x_priori);

  void updateEstimatePropagateErrorEstimate(
      const Eigen::Matrix<double, 6, 1>& dx_old,
      const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
      Eigen::Matrix<double, 6, 1>* dx_priori);

  void updateEstimatePropagateErrorCovariance(
      Eigen::Matrix<double, 6, 6>& cov_old,
      const Eigen::Matrix<double, 7, 1>& x_old, const double dt,
      Eigen::Matrix<double, 6, 6>* covariance_priori);

  void updateEstimateUpdateErrorEstimate(
      const Eigen::Quaterniond& orientation_measured,
      const Eigen::Matrix<double, 7, 1>& x_priori,
      const Eigen::Matrix<double, 6, 1>& dx_priori,
      const Eigen::Matrix<double, 6, 6>& covariance_priori,
      Eigen::Matrix<double, 6, 1>* dx_measurement,
      Eigen::Matrix<double, 6, 6>* covariance_measurement);

  void updateEstimateRecombineErrorGlobal(
      const Eigen::Matrix<double, 7, 1> x_priori,
      Eigen::Matrix<double, 7, 1>* x_measurement,
      Eigen::Matrix<double, 6, 1>* dx_measurement);

  // Checks if the estimator has crashed.
  bool checkForEstimatorCrash();

  // Detects if the passed measurement is an outlier
  bool detectMeasurementOutlierMahalanobis(
      const Eigen::Quaterniond& orientation_measured,
      const Eigen::Matrix<double, 6, 6>& covariance);
  bool detectMeasurementOutlierSubsequent(
      const Eigen::Quaterniond& orientation_measured);
  // Returns the magnitude of the rotation represented by a quaternion
  double quaternionRotationMagnitude(const Eigen::Quaterniond& rotation);
  // Ensure covariance symmetry
  void makeCovarianceSymmetric(Eigen::Matrix<double, 6, 6>* covariance);
};

class ViconEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViconEstimator();

  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Vector3d& position_measured_W,
                      const Eigen::Quaterniond& orientation_measured_B_W,
                      const double timestamp);
  // Reset the estimator
  void reset();
  // Set estimator parameters
  void setParameters(
      const TranslationalEstimatorParameters&
          translational_estimator_parameters,
      const RotationalEstimatorParameters& rotational_estimator_parameters);
  // Get intermediate results
  void getIntermediateResults(
      TranslationalEstimatorResults* translational_estimator_results,
      RotationalEstimatorResults* rotational_estimator_results) const;

  void getEstimatorStatuses(EstimatorStatus* translational_estimator_status,
                            EstimatorStatus* rotational_estimator_status) const;

  // Functions providing access to the various estimates
  Eigen::Vector3d getEstimatedPosition() const {
    return translational_estimator_.getEstimatedPosition();
  }
  Eigen::Vector3d getEstimatedVelocity() const {
    return translational_estimator_.getEstimatedVelocity();
  }
  Eigen::Quaterniond getEstimatedOrientation() const {
    return rotational_estimator_.getEstimatedOrientation();
  }
  Eigen::Vector3d getEstimatedAngularVelocity() const {
    return rotational_estimator_.getEstimatedRate();
  }

 private:
  TranslationalEstimator translational_estimator_;
  RotationalEstimator rotational_estimator_;
  EstimatorStatus translational_estimator_status_;
  EstimatorStatus rotational_estimator_status_;
};
}

#endif  // VICON_ESTIMATOR_H
