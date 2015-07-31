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

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <Eigen/Geometry>

namespace vicon_estimator {

// The parameter class for the translational estimator and parameter default values
static const double kDefaultTranslationalDt = 0.01;
static const double kDefaultTranslationalKp = 1.0;
static const double kDefaultTranslationalKv = 10.0;
static const Eigen::Vector3d kDefaultInitialPositionEstimate = Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialVelocityEstimate = Eigen::Vector3d::Zero();

class TranslationalEstimatorParameters
{

 public:
  // Constructor
  TranslationalEstimatorParameters()
      : dt_(kDefaultTranslationalDt),
        kp_(kDefaultTranslationalKp),
        kv_(kDefaultTranslationalKv),
        initial_position_estimate_(kDefaultInitialPositionEstimate),
        initial_velocity_estimate_(kDefaultInitialVelocityEstimate)
  { }

  double dt_;
  double kp_;
  double kv_;
  Eigen::Vector3d initial_position_estimate_;
  Eigen::Vector3d initial_velocity_estimate_;
};

class TranslationalEstimatorResults
{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimatorResults()
      : position_measured_(Eigen::Vector3d::Zero()),
        position_old_(Eigen::Vector3d::Zero()),
        velocity_old_(Eigen::Vector3d::Zero()),
        position_estimate_(Eigen::Vector3d::Zero()),
        velocity_estimate_(Eigen::Vector3d::Zero())
  { }

  // Intermediate Estimator results
  Eigen::Vector3d position_measured_;
  Eigen::Vector3d position_old_;
  Eigen::Vector3d velocity_old_;
  Eigen::Vector3d position_estimate_;
  Eigen::Vector3d velocity_estimate_;

};

// Estimated object position and velocity from vicon data
class TranslationalEstimator
{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimator();
  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Vector3d& pos_measured);
  // Reset the estimator
  void reset();
  // Setting the estimator parameters
  void setParameters(const TranslationalEstimatorParameters& estimator_parameters);
  // Return intermediate results structure
  TranslationalEstimatorResults getResults() const { return estimator_results_; }
  // Return estimated position
  Eigen::Vector3d getEstimatedPosition() const { return position_estimate_; }
  // Return estimated velocity
  Eigen::Vector3d getEstimatedVelocity() const { return velocity_estimate_; }

 private:

  // Estimator parameters
  TranslationalEstimatorParameters estimator_parameters_;
  TranslationalEstimatorResults estimator_results_;

  // Estimates
  Eigen::Vector3d position_estimate_;
  Eigen::Vector3d velocity_estimate_;

};

// The parameter class for the translational estimator and parameter default values
static const double kDefaultRotationalDt = 0.01;
static const double kDefaultdOrientationEstimateInitialCovariance = 1;
static const double kDefaultdRollrateEstimateInitialCovariance = 1;
static const double kDefaultdOrientationProcessCovariance = 0.01;
static const double kDefaultdRollrateProcessCovariance = 1;
static const double kDefaultOrientationMeasurementCovariance = 0.0005;
static const Eigen::Quaterniond kDefaultInitialOrientationEstimate = Eigen::Quaterniond::Identity();
static const Eigen::Vector3d kDefaultInitialRollrateEstimate = Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialDorientationEstimate = Eigen::Vector3d::Zero();
static const Eigen::Vector3d kDefaultInitialDrollrateEstimate = Eigen::Vector3d::Zero();

class RotationalEstimatorParameters
{

 public:
  // Constructor
  RotationalEstimatorParameters()
      : dt_(kDefaultRotationalDt),
        dorientation_estimate_initial_covariance_(kDefaultdOrientationEstimateInitialCovariance),
        drollrate_estimate_initial_covariance_(kDefaultdRollrateEstimateInitialCovariance),
        dorientation_process_covariance_(kDefaultdOrientationProcessCovariance),
        drollrate_process_covariance_(kDefaultdRollrateProcessCovariance),
        orientation_measurement_covariance_(kDefaultOrientationMeasurementCovariance),
        initial_orientation_estimate_(kDefaultInitialOrientationEstimate),
        initial_rollrate_estimate_(kDefaultInitialRollrateEstimate),
        initial_dorientation_estimate_(kDefaultInitialDorientationEstimate),
        initial_drollrate_estimate_(kDefaultInitialDrollrateEstimate)
  { };

  double dt_;
  double dorientation_estimate_initial_covariance_;
  double drollrate_estimate_initial_covariance_;
  double dorientation_process_covariance_;
  double drollrate_process_covariance_;
  double orientation_measurement_covariance_;
  Eigen::Quaterniond initial_orientation_estimate_;
  Eigen::Vector3d initial_rollrate_estimate_;
  Eigen::Vector3d initial_dorientation_estimate_;
  Eigen::Vector3d initial_drollrate_estimate_;
};

class RotationalEstimatorResults
{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimatorResults()
      : orientation_measured_(Eigen::Quaterniond::Identity()),
        orientation_old_(Eigen::Quaterniond::Identity()),
        rollrate_old_(Eigen::Vector3d::Zero()),
        orientation_estimate_(Eigen::Quaterniond::Identity()),
        rollrate_estimate_(Eigen::Vector3d::Zero())
  { };

  // Intermediate Estimator results
  Eigen::Quaterniond orientation_measured_; //TODO(millanea): refactoring up to here.
  Eigen::Quaterniond orientation_old_;
  Eigen::Vector3d rollrate_old_;
  Eigen::Quaterniond orientation_estimate_;
  Eigen::Vector3d rollrate_estimate_;

};

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator
{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimator();
  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Quaterniond& orientation_measured);
  // Reset the estimator
  void reset();
  // Setting the estimator parameters
  void setParameters(const RotationalEstimatorParameters& estimator_parameters);
  // Return intermediate results structure
  RotationalEstimatorResults getResults() const { return estimator_results_; }
  // Return estimated orientation
  const Eigen::Quaterniond getEstimatedOrientation() const { return orientation_estimate_; }
  // Return estimated angular velocity
  Eigen::Vector3d getEstimatedRollrate() const { return rollrate_estimate_; }

 private:

  // Estimator parameters
  RotationalEstimatorParameters estimator_parameters_;
  // Estimator (intermediate) results
  RotationalEstimatorResults estimator_results_;

  // Global estimates
  Eigen::Quaterniond orientation_estimate_;
  Eigen::Vector3d rollrate_estimate_;
  // Error estimates
  Eigen::Vector3d dorientation_estimate_;
  Eigen::Vector3d drollrate_estimate_;
  // Covariance Estimates
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_covariance_;
  Eigen::Matrix<double, 4, 4> measurement_covariance_;

  // Function to generate a skew symmetric matrix from a vector
  Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec) const;
  // Serial of functions performing the estimate update steps
  void updateEstimatePropagateGlobalEstimate( const Eigen::Matrix<double, 7, 1>& x_old,
                                              Eigen::Matrix<double, 7, 1>* x_priori);

  void updateEstimatePropagateErrorEstimate( const Eigen::Matrix<double, 6, 1>& dx_old,
                                             const Eigen::Matrix<double, 7, 1>& x_old,
                                             Eigen::Matrix<double, 6, 1>* dx_priori);

  void updateEstimatePropagateErrorCovariance( Eigen::Matrix<double, 6, 6>& cov_old,
                                               const Eigen::Matrix<double, 7, 1>& x_old,
                                               Eigen::Matrix<double, 6, 6>* covariance_priori);

  void updateEstimateUpdateErrorEstimate( const Eigen::Quaterniond& orientation_measured,
                                          const Eigen::Matrix<double, 7, 1>& x_priori,
                                          const Eigen::Matrix<double, 6, 1>& dx_priori,
                                          const Eigen::Matrix<double, 6, 6>& covariance_priori,
                                          Eigen::Matrix<double, 6, 1>* dx_measurement,
                                          Eigen::Matrix<double, 6, 6>* covariance_measurement);

  void updateEstimateRecombineErrorGlobal(  const Eigen::Matrix<double, 7, 1> x_priori,
                                            Eigen::Matrix<double, 7, 1>* x_measurement,
                                            Eigen::Matrix<double, 6, 1>* dx_measurement);
};

class ViconEstimator
{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViconEstimator();

  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Vector3d& position_measured, const Eigen::Quaterniond& orientation_measured);
  // Reset the estimator
  void reset();
  // Set estimator parameters
  void setParameters(const TranslationalEstimatorParameters& translational_estimator_parameters,
                     const RotationalEstimatorParameters& rotational_estimator_parameters);
  // Get intermediate results
  void getIntermediateResults ( TranslationalEstimatorResults* translational_estimator_results,
                                RotationalEstimatorResults* rotational_estimator_results) const ;

  // Functions providing access to the various estimates
  Eigen::Vector3d getEstimatedPosition() const { return translational_estimator_.getEstimatedPosition(); }
  Eigen::Vector3d getEstimatedVelocity() const { return translational_estimator_.getEstimatedVelocity(); }
  Eigen::Quaterniond getEstimatedOrientation() const { return rotational_estimator_.getEstimatedOrientation(); }
  Eigen::Vector3d getEstimatedAngularVelocity() const { return rotational_estimator_.getEstimatedRollrate(); }

 private:

  TranslationalEstimator translational_estimator_;
  RotationalEstimator rotational_estimator_;

};

}

#endif // VICON_ESTIMATOR_H
