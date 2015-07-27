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

namespace viconEstimator {

// The parameter class for the translational estimator and parameter default values
static const double dtTranslationalDefault = 0.01;
static const double kpTranslationalDefault = 1.0;
static const double dvTranslationalDefault = 10.0;
class TranslationalEstimatorParameters
{

 public:
  // Constructor
  TranslationalEstimatorParameters()
      : dt(dtTranslationalDefault),
        kp(kpTranslationalDefault),
        kv(dvTranslationalDefault)
  { }

  double dt;
  double kp;
  double kv;
};

class TranslationalEstimatorResults
{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  TranslationalEstimatorResults()
      : posMeasured(0.0, 0.0, 0.0),
        posOld(0.0, 0.0, 0.0),
        velOld(0.0, 0.0, 0.0),
        posEst(0.0, 0.0, 0.0),
        velEst(0.0, 0.0, 0.0)
  { }

  // Intermediate Estimator results
  Eigen::Vector3d posMeasured;
  Eigen::Vector3d posOld;
  Eigen::Vector3d velOld;
  Eigen::Vector3d posEst;
  Eigen::Vector3d velEst;

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
  void setParameters(const TranslationalEstimatorParameters& translationalEstimatorParameters);
  // Return intermediate results structure
  TranslationalEstimatorResults getResults() const { return estimatorResults_; }
  // Return estimated position
  Eigen::Vector3d getEstimatedPosition() const { return posHat_; }
  // Return estimated velocity
  Eigen::Vector3d getEstimatedVelocity() const { return velHat_; }

 private:

  // Estimator parameters
  TranslationalEstimatorParameters estimatorParameters_;
  TranslationalEstimatorResults estimatorResults_;

  // Estimates
  Eigen::Vector3d posHat_;
  Eigen::Vector3d velHat_;

};

// The parameter class for the translational estimator and parameter default values
static const double dtRotationalDefault = 0.01;
static const double dQuatHatInitialCovarianceDefault = 1;
static const double dOmegaHatInitialCovarianceDefault = 1;
static const double dQuatProcessCovarianceDefault = 0.01;
static const double dOmegaProcessCovarianceDefault = 1;
static const double quatMeasurementCovarianceDefault = 0.0005;
class RotationalEstimatorParameters
{

 public:
  // Constructor
  RotationalEstimatorParameters()
      : dt(dtRotationalDefault),
        dQuatHatInitialCovariance(dQuatHatInitialCovarianceDefault),
        dOmegaHatInitialCovariance(dOmegaHatInitialCovarianceDefault),
        dQuatProcessCovariance(dQuatProcessCovarianceDefault),
        dOmegaProcessCovariance(dOmegaProcessCovarianceDefault),
        quatMeasurementCovariance(quatMeasurementCovarianceDefault)
  { };

  double dt;
  double dQuatHatInitialCovariance;
  double dOmegaHatInitialCovariance;
  double dQuatProcessCovariance;
  double dOmegaProcessCovariance;
  double quatMeasurementCovariance;
};

class RotationalEstimatorResults
{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimatorResults()
      : quatMeasured(1.0, 0.0, 0.0, 0.0),
        quatOld(1.0, 0.0, 0.0, 0.0),
        omegaOld(0.0, 0.0, 0.0),
        quatEst(1.0, 0.0, 0.0, 0.0),
        omegaEst(0.0, 0.0, 0.0)
  { };

  // Intermediate Estimator results
  Eigen::Quaterniond quatMeasured;
  Eigen::Quaterniond quatOld;
  Eigen::Vector3d omegaOld;
  Eigen::Quaterniond quatEst;
  Eigen::Vector3d omegaEst;

};

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator
{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  RotationalEstimator();  //ros::NodeHandle& nh
  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Quaterniond& quatMeasured);
  // Reset the estimator
  void reset();
  // Setting the estimator parameters
  void setParameters(const RotationalEstimatorParameters& rotationalEstimatorParameters);
  // Return intermediate results structure
  RotationalEstimatorResults getResults() const { return estimatorResults_; }
  // Return estimated orientation
  const Eigen::Quaterniond getEstimatedOrientation() const { return quatHat_; }
  // Return estimated angular velocity
  Eigen::Vector3d getEstimatedAngularVelocity() const { return omegaHat_; }

 private:

  // Estimator parameters
  RotationalEstimatorParameters estimatorParameters_;
  // Estimator (intermediate) results
  RotationalEstimatorResults estimatorResults_;

  // Global estimates
  Eigen::Quaterniond quatHat_;
  Eigen::Vector3d omegaHat_;
  // Error estimates
  Eigen::Vector3d dQuatHat_;
  Eigen::Vector3d dOmegaHat_;
  // Covariance Estimates
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> processCovariance_;
  Eigen::Matrix<double, 4, 4> measurementCovariance_;

  // Function to generate a skew symmetric matrix from a vector
  Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec) const;
  //
  void updateEstimatePropagateGlobalEstimate( const Eigen::Matrix<double, 7, 1>& xOld,
                                              Eigen::Matrix<double, 7, 1>* xPriori);

  void updateEstimatePropagateErrorEstimate( const Eigen::Matrix<double, 6, 1>& dxOld,
                                             const Eigen::Matrix<double, 7, 1>& xOld,
                                             Eigen::Matrix<double, 6, 1>* dxPriori);

  void updateEstimatePropagateErrorCovariance( Eigen::Matrix<double, 6, 6>& covOld,
                                               const Eigen::Matrix<double, 7, 1>& xOld,
                                               Eigen::Matrix<double, 6, 6>* covPriori);

  void updateEstimateUpdateErrorEstimate( const Eigen::Quaterniond& quatMeasured,
                                          const Eigen::Matrix<double, 7, 1>& xPriori,
                                          const Eigen::Matrix<double, 6, 1>& dxPriori,
                                          const Eigen::Matrix<double, 6, 6>& covPriori,
                                          Eigen::Matrix<double, 6, 1>* dxMeasurement,
                                          Eigen::Matrix<double, 6, 6>* covMeasurement);

  void updateEstimateRecombineErrorGlobal(  const Eigen::Matrix<double, 7, 1> xPriori,
                                            Eigen::Matrix<double, 7, 1>* xMeasurement,
                                            Eigen::Matrix<double, 6, 1>* dxMeasurement);
};

class ViconEstimator
{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViconEstimator();  //ros::NodeHandle& nh

  // Update estimated quantities with new measurement
  void updateEstimate(const Eigen::Vector3d& posMeasured, const Eigen::Quaterniond& quatMeasured);
  // Reset the estimator
  void reset();
  // Set estimator parameters
  void setParameters(const TranslationalEstimatorParameters& translationalEstimatorParameters,
                     const RotationalEstimatorParameters& rotationalEstimatorParameters);
  // Get intermediate results
  void getIntermediateResults ( TranslationalEstimatorResults* translationalEstimatorResults,
                                RotationalEstimatorResults* rotationalEstimatorResults) const ;

  // Functions providing access to the various estimates
  Eigen::Vector3d getEstimatedPosition() const { return translationalEstimator_.getEstimatedPosition(); }
  Eigen::Vector3d getEstimatedVelocity() const { return translationalEstimator_.getEstimatedVelocity(); }
  Eigen::Quaterniond getEstimatedOrientation() const { return rotationalEstimator_.getEstimatedOrientation(); }
  Eigen::Vector3d getEstimatedAngularVelocity() const { return rotationalEstimator_.getEstimatedAngularVelocity(); }

 private:

  TranslationalEstimator translationalEstimator_;
  RotationalEstimator rotationalEstimator_;

};

}

#endif // VICON_ESTIMATOR_H
