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

#include "vicon_estimator.h"

namespace viconEstimator {

/*
 * --------------------------------------------------------------------
 * Vicon Odometry Estimator
 * --------------------------------------------------------------------
 */

ViconEstimator::ViconEstimator()
    : translationalEstimator_(),
      rotationalEstimator_()
{

}

void ViconEstimator::updateEstimate(const Eigen::Vector3d& posMeasured,
                                    const Eigen::Quaterniond& quatMeasured)
{
  // Updating the translational and rotation sub-estimates
  translationalEstimator_.updateEstimate(posMeasured);
  rotationalEstimator_.updateEstimate(quatMeasured);
}

void ViconEstimator::reset()
{
  // Resetting the translational and rotation
  translationalEstimator_.reset();
  rotationalEstimator_.reset();
}

void ViconEstimator::setParameters(
    const TranslationalEstimatorParameters& translationalEstimatorParameters,
    const RotationalEstimatorParameters& rotationalEstimatorParameters)
{
  translationalEstimator_.setParameters(translationalEstimatorParameters);
  rotationalEstimator_.setParameters(rotationalEstimatorParameters);
}

void ViconEstimator::getIntermediateResults (
    TranslationalEstimatorResults* translationalEstimatorResults,
    RotationalEstimatorResults* rotationalEstimatorResults) const
{
  *translationalEstimatorResults = translationalEstimator_.getResults();
  *rotationalEstimatorResults = rotationalEstimator_.getResults();
}

/*
 * --------------------------------------------------------------------
 * Translational Estimator
 * --------------------------------------------------------------------
 */

TranslationalEstimator::TranslationalEstimator()
    ://ros::NodeHandle& nh
      posHat_(0.0, 0.0, 0.0),
      velHat_(0.0, 0.0, 0.0),
      estimatorParameters_(),
      estimatorResults_()
{

}

void TranslationalEstimator::updateEstimate(const Eigen::Vector3d& posMeasured)
{
  // Saving the measurement to the intermediate results
  estimatorResults_.posMeasured = posMeasured;
  // Saving the old state to the intermediate results
  estimatorResults_.posOld = posHat_;
  estimatorResults_.velOld = velHat_;
  // Constructing the full state
  Eigen::Matrix<double, 6, 1> xHat;
  xHat << posHat_, velHat_;
  // Constructing the system matrix
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(0, 3) = estimatorParameters_.dt * Eigen::Matrix3d::Identity();
  // Constructing the measurement matrix
  Eigen::Matrix<double, 3, 6> C;
  C.setZero();
  C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Constructing the Luenberger gain matrix
  Eigen::Matrix<double, 6, 3> L_gain;
  L_gain << estimatorParameters_.kp * Eigen::Matrix3d::Identity(), estimatorParameters_.kv
      * Eigen::Matrix3d::Identity();
  // Correction using the Luenberger equations + gain
  xHat = (A - L_gain * C * A) * xHat + L_gain * posMeasured;
  // Extracting state components
  posHat_ = xHat.block<3, 1>(0, 0);
  velHat_ = xHat.block<3, 1>(3, 0);
  // Saving estimate to intermediate results
  estimatorResults_.posEst = posHat_;
  estimatorResults_.velEst = velHat_;
}

void TranslationalEstimator::reset()
{
  posHat_ = Eigen::Vector3d::Zero();
  velHat_ = Eigen::Vector3d::Zero();
}

void TranslationalEstimator::setParameters(
    const TranslationalEstimatorParameters& translationalEstimatorParameters)
{
  estimatorParameters_.dt = translationalEstimatorParameters.dt;
  estimatorParameters_.kp = translationalEstimatorParameters.kp;
  estimatorParameters_.kv = translationalEstimatorParameters.kv;
}

/*
 * --------------------------------------------------------------------
 * Rotational Estimator
 * --------------------------------------------------------------------
 */

RotationalEstimator::RotationalEstimator()
    ://ros::NodeHandle& nh
      quatHat_(1.0, 0.0, 0.0, 0.0),
      omegaHat_(0.0, 0.0, 0.0),
      dQuatHat_(0.0, 0.0, 0.0),
      dOmegaHat_(0.0, 0.0, 0.0),
      estimatorParameters_(),
      estimatorResults_()
{
  // Initializing the initial covariance
  covariance_ <<  estimatorParameters_.dQuatHatInitialCovariance * Eigen::Matrix3d::Identity(),
                  Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), estimatorParameters_.dOmegaHatInitialCovariance * Eigen::Matrix3d::Identity();
  // Constructing process and measurement covariance matrices
  processCovariance_ << estimatorParameters_.dQuatProcessCovariance * Eigen::Matrix3d::Identity(),
                        Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), estimatorParameters_.dOmegaProcessCovariance * Eigen::Matrix3d::Identity();
  measurementCovariance_ << estimatorParameters_.quatMeasurementCovariance * Eigen::Matrix4d::Identity();
}

void RotationalEstimator::reset()
{
  quatHat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  omegaHat_ = Eigen::Matrix<double, 3, 1>::Zero();
  dQuatHat_ = Eigen::Matrix<double, 3, 1>::Zero();
  dOmegaHat_ = Eigen::Matrix<double, 3, 1>::Zero();
  // Reseting to initial covariance
  covariance_ <<  estimatorParameters_.dQuatHatInitialCovariance * Eigen::Matrix3d::Identity(),
                  Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), estimatorParameters_.dOmegaHatInitialCovariance * Eigen::Matrix3d::Identity();
  // Constructing process and measurement covariance matrices
  processCovariance_ << estimatorParameters_.dQuatProcessCovariance * Eigen::Matrix3d::Identity(),
                        Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), estimatorParameters_.dOmegaProcessCovariance * Eigen::Matrix3d::Identity();
  measurementCovariance_ << estimatorParameters_.quatMeasurementCovariance * Eigen::Matrix4d::Identity();
}

void RotationalEstimator::setParameters(const RotationalEstimatorParameters& rotationalEstimatorParameters)
{
  estimatorParameters_.dt = rotationalEstimatorParameters.dt;
  estimatorParameters_.dQuatHatInitialCovariance = rotationalEstimatorParameters.dQuatHatInitialCovariance;
  estimatorParameters_.dOmegaHatInitialCovariance = rotationalEstimatorParameters.dOmegaHatInitialCovariance;
  estimatorParameters_.dQuatProcessCovariance = rotationalEstimatorParameters.dQuatProcessCovariance;
  estimatorParameters_.dOmegaProcessCovariance = rotationalEstimatorParameters.dOmegaProcessCovariance;
  estimatorParameters_.quatMeasurementCovariance = rotationalEstimatorParameters.quatMeasurementCovariance;
}

void RotationalEstimator::updateEstimate(const Eigen::Quaterniond& quatMeasured)
{
  // Writing the raw measurement to the intermediate results structure
  estimatorResults_.quatMeasured = quatMeasured;

  // Writing the old estimate to the intermediate results structure
  estimatorResults_.quatOld = quatHat_;
  estimatorResults_.omegaOld = omegaHat_;

  // Propagating the global state estimate
  Eigen::Matrix<double, 7, 1> x_old;
  Eigen::Matrix<double, 7, 1> x_p;
  x_old << quatHat_.coeffs(), omegaHat_;
  updateEstimatePropagateGlobalEstimate(x_old, &x_p);

  // Propagating the error state estimate
  Eigen::Matrix<double, 6, 1> dx_old;
  Eigen::Matrix<double, 6, 1> dx_p;
  dx_old << dQuatHat_, dOmegaHat_;
  updateEstimatePropagateErrorEstimate(dx_old, x_old, &dx_p);

  // Propagating the estimate covariance
  Eigen::Matrix<double, 6, 6> P_old;
  Eigen::Matrix<double, 6, 6> P_p;
  P_old = covariance_;
  updateEstimatePropagateErrorCovariance(P_old, x_old, &P_p);

  // Measurement Update
  Eigen::Matrix<double, 6, 1> dx_m;
  Eigen::Matrix<double, 6, 6> P_m;
  updateEstimateUpdateErrorEstimate(quatMeasured, x_p, dx_p, P_p, &dx_m, &P_m);

  // Global state correction
  Eigen::Matrix<double, 7, 1> x_m;
  updateEstimateRecombineErrorGlobal(x_p, &x_m, &dx_m);

  // Extracting estimated quantities from the posteriori state
  quatHat_ = Eigen::Quaterniond(x_m.block<4, 1>(0, 0));
  omegaHat_ = x_m.block<3, 1>(4, 0);
  covariance_ = P_m;

  // Writing the old estimate to the intermediate results structure
  estimatorResults_.quatEst = quatHat_;
  estimatorResults_.omegaEst = omegaHat_;

}

Eigen::Matrix3d RotationalEstimator::skewMatrix(const Eigen::Vector3d& vec) const
{
  Eigen::Matrix3d vecCross;
  vecCross << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return vecCross;
}

void RotationalEstimator::updateEstimatePropagateGlobalEstimate(
    const Eigen::Matrix<double, 7, 1>& xOld,
    Eigen::Matrix<double, 7, 1>* x_p)
{
  // Extracting components of the state
  Eigen::Quaterniond quatHatOld = Eigen::Quaterniond(xOld.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> omegaHatOld = xOld.block<3, 1>(4, 0);
  // Performing propagation
  Eigen::Quaterniond omegaHatQuat = Eigen::Quaterniond( 0, omegaHatOld.x(), omegaHatOld.y(), omegaHatOld.z());
  Eigen::Quaterniond quatHatPrioriRoc = Eigen::Quaterniond( 0.5 * (quatHatOld * omegaHatQuat).coeffs());  // Using hamilton definition
  Eigen::Vector3d omegaHatPrioriRoc = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quatHatPriori = Eigen::Quaterniond( quatHatOld.coeffs() + quatHatPrioriRoc.coeffs() * estimatorParameters_.dt);
  Eigen::Vector3d omegaHatPriori = omegaHatOld + omegaHatPrioriRoc * estimatorParameters_.dt;
  // Writing to apriori state
  *x_p << quatHatPriori.coeffs(), omegaHatPriori;
}

void RotationalEstimator::updateEstimatePropagateErrorEstimate(
    const Eigen::Matrix<double, 6, 1>& dxOld,
    const Eigen::Matrix<double, 7, 1>& xOld,
    Eigen::Matrix<double, 6, 1>* dx_p)
{
  // Extracting components of the states
  Eigen::Quaterniond quatHat = Eigen::Quaterniond(xOld.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> omegaHat = xOld.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dQuatHat = dxOld.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> dOmegaHat = dxOld.block<3, 1>(3, 0);
  // Performing propagation
  Eigen::Vector3d dQuatHatPrioriRoc = -omegaHat.cross(dQuatHat) + 0.5 * dOmegaHat;  // Appears to agree with equations in papers
  Eigen::Vector3d dOmegaHatPrioriRoc = Eigen::Vector3d::Zero();
  Eigen::Vector3d dQuathatPriori = dQuatHat + dQuatHatPrioriRoc * estimatorParameters_.dt;
  Eigen::Vector3d dOmegaHatPriori = dOmegaHat + dOmegaHatPrioriRoc * estimatorParameters_.dt;
  // Writing to apriori error state
  *dx_p << dQuathatPriori, dOmegaHatPriori;
}

void RotationalEstimator::updateEstimatePropagateErrorCovariance(
    Eigen::Matrix<double, 6, 6>& covOld,
    const Eigen::Matrix<double, 7, 1>& xOld,
    Eigen::Matrix<double, 6, 6>* P_p)
{
  // Extracting components of the state
  Eigen::Quaterniond quatHat = Eigen::Quaterniond(xOld.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> omegaHat = xOld.block<3, 1>(4, 0);
  // Constructing linearized system matrices
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 6> L;
  A << -1 * skewMatrix(omegaHat), 0.5 * Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();  // Should be accordance with the hamilton definition
  L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  // Performing propagation
  *P_p = covOld + (A * covOld + covOld * A.transpose() + L * processCovariance_ * (L.transpose())) * estimatorParameters_.dt;
}

void RotationalEstimator::updateEstimateUpdateErrorEstimate(
    const Eigen::Quaterniond& quatMeasured,
    const Eigen::Matrix<double, 7, 1>& xPriori,
    const Eigen::Matrix<double, 6, 1>& dxPriori,
    const Eigen::Matrix<double, 6, 6>& covPriori,
    Eigen::Matrix<double, 6, 1>* dx_m,
    Eigen::Matrix<double, 6, 6>* covMeasurement)
{
  // Extracting components of the state
  Eigen::Quaterniond quatHatPriori = Eigen::Quaterniond(xPriori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> omegaHatPriori = xPriori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dQuatHatPriori = dxPriori.block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> dOmegaHatPriori = dxPriori.block<3, 1>(3, 0);
  // Constructing linearized measurement matrix
  Eigen::Matrix<double, 4, 3> Hdq;
  Eigen::Matrix<double, 4, 6> H;
  Eigen::Vector3d quatHatPrioriVec = quatHatPriori.vec();
  Hdq << quatHatPriori.w() * Eigen::Matrix<double, 3, 3>::Identity() + skewMatrix(quatHatPrioriVec), -quatHatPriori.vec().transpose();
  H << Hdq, Eigen::Matrix<double, 4, 3>::Zero();
  // Predicting the measurement
  Eigen::Quaterniond quatPredicted = Eigen::Quaterniond( Hdq * dQuatHatPriori + quatHatPriori.coeffs());
  // Computing the Kalman gain
  Eigen::Matrix<double, 4, 4> S = H * covPriori * H.transpose() + measurementCovariance_;
  Eigen::Matrix<double, 6, 4> K;
  K = covPriori * H.transpose() * S.inverse();
  // Correcting the state
  *dx_m = dxPriori + K * (quatMeasured.coeffs() - quatPredicted.coeffs());
  // Updating the covariance
  *covMeasurement = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covPriori;
  //*P_m = (Eigen::Matrix<double, 6, 6>::Identity() - K*H)*P_p*((Eigen::Matrix<double, 6, 6>::Identity() - K*H).transpose()) + K*measurementCovariance*K.transpose();
}

void RotationalEstimator::updateEstimateRecombineErrorGlobal( const Eigen::Matrix<double, 7, 1> xPriori,
                                                              Eigen::Matrix<double, 7, 1>* xMeasurement,
                                                              Eigen::Matrix<double, 6, 1>* dxMeasurement)
{
  // Extracting components of the state
  Eigen::Quaterniond quatHatPriori = Eigen::Quaterniond(xPriori.block<4, 1>(0, 0));
  Eigen::Matrix<double, 3, 1> omegaHatPriori = xPriori.block<3, 1>(4, 0);
  Eigen::Matrix<double, 3, 1> dQuatHatMeasurement = dxMeasurement->block<3, 1>(0, 0);
  Eigen::Matrix<double, 3, 1> dOmegaHatMeasurement = dxMeasurement->block<3, 1>(3, 0);
  // Completing the error quaternion
  Eigen::Quaterniond dQuatHatMeasurementQuat = Eigen::Quaterniond(1.0, dQuatHatMeasurement.x(), dQuatHatMeasurement.y(), dQuatHatMeasurement.z());
  // Correction
  Eigen::Quaterniond quatHatMeasurement = quatHatPriori * dQuatHatMeasurementQuat;
  Eigen::Matrix<double, 3, 1> omegaHatMeasurement = omegaHatPriori + dOmegaHatMeasurement;
  // Normalizing the posteriori quaternion
  quatHatMeasurement.normalize();
  // Writing to posteriori global state and error states
  *xMeasurement << quatHatMeasurement.coeffs(), omegaHatMeasurement;
  *dxMeasurement = Eigen::Matrix<double, 6, 1>::Zero();
}

}
