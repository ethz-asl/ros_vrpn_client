/*
 * Copyright 2015 Alexander Millane, ASL, ETH Zurich, Switzerland
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
static const double dtTranslationalDefault = 0.01 ;
static const double kpTranslationalDefault = 1.0 ;
static const double dvTranslationalDefault = 10.0 ;
class TranslationalEstimatorParameters
{

  public:
  	// Constructor
    TranslationalEstimatorParameters() :
      dt(dtTranslationalDefault),
      kp(kpTranslationalDefault),
      kv(dvTranslationalDefault)
    { };

	  double dt;
	  double kp;
	  double kv;
};

class TranslationalEstimatorResults
{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Constructor
    TranslationalEstimatorResults() :
      pos_measured(0.0, 0.0, 0.0),
      pos_old(0.0, 0.0, 0.0),
      vel_old(0.0, 0.0, 0.0),
      pos_est(0.0, 0.0, 0.0),
      vel_est(0.0, 0.0, 0.0)
    { };

    // Intermediate Estimator results
    Eigen::Vector3d pos_measured;
    Eigen::Vector3d pos_old;
    Eigen::Vector3d vel_old;
    Eigen::Vector3d pos_est;
    Eigen::Vector3d vel_est;

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
    TranslationalEstimatorResults getResults() const { return estimator_results_; };
    // Return estimated position
    Eigen::Vector3d getEstimatedPosition() const { return pos_hat_; };
    // Return estimated velocity
    Eigen::Vector3d getEstimatedVelocity() const { return vel_hat_; };


  private:

    // Estimator parameters
    TranslationalEstimatorParameters estimator_parameters_;
    TranslationalEstimatorResults estimator_results_;

    // Estimates
    Eigen::Vector3d pos_hat_ ;
    Eigen::Vector3d vel_hat_ ;


};

// The parameter class for the translational estimator and parameter default values
static const double dtRotationalDefault = 0.01;
static const double dQuat_hat_initialCovarianceDefault = 1;
static const double dOmega_hat_initialCovarianceDefault = 1;
static const double dQuat_processCovarianceDefault = 0.01;
static const double dOmega_processCovarianceDefault = 1;
static const double quat_measurementCovarianceDefault = 0.0005;
class RotationalEstimatorParameters
{

  public:
  	// Constructor
    RotationalEstimatorParameters() :
      dt(dtRotationalDefault),
      dQuat_hat_initialCovariance(dQuat_hat_initialCovarianceDefault),
	  	dOmega_hat_initialCovariance(dOmega_hat_initialCovarianceDefault),
	  	dQuat_processCovariance(dQuat_processCovarianceDefault),
	  	dOmega_processCovariance(dOmega_processCovarianceDefault),
	  	quat_measurementCovariance(quat_measurementCovarianceDefault)
    { };

	  double dt;
	  double dQuat_hat_initialCovariance;
	  double dOmega_hat_initialCovariance;
	  double dQuat_processCovariance;
	  double dOmega_processCovariance;
	  double quat_measurementCovariance;
};

class RotationalEstimatorResults
{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Constructor
    RotationalEstimatorResults() :
      quat_measured(1.0, 0.0, 0.0, 0.0),
      quat_old(1.0, 0.0, 0.0, 0.0),
      omega_old(0.0, 0.0, 0.0),
      quat_est(1.0, 0.0, 0.0, 0.0),
      omega_est(0.0, 0.0, 0.0)
    { };

    // Intermediate Estimator results
    Eigen::Quaterniond quat_measured;
    Eigen::Quaterniond quat_old;
    Eigen::Vector3d omega_old;
    Eigen::Quaterniond quat_est;
    Eigen::Vector3d omega_est;

};

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator
{

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Constructor
    RotationalEstimator(); //ros::NodeHandle& nh
    // Update estimated quantities with new measurement
    void updateEstimate(const Eigen::Quaterniond& quat_measured);
    // Reset the estimator
    void reset();
    // Setting the estimator parameters
    void setParameters(const RotationalEstimatorParameters& rotationalEstimatorParameters);
    // Return intermediate results structure
    RotationalEstimatorResults getResults() const { return estimator_results_; };
    // Return estimated orientation
    const Eigen::Quaterniond getEstimatedOrientation() const { return quat_hat_ ; };
    // Return estimated angular velocity
    Eigen::Vector3d getEstimatedAngularVelocity() const { return omega_hat_ ; };

  private:

    // Estimator parameters
    RotationalEstimatorParameters estimator_parameters_;
    // Estimator (intermediate) results
    RotationalEstimatorResults estimator_results_ ;

  	// Global estimates
    Eigen::Quaterniond quat_hat_;
    Eigen::Vector3d omega_hat_;
    // Error estimates
    Eigen::Vector3d dQuat_hat_;
    Eigen::Vector3d dOmega_hat_;
    // Covariance Estimates
    Eigen::Matrix<double, 6, 6> covariance_;
    Eigen::Matrix<double, 6, 6> processCovariance_;
    Eigen::Matrix<double, 4, 4> measurementCovariance_;

    // Function to generate a skew symmetric matrix from a vector
  	Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec ) const;
    //
    void updateEstimate_propagateGlobalEstimate(Eigen::Matrix<double, 7, 1>* x_p, const Eigen::Matrix<double, 7, 1>& x_old ); //ros_vrpn_client::rotationalEstimator* msg

    void updateEstimate_propagateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_p, const Eigen::Matrix<double, 6, 1>& dx_old, const Eigen::Matrix<double, 7, 1>& x_old); //ros_vrpn_client::rotationalEstimator* msg

    void updateEstimate_propagateErrorCovariance(Eigen::Matrix<double, 6, 6>* P_p, Eigen::Matrix<double, 6, 6>& P_old, const Eigen::Matrix<double, 7, 1>& x_old); //ros_vrpn_client::rotationalEstimator* msg

    void updateEstimate_updateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_m, Eigen::Matrix<double, 6, 6>* P_m, const Eigen::Quaterniond& quat_measured,
                                            const Eigen::Matrix<double, 7, 1>& x_p, const Eigen::Matrix<double, 6, 1>& dx_p, const Eigen::Matrix<double, 6,6>& P_p); //ros_vrpn_client::rotationalEstimator* msg

    void updateEstimate_recombineErrorGlobal(Eigen::Matrix<double, 7, 1>* x_m, Eigen::Matrix<double, 6, 1>* dx_m, const Eigen::Matrix<double, 7, 1> x_p); // ros_vrpn_client::rotationalEstimator* msg
};

class ViconEstimator
{

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ViconEstimator(); //ros::NodeHandle& nh

    // Update estimated quantities with new measurement
    void updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured);
    // Reset the estimator
    void reset();
    // Set estimator parameters
    void setParameters(const TranslationalEstimatorParameters& translationalEstimatorParameters, const RotationalEstimatorParameters& rotationalEstimatorParameters);
    // Get intermediate results
    void getIntermediateResults(TranslationalEstimatorResults* translationalEstimatorResults, RotationalEstimatorResults* rotationalEstimatorResults);

    // Functions providing access to the various estimates
    Eigen::Vector3d getEstimatedPosition() const { return translationalEstimator_.getEstimatedPosition(); };
  	Eigen::Vector3d getEstimatedVelocity() const { return translationalEstimator_.getEstimatedVelocity(); };
  	Eigen::Quaterniond getEstimatedOrientation() const { return rotationalEstimator_.getEstimatedOrientation(); };
  	Eigen::Vector3d getEstimatedAngularVelocity() const { return rotationalEstimator_.getEstimatedAngularVelocity(); };

  private:

    TranslationalEstimator translationalEstimator_;
    RotationalEstimator rotationalEstimator_;
    
};


}

#endif // VICON_ESTIMATOR_H
