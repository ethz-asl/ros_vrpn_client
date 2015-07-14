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

#ifndef VICON_ODOMETRY_ESTIMATOR_H
#define VICON_ODOMETRY_ESTIMATOR_H

//#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Geometry>

namespace vicon_estimation {



// The parameter class for the translational estimator and parameter default values
static const double dtTranslationalDefault = 0.01 ;
static const double kpTranslationalDefault = 1.0 ;
static const double dvTranslationalDefault = 10.0 ;
class TranslationalEstimatorParameters {

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

// Estimated object position and velocity from vicon data
class TranslationalEstimator {

  public:
    // Constructor
    TranslationalEstimator();
    // Update estimated quantities with new measurement
    void updateEstimate(const Eigen::Vector3d& pos_measured);
    // Return estimated position
    Eigen::Vector3d getEstimatedPosition() const;
    // Return estimated velocity
    Eigen::Vector3d getEstimatedVelocity() const;

    // Estimator parameters
    TranslationalEstimatorParameters estimator_parameters_;

  private:
    Eigen::Vector3d pos_hat ;
    Eigen::Vector3d vel_hat ;


};

// The parameter class for the translational estimator and parameter default values
static const double dtRotationalDefault = 0.01;
static const double dQuat_hat_initialCovarianceDefault = 1;
static const double dOmega_hat_initialCovarianceDefault = 1;
static const double dQuat_processCovarianceDefault = 1;
static const double dOmega_processCovarianceDefault = 1;
static const double quat_measurementCovarianceDefault = 1;
class RotationalEstimatorParameters {

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

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator {

  public:
    // Constructor
    RotationalEstimator();
    // Update estimated quantities with new measurement
    void updateEstimate(const Eigen::Quaterniond& quat_measured);
    // Return estimated orientation
    Eigen::Quaterniond getEstimatedOrientation() const;
    // Return estimated angular velocity
    Eigen::Vector3d getEstimatedAngularVelocity() const;

    // Estimator parameters
    RotationalEstimatorParameters estimator_parameters_;

  private:
  	// Global estimates
    Eigen::Quaterniond quat_hat;
    Eigen::Vector3d omega_hat;
    // Error estimates
    Eigen::Vector3d dQuat_hat;
    Eigen::Vector3d dOmega_hat;
    // Covariance Estimates
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> processCovariance;
    Eigen::Matrix<double, 4, 4> measurementCovariance;

    // Function to generate a skew symmetric matrix from a vector
  	Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec ) const;
    //
    void updateEstimate_propagateGlobalEstimate(Eigen::Matrix<double, 7, 1>* x_p, const Eigen::Matrix<double, 7, 1>& x_old);
    void updateEstimate_propagateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_p, const Eigen::Matrix<double, 6, 1>& dx_old, const Eigen::Matrix<double, 7, 1>& x_old);
    void updateEstimate_propagateErrorCovariance(Eigen::Matrix<double, 6, 6>* P_p, Eigen::Matrix<double, 6, 6>& P_old, const Eigen::Matrix<double, 7, 1>& x_old);
    void updateEstimate_updateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_m, Eigen::Matrix<double, 6, 6>* P_m, const Eigen::Quaterniond& quat_measured,
                                            const Eigen::Matrix<double, 7, 1>& x_p, const Eigen::Matrix<double, 6, 1>& dx_p, const Eigen::Matrix<double, 6,6>& P_p);
    void updateEstimate_recombineErrorGlobal(Eigen::Matrix<double, 7, 1>* x_m, Eigen::Matrix<double, 6, 1>* dx_m, const Eigen::Matrix<double, 7, 1> x_p);
};

class ViconOdometryEstimator{

  public:
    ViconOdometryEstimator();

    // Update estimated quantities with new measurement
    void updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured);

    // Functions providing access to the various estimates
    Eigen::Vector3d getEstimatedPosition() const;
  	Eigen::Vector3d getEstimatedVelocity() const;
  	Eigen::Quaterniond getEstimatedOrientation() const;
  	Eigen::Vector3d getEstimatedAngularVelocity() const;

    TranslationalEstimator translationalEstimator;
    RotationalEstimator rotationalEstimator;

  private:
    
};


}

#endif // VICON_ODOMETRY_ESTIMATOR_H
