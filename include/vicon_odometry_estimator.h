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
    void updateEstimate(Eigen::Vector3d pos_measured);
    // Return estimated position
    Eigen::Vector3d getEstimatedPosition();
    // Return estimated velocity
    Eigen::Vector3d getEstimatedVelocity();

    // Estimator parameters
    TranslationalEstimatorParameters estimator_parameters_;

  private:
    Eigen::Vector3d pos_hat ;
    Eigen::Vector3d vel_hat ;


};

// The parameter class for the translational estimator and parameter default values
static const double dtRotationalDefault = 0.01 ;
class RotationalEstimatorParameters {

  public:
  	// Constructor
    RotationalEstimatorParameters() :
      dt(dtRotationalDefault)
    { };

	  double dt;
};

// Estimated object orientation and roll rates from vicon data
class RotationalEstimator {

  public:
    // Constructor
    RotationalEstimator();
    // Update estimated quantities with new measurement
    void updateEstimate(Eigen::Quaterniond quat_measured);
    // Return estimated orientation
    Eigen::Quaterniond getEstimatedOrientation();
    // Return estimated angular velocity
    Eigen::Vector3d getEstimatedAngularVelocity();

    // Estimator parameters
    RotationalEstimatorParameters estimator_parameters_;

  private:
    Eigen::Quaterniond quat_hat ;
    Eigen::Vector3d omega_hat ;

};

class ViconOdometryEstimator{

  public:
    ViconOdometryEstimator();

    // Update estimated quantities with new measurement
    void updateEstimate(Eigen::Vector3d pos_measured, Eigen::Quaterniond quat_measured);

    // Functions providing access to the various estimates
    Eigen::Vector3d getEstimatedPosition();
  	Eigen::Vector3d getEstimatedVelocity();
  	Eigen::Quaterniond getEstimatedOrientation();
  	Eigen::Vector3d getEstimatedAngularVelocity();

    TranslationalEstimator translationalEstimator;
    RotationalEstimator rotationalEstimator;

  private:
    
};


}

#endif // VICON_ODOMETRY_ESTIMATOR_H
