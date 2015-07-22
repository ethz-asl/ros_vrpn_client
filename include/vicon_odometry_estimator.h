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

#include <iostream>
#include <stdio.h>
#include <math.h>

 #include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros_vrpn_client/viconEstimator.h>

#include "vicon_estimator.h"

namespace viconEstimator {

class ViconOdometryEstimator
{

  public:
  	// Constructor
    ViconOdometryEstimator(ros::NodeHandle& nh); 

		// Initialize the estimator parameters
    void initializeParameters(ros::NodeHandle& nh);
    // Reset the estimator
  	void reset();
  	// Publishing the intermediate results
  	void publishResults(ros::Time timestamp);

    // Calls the underlying estimator, updating the estimate with the latest measurement
    void updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured);
    // Getter methods for estimates values
		Eigen::Vector3d getEstimatedPosition() const { viconEstimator_.getEstimatedPosition(); };
		Eigen::Vector3d getEstimatedVelocity() const { viconEstimator_.getEstimatedVelocity(); };
		Eigen::Quaterniond getEstimatedOrientation() const { viconEstimator_.getEstimatedOrientation(); };
		Eigen::Vector3d getEstimatedAngularVelocity() const { viconEstimator_.getEstimatedAngularVelocity(); };

  private:
    // Underlying estimator
  	viconEstimator::ViconEstimator viconEstimator_;
    // Publisher
  	ros::Publisher publisher_;

};

    
}


#endif // VICON_ODOMETRY_ESTIMATOR_H