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

// Ros includes
#include <ros/ros.h>
#include <ros_vrpn_client/rotationalEstimator.h>

 // Estimator include
#include "vicon_estimator.h"

//namespace vicon_estimation {

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

      void updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured);
  		Eigen::Vector3d getEstimatedPosition() const;
  		Eigen::Vector3d getEstimatedVelocity() const;
  		Eigen::Quaterniond getEstimatedOrientation() const;
  		Eigen::Vector3d getEstimatedAngularVelocity() const;

    private:
    	ViconEstimator viconEstimator_ ;

    	ros::Publisher publisher;

  };

    
//}


#endif // VICON_ODOMETRY_ESTIMATOR_H
