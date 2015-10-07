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

// This file implements a node which subscribes to published raw vicon
// position and orientation measurements and, the vicon estimator and
// finally publishes position, velcocity, orientation and angular
// velocity estimates.

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include "vicon_odometry_estimator.h"

// Class for collecting data and passing it to the underlying estimator
class ViconDataListener {

  public:
    // Constructor
    ViconDataListener(ros::NodeHandle nh, ros::NodeHandle nh_private)
    {
      raw_transform_sub_ = nh.subscribe("vrpn_client/raw_transform", 10, &ViconDataListener::transformStampedCallback, this);
    }

    // Raw vicon data callback.
    void transformStampedCallback(const geometry_msgs::TransformStampedConstPtr& msg)
    {
      std::cout << "Transform message received." << std::endl;
    }

  private:
    // Raw vicon data subscriber.
    ros::Subscriber raw_transform_sub_;
};

// Standard C++ entry point
int main(int argc, char** argv)
{
  // Announce this program to the ROS master
  ros::init(argc, argv, "vicon_to_csv_node");

  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //DEBUG
  std::cout << "Test" << std::endl;

  // Exit tranquilly
  return 0;
}
