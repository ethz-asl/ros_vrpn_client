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

#include <math.h>
#include <iostream>
#include <fstream>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "vicon_estimator.h"
#include "test_helper_library.h"

// Translational trajectory defines
#define TRANS_TRAJECTORY_PERIOD 10.0
#define TRANS_TRAJECTORY_AMPLITUDE 1.0
#define TRANS_TRAJECTORY_DT 0.01
#define TRANS_TRAJECTORY_FREQ 0.5
#define TRANS_TRAJECTORY_PHASE_OFFSET_X 0.0
#define TRANS_TRAJECTORY_PHASE_OFFSET_Y M_PI/2.0
#define TRANS_TRAJECTORY_PHASE_OFFSET_Z M_PI/1.0

// Rotational trajectory defines
#define ROT_TRAJECTORY_PERIOD 10.0
#define ROT_TRAJECTORY_AMPLITUDE 1.0*M_PI
#define ROT_TRAJECTORY_DT 0.01
#define ROT_TRAJECTORY_FREQ 0.5
#define ROT_TRAJECTORY_PHASE_OFFSET_X 0.0
#define ROT_TRAJECTORY_PHASE_OFFSET_Y M_PI/2.0
#define ROT_TRAJECTORY_PHASE_OFFSET_Z M_PI/1.0

#define POS_ERROR_THRESHOLD 0.01
#define VEL_ERROR_THRESHOLD 0.1

#define QUAT_ERROR_THRESHOLD 0.005
#define OMEGA_ERROR_THRESHOLD 1.0


/*
 *  Helper Function Tests
 */

/*TEST(helperFunctions, euler2Quat)
{
  //
  double roll = 1.0/2.0*M_PI ;
  double pitch = 0.0 ;
  double yaw = 0.0 ;
  void euler2quat(double roll, double pitch, double yaw, double* q1, double* q2, double* q3, double* q4);


}*/


/*
 *  Translational Estimator Tests
 */

void generateTranslationalTrajectorySinusoidal(double position_trajectory[][3], double velocity_trajectory[][3], const int trajectory_length)
{
  // Generating position trajectories
  for (int i = 0; i < trajectory_length; i++)
  {
    position_trajectory[i][0] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_X);
    position_trajectory[i][1] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Y);
    position_trajectory[i][2] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Z);
  }
  // Generating velocity trajectories from algebraic differentiation
  for (int i = 0; i < trajectory_length; i++)
  {
    velocity_trajectory[i][0] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_X);
    velocity_trajectory[i][1] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Y);
    velocity_trajectory[i][2] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Z);
  }
}

TEST(translationalEstimator, sinusoidal_clean)
{
  // Creating the estimator
  vicon_estimator::TranslationalEstimator translational_estimator;

  // Setting the estimator gains
  vicon_estimator::TranslationalEstimatorParameters translational_estimator_parameters;
  translational_estimator_parameters.dt_ = TRANS_TRAJECTORY_DT;
  translational_estimator_parameters.kp_ = 1.0;
  translational_estimator_parameters.kv_ = 10 * 10.0;
  translational_estimator.setParameters(translational_estimator_parameters);
  translational_estimator.reset();

  // Generating the trajectory over which to test the estimator
  const int trajectory_length = static_cast<int>(TRANS_TRAJECTORY_PERIOD / TRANS_TRAJECTORY_DT) + 1;
  double position_trajectory[trajectory_length][3];
  double velocity_trajectory[trajectory_length][3];
  generateTranslationalTrajectorySinusoidal(position_trajectory, velocity_trajectory, trajectory_length);

  // Looping over trajectory and retrieving estimates
  double position_trajectory_estimate[trajectory_length][3];
  double velocity_trajectory_estimate[trajectory_length][3];
  for (int i = 0; i < trajectory_length; i++)
  {
    // Constructing input
    Eigen::Vector3d input(position_trajectory[i][0], position_trajectory[i][1], position_trajectory[i][2]);
    // Updating the estimate with the measurement
    translational_estimator.updateEstimate(input);
    // Getting the position and velocity estimates
    Eigen::Vector3d estimated_position = translational_estimator.getEstimatedPosition();
    Eigen::Vector3d estimated_velocity = translational_estimator.getEstimatedVelocity();
    // Moving values to arrays
    position_trajectory_estimate[i][0] = estimated_position.x();
    position_trajectory_estimate[i][1] = estimated_position.y();
    position_trajectory_estimate[i][2] = estimated_position.z();
    velocity_trajectory_estimate[i][0] = estimated_velocity.x();
    velocity_trajectory_estimate[i][1] = estimated_velocity.y();
    velocity_trajectory_estimate[i][2] = estimated_velocity.z();
  }

  // Start index for error calculation
  const int start_index = trajectory_length / 2;

  // Calculating position estimate errors
  double position_error[3];
  calculate3dRmsError(position_trajectory, position_trajectory_estimate, trajectory_length, start_index, position_error);

  // Performing test
  EXPECT_NEAR(position_error[0], 0, POS_ERROR_THRESHOLD) << "X position estimate error too great";
  EXPECT_NEAR(position_error[1], 0, POS_ERROR_THRESHOLD) << "Y position estimate error too great";
  EXPECT_NEAR(position_error[2], 0, POS_ERROR_THRESHOLD) << "Z position estimate error too great";

  // Calculating velocity estimate errors
  double velocity_error[3];
  calculate3dRmsError(velocity_trajectory, velocity_trajectory_estimate, trajectory_length, start_index, velocity_error);

  // Performing test
  EXPECT_NEAR(velocity_error[0], 0, VEL_ERROR_THRESHOLD) << "X velocity estimate error too great";
  EXPECT_NEAR(velocity_error[1], 0, VEL_ERROR_THRESHOLD) << "Y velocity estimate error too great";
  EXPECT_NEAR(velocity_error[2], 0, VEL_ERROR_THRESHOLD) << "Z velocity estimate error too great";

  /*
   *  For matlab debug
   */

/*  // Opening a file for trajectories debug data
  std::ofstream trajectoriesFile;
  trajectoriesFile.open("translationalTrajectories.txt");
  // Writing the data to file
  for (int i = 0; i < trajectoryLength; i++)
  {
    trajectoriesFile << posTrajectory[i][0] << ", " << posTrajectory[i][1] << ", " << posTrajectory[i][2] << ", ";
    trajectoriesFile << posEstTrajectory[i][0] << ", " << posEstTrajectory[i][1] << ", " << posEstTrajectory[i][2] << ", ";
    trajectoriesFile << velTrajectory[i][0] << ", " << velTrajectory[i][1] << ", " << velTrajectory[i][2] << ", ";
    trajectoriesFile << velEstTrajectory[i][0] << ", " << velEstTrajectory[i][1] << ", " << velEstTrajectory[i][2];
    trajectoriesFile << std::endl;
  }
  // Closing file
  trajectoriesFile.close();

  // Opening a file for errors debug data
  std::ofstream errorsFile;
  errorsFile.open("translationalErrors.txt");
  // Writing the data to file
  errorsFile << posError[0] << ", " << posError[1] << ", " << posError[2] << std::endl;
  errorsFile << velError[0] << ", " << velError[1] << ", " << velError[2] << std::endl;
  // Closing file
  errorsFile.close();*/

}

/*
 *  Rotational Estimator Tests
 */


void generateRotationalTrajectorySinusoidal(double orientation_trajectory[][4], double rollrate_trajectory[][3], const int trajectory_length)
{
  // Generating quaternion trajectories
  double roll, pitch, yaw;
  for (int i = 0; i < trajectory_length; i++) {
    // Generating euler angles
    roll = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_X);
    pitch = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_Y);
    yaw = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_Z);
    // Converting to quaternions
    euler2quat(roll, pitch, yaw, orientation_trajectory[i]);
  }

  // Generating the omega trajectories through numeric differentiation
  Eigen::Quaterniond q_k, q_k_1;
  for (int i = 0; i < trajectory_length - 1; i++)
  {
    // Generating quaternions
    q_k = Eigen::Quaterniond(orientation_trajectory[i + 1][0], orientation_trajectory[i + 1][1], orientation_trajectory[i + 1][2], orientation_trajectory[i + 1][3]);
    q_k_1 = Eigen::Quaterniond(orientation_trajectory[i][0], orientation_trajectory[i][1], orientation_trajectory[i][2], orientation_trajectory[i][3]);
    // Calculating quaternion derivative
    Eigen::Quaterniond diff = Eigen::Quaterniond(
        (q_k.coeffs() - q_k_1.coeffs()) / ROT_TRAJECTORY_DT);
    Eigen::Quaterniond omega = Eigen::Quaterniond(2 * (q_k_1.inverse() * diff).coeffs());
    // Writing to the trajectory
    rollrate_trajectory[i][0] = omega.x();
    rollrate_trajectory[i][1] = omega.y();
    rollrate_trajectory[i][2] = omega.z();
  }
}

TEST(rotationalEstimator, sinusoidal_clean)
{
  // Creating the estimator
  vicon_estimator::RotationalEstimator rotational_estimator;

  // Setting the estimator gains
  vicon_estimator::RotationalEstimatorParameters rotational_estimator_parameters;
  rotational_estimator_parameters.dt_ = ROT_TRAJECTORY_DT;
  rotational_estimator_parameters.dorientation_estimate_initial_covariance_ = 1;
  rotational_estimator_parameters.drate_estimate_initial_covariance_ = 1;
  rotational_estimator_parameters.dorientation_process_covariance_ = 0.01;
  rotational_estimator_parameters.drate_process_covariance_ = 1000 * 1;
  rotational_estimator_parameters.orientation_measurement_covariance_ = 0.0005;
  rotational_estimator.setParameters(rotational_estimator_parameters);
  rotational_estimator.reset();

  // Generating the trajectory over which to test the estimator
  const int trajectory_length = static_cast<int>(ROT_TRAJECTORY_PERIOD / ROT_TRAJECTORY_DT) + 1;
  double orientation_trajectory[trajectory_length][4], omega_trajectory[trajectory_length][3];
  generateRotationalTrajectorySinusoidal(orientation_trajectory, omega_trajectory, trajectory_length);

  // Looping over trajectory and retrieving estimates
  double orientation_estimate_trajectory[trajectory_length][4];
  double rollrate_estimate_trajectory[trajectory_length][3];
  for (int i = 0; i < trajectory_length; i++) {
    // Constructing input
    Eigen::Quaterniond input(orientation_trajectory[i][0], orientation_trajectory[i][1], orientation_trajectory[i][2], orientation_trajectory[i][3]);
    // Updating the estimate with the measurement
    rotational_estimator.updateEstimate(input);
    // Getting the position and velocity estimates
    Eigen::Quaterniond estimated_orientation = rotational_estimator.getEstimatedOrientation();
    Eigen::Vector3d estimated_rollrate = rotational_estimator.getEstimatedRate();
    // Moving values to arrays
    orientation_estimate_trajectory[i][0] = estimated_orientation.w();
    orientation_estimate_trajectory[i][1] = estimated_orientation.x();
    orientation_estimate_trajectory[i][2] = estimated_orientation.y();
    orientation_estimate_trajectory[i][3] = estimated_orientation.z();
    rollrate_estimate_trajectory[i][0] = estimated_rollrate.x();
    rollrate_estimate_trajectory[i][1] = estimated_rollrate.y();
    rollrate_estimate_trajectory[i][2] = estimated_rollrate.z();
  }

  // Start index for error calculation
  const int start_index = trajectory_length / 2;

  // Calculating position estimate errors
  double orientation_error[3];
  calculateQuaternionRmsError(orientation_trajectory, orientation_estimate_trajectory, trajectory_length, start_index, orientation_error);

  // Performing test
  EXPECT_NEAR(orientation_error[0], 0, QUAT_ERROR_THRESHOLD) << "X errorquaternion estimate error too great";
  EXPECT_NEAR(orientation_error[1], 0, QUAT_ERROR_THRESHOLD) << "Y errorquaternion estimate error too great";
  EXPECT_NEAR(orientation_error[2], 0, QUAT_ERROR_THRESHOLD) << "Z errorquaternion estimate error too great";

  // Calculating velocity estimate errors
  double omegaError[3];
  calculate3dRmsError(omega_trajectory, rollrate_estimate_trajectory, trajectory_length,
                      start_index, omegaError);

  // Performing test
  EXPECT_NEAR(omegaError[0], 0, OMEGA_ERROR_THRESHOLD) << "X rotational velocity estimate error too great";
  EXPECT_NEAR(omegaError[1], 0, OMEGA_ERROR_THRESHOLD) << "Y rotational velocity estimate error too great";
  EXPECT_NEAR(omegaError[2], 0, OMEGA_ERROR_THRESHOLD) << "Z rotational velocity estimate error too great";

  /*
   *  For matlab debug
   */

/*  // Opening a file for trajectories debug data
  std::ofstream trajectoriesFile;
  trajectoriesFile.open("rotationalTrajectories.txt");
  // Writing the data to file
  for (int i = 0; i < trajectoryLength; i++)
  {
    trajectoriesFile << quatTrajectory[i][0] << ", " << quatTrajectory[i][1] << ", " << quatTrajectory[i][2] << ", " << quatTrajectory[i][3] << ", ";
    trajectoriesFile << quatEstTrajectory[i][0] << ", " << quatEstTrajectory[i][1] << ", " << quatEstTrajectory[i][2] << ", " << quatEstTrajectory[i][3] << ", ";
    trajectoriesFile << omegaTrajectory[i][0] << ", " << omegaTrajectory[i][1] << ", " << omegaTrajectory[i][2] << ", ";
    trajectoriesFile << omegaEstTrajectory[i][0] << ", " << omegaEstTrajectory[i][1] << ", " << omegaEstTrajectory[i][2];
    trajectoriesFile << std::endl;
  }
  // Closing file
  trajectoriesFile.close();

  // Opening a file for errors debug data
  std::ofstream errorsFile;
  errorsFile.open("rotationalErrors.txt");
  // Writing the data to file
  errorsFile << quatError[0] << ", " << quatError[1] << ", " << quatError[2] << std::endl;
  errorsFile << omegaError[0] << ", " << omegaError[1] << ", " << omegaError[2] << std::endl;
  // Closing file
  errorsFile.close();*/

}

// Quaternions representing flips in various dimensions
const int kNumberOfInversions = 7;
const Eigen::Quaterniond rotation_inversion_x = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
const Eigen::Quaterniond rotation_inversion_y = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
const Eigen::Quaterniond rotation_inversion_z = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
const Eigen::Quaterniond rotation_inversion_xy = rotation_inversion_x * rotation_inversion_y;
const Eigen::Quaterniond rotation_inversion_yz = rotation_inversion_y * rotation_inversion_z;
const Eigen::Quaterniond rotation_inversion_xz = rotation_inversion_x * rotation_inversion_z;
const Eigen::Quaterniond rotation_inversion_xyz = rotation_inversion_x * rotation_inversion_y * rotation_inversion_z;
const Eigen::Quaterniond rotation_inversions[kNumberOfInversions] = { rotation_inversion_x,
                                                                      rotation_inversion_y, 
                                                                      rotation_inversion_z,
                                                                      rotation_inversion_xy,
                                                                      rotation_inversion_yz, 
                                                                      rotation_inversion_xz,
                                                                      rotation_inversion_xyz };

void generateCorruptedInputTrajectory(const Eigen::Quaterniond clean_trajectory[], const int trajectory_length, const int corruption_rate, Eigen::Quaterniond corrupted_trajectory[])
{
  // Looping over the clean input trajecty and corruputing measurements at a rate determined by corruption_rate
  // Note that corruption starts only after corruption_rate samples.
  int inversion_index = 0;
  for(int i = corruption_rate; i <= trajectory_length; i+=corruption_rate)
  {
    // Corrupting the measurement with an inversion from the inversion list
    corrupted_trajectory[i] = rotation_inversions[inversion_index]*clean_trajectory[i];
    //corrupted_trajectory[i] = clean_trajectory[i];
    // Incrementing the inversion
    inversion_index = (inversion_index + 1) % kNumberOfInversions;
  }
}


TEST(rotationalEstimator, sinusoidal_corrupted)
{
  // Creating the estimator
  vicon_estimator::RotationalEstimator rotational_estimator;

  // Setting the estimator gains
  vicon_estimator::RotationalEstimatorParameters rotational_estimator_parameters;
  rotational_estimator_parameters.dt_ = ROT_TRAJECTORY_DT;
  rotational_estimator_parameters.dorientation_estimate_initial_covariance_ = 1;
  rotational_estimator_parameters.drate_estimate_initial_covariance_ = 1;
  rotational_estimator_parameters.dorientation_process_covariance_ = 0.01;
  rotational_estimator_parameters.drate_process_covariance_ = 1000 * 1;
  rotational_estimator_parameters.orientation_measurement_covariance_ = 0.0005;
  rotational_estimator.setParameters(rotational_estimator_parameters);
  rotational_estimator.reset();

  // Generating the trajectory over which to test the estimator
  const int trajectory_length = static_cast<int>(ROT_TRAJECTORY_PERIOD / ROT_TRAJECTORY_DT) + 1;
  double orientation_trajectory[trajectory_length][4], omega_trajectory[trajectory_length][3];
  generateRotationalTrajectorySinusoidal(orientation_trajectory, omega_trajectory, trajectory_length);



  // Constructing the measurement vector
  Eigen::Quaterniond clean_input_trajectory[trajectory_length];
  Eigen::Quaterniond corrupted_input_trajectory[trajectory_length];
  // Creating the clean measurement vector
  for (int i = 0; i < trajectory_length; i++) {
    clean_input_trajectory[i] = Eigen::Quaterniond(orientation_trajectory[i][0], orientation_trajectory[i][1], orientation_trajectory[i][2], orientation_trajectory[i][3]);
    corrupted_input_trajectory[i] = Eigen::Quaterniond(orientation_trajectory[i][0], orientation_trajectory[i][1], orientation_trajectory[i][2], orientation_trajectory[i][3]);
  }
  // Corrupting the measurement vector
  int corruption_rate = 100; //TODO(millanea): Parameter above.
  generateCorruptedInputTrajectory(clean_input_trajectory, trajectory_length, corruption_rate, corrupted_input_trajectory);


  // Looping over trajectory and retrieving estimates
  double orientation_estimate_trajectory[trajectory_length][4];
  double rollrate_estimate_trajectory[trajectory_length][3];
  for (int i = 0; i < trajectory_length; i++) {
    // Updating the estimate with the measurement
    rotational_estimator.updateEstimate(corrupted_input_trajectory[i]);
    // Getting the position and velocity estimates
    Eigen::Quaterniond estimated_orientation = rotational_estimator.getEstimatedOrientation();
    Eigen::Vector3d estimated_rollrate = rotational_estimator.getEstimatedRate();
    // Moving values to arrays
    orientation_estimate_trajectory[i][0] = estimated_orientation.w();
    orientation_estimate_trajectory[i][1] = estimated_orientation.x();
    orientation_estimate_trajectory[i][2] = estimated_orientation.y();
    orientation_estimate_trajectory[i][3] = estimated_orientation.z();
    rollrate_estimate_trajectory[i][0] = estimated_rollrate.x();
    rollrate_estimate_trajectory[i][1] = estimated_rollrate.y();
    rollrate_estimate_trajectory[i][2] = estimated_rollrate.z();
  }

  // Start index for error calculation
  const int start_index = trajectory_length / 2;

  // Calculating position estimate errors
  double orientation_error[3];
  calculateQuaternionRmsError(orientation_trajectory, orientation_estimate_trajectory, trajectory_length, start_index, orientation_error);

  // Performing test
  EXPECT_NEAR(orientation_error[0], 0, QUAT_ERROR_THRESHOLD) << "X errorquaternion estimate error too great";
  EXPECT_NEAR(orientation_error[1], 0, QUAT_ERROR_THRESHOLD) << "Y errorquaternion estimate error too great";
  EXPECT_NEAR(orientation_error[2], 0, QUAT_ERROR_THRESHOLD) << "Z errorquaternion estimate error too great";

  // Calculating velocity estimate errors
  double omegaError[3];
  calculate3dRmsError(omega_trajectory, rollrate_estimate_trajectory, trajectory_length,
                      start_index, omegaError);

  // Performing test
  EXPECT_NEAR(omegaError[0], 0, OMEGA_ERROR_THRESHOLD) << "X rotational velocity estimate error too great";
  EXPECT_NEAR(omegaError[1], 0, OMEGA_ERROR_THRESHOLD) << "Y rotational velocity estimate error too great";
  EXPECT_NEAR(omegaError[2], 0, OMEGA_ERROR_THRESHOLD) << "Z rotational velocity estimate error too great";

}

/*
 *  GTests Main
 */

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
