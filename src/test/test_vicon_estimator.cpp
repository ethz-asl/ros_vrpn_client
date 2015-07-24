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
 *	Translational Estimator Tests
 */

void generateTranslationalTrajectorySinusoidal(double posTrajectory[][3], double velTrajectory[][3], const int trajectoryLength)
{
  // Generating position trajectories
  for (int i = 0; i < trajectoryLength; i++)
  {
    posTrajectory[i][0] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_X);
    posTrajectory[i][1] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Y);
    posTrajectory[i][2] = TRANS_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Z);
  }
  // Generating velocity trajectories from algebraic differentiation
  for (int i = 0; i < trajectoryLength; i++)
  {
    velTrajectory[i][0] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_X);
    velTrajectory[i][1] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Y);
    velTrajectory[i][2] = 2 * M_PI * TRANS_TRAJECTORY_FREQ * cos( 2 * M_PI * i * TRANS_TRAJECTORY_DT * TRANS_TRAJECTORY_FREQ + TRANS_TRAJECTORY_PHASE_OFFSET_Z);
  }
}

void calculate3dRmsError(double* error, double truth[][3], double est[][3], const int trajectoryLength, const int startIndex)
{
  // Looping over trajectories summing squared errors
  double errorSum[3] = { 0.0, 0.0, 0.0 };
  for (int i = startIndex; i < trajectoryLength; i++)
  {
    errorSum[0] += pow(truth[i][0] - est[i][0], 2);
    errorSum[1] += pow(truth[i][1] - est[i][1], 2);
    errorSum[2] += pow(truth[i][2] - est[i][2], 2);
  }
  // Averaging
  int numSamples = trajectoryLength - startIndex + 1;
  errorSum[0] /= numSamples;
  errorSum[1] /= numSamples;
  errorSum[2] /= numSamples;
  // Square rooting to obtain RMS
  error[0] = sqrt(errorSum[0]);
  error[1] = sqrt(errorSum[1]);
  error[2] = sqrt(errorSum[2]);
}

TEST(translationalEstimator, sinusoidal_clear)
{
  // Creating the estimator
  viconEstimator::TranslationalEstimator translationalEstimator;

  // Setting the estimator gains
  viconEstimator::TranslationalEstimatorParameters translationalEstimatorParameters;
  translationalEstimatorParameters.dt = TRANS_TRAJECTORY_DT;
  translationalEstimatorParameters.kp = 1.0;
  translationalEstimatorParameters.kv = 10 * 10.0;
  translationalEstimator.setParameters(translationalEstimatorParameters);
  translationalEstimator.reset();

  // Generating the trajectory over which to test the estimator
  const int trajectoryLength = static_cast<int>(TRANS_TRAJECTORY_PERIOD / TRANS_TRAJECTORY_DT) + 1;
  double posTrajectory[trajectoryLength][3], velTrajectory[trajectoryLength][3];
  generateTranslationalTrajectorySinusoidal(posTrajectory, velTrajectory, trajectoryLength);

  // Looping over trajectory and retrieving estimates
  double posEstTrajectory[trajectoryLength][3];
  double velEstTrajectory[trajectoryLength][3];
  for (int i = 0; i < trajectoryLength; i++)
  {
    // Constructing input
    Eigen::Vector3d input(posTrajectory[i][0], posTrajectory[i][1], posTrajectory[i][2]);
    // Updating the estimate with the measurement
    translationalEstimator.updateEstimate(input);
    // Getting the position and velocity estimates
    Eigen::Vector3d estimatedPosition = translationalEstimator.getEstimatedPosition();
    Eigen::Vector3d estimatedVelocity = translationalEstimator.getEstimatedVelocity();
    // Moving values to arrays
    posEstTrajectory[i][0] = estimatedPosition.x();
    posEstTrajectory[i][1] = estimatedPosition.y();
    posEstTrajectory[i][2] = estimatedPosition.z();
    velEstTrajectory[i][0] = estimatedVelocity.x();
    velEstTrajectory[i][1] = estimatedVelocity.y();
    velEstTrajectory[i][2] = estimatedVelocity.z();
  }

  // Start index for error calculation
  const int startIndex = trajectoryLength / 2;

  // Calculating position estimate errors
  double posError[3];
  calculate3dRmsError(posError, posTrajectory, posEstTrajectory, trajectoryLength, startIndex);

  // Performing test
  EXPECT_NEAR(posError[0], 0, POS_ERROR_THRESHOLD) << "X position estimate error too great";
  EXPECT_NEAR(posError[1], 0, POS_ERROR_THRESHOLD) << "Y position estimate error too great";
  EXPECT_NEAR(posError[2], 0, POS_ERROR_THRESHOLD) << "Z position estimate error too great";

  // Calculating velocity estimate errors
  double velError[3];
  calculate3dRmsError(velError, velTrajectory, velEstTrajectory, trajectoryLength, startIndex);

  // Performing test
  EXPECT_NEAR(velError[0], 0, VEL_ERROR_THRESHOLD) << "X velocity estimate error too great";
  EXPECT_NEAR(velError[1], 0, VEL_ERROR_THRESHOLD) << "Y velocity estimate error too great";
  EXPECT_NEAR(velError[2], 0, VEL_ERROR_THRESHOLD) << "Z velocity estimate error too great";

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
 *	Rotational Estimator Tests
 */

void euler2quat(double roll, double pitch, double yaw, double* q1, double* q2, double* q3, double* q4)
{
  double cos_z_2 = cos(yaw * 0.5);
  double cos_y_2 = cos(pitch * 0.5);
  double cos_x_2 = cos(roll * 0.5);

  double sin_z_2 = sin(yaw * 0.5);
  double sin_y_2 = sin(pitch * 0.5);
  double sin_x_2 = sin(roll * 0.5);

  // compute quaternion
  *q1 = cos_z_2 * cos_y_2 * cos_x_2 + sin_z_2 * sin_y_2 * sin_x_2;
  *q2 = cos_z_2 * cos_y_2 * sin_x_2 - sin_z_2 * sin_y_2 * cos_x_2;
  *q3 = cos_z_2 * sin_y_2 * cos_x_2 + sin_z_2 * cos_y_2 * sin_x_2;
  *q4 = sin_z_2 * cos_y_2 * cos_x_2 - cos_z_2 * sin_y_2 * sin_x_2;
}

void calculate3dRmsError(double* error, double truth[][4], double est[][4], const int trajectoryLength, const int startIndex)
{
  // Generating the error trajectory
  double errorTrajectory[trajectoryLength][3];
  for (int i = 0; i < trajectoryLength; i++)
  {
    // Turning vectors into quaternions
    Eigen::Quaterniond truthQuat(truth[i][0], truth[i][1], truth[i][2], truth[i][3]);
    Eigen::Quaterniond estQuat(est[i][0], est[i][1], est[i][2], est[i][3]);
    // Calculating the error quaternion
    Eigen::Quaterniond errorQuat = estQuat.inverse() * truthQuat;
    // Extracting the three meaningful components of the error quaternion
    errorTrajectory[i][0] = errorQuat.x();
    errorTrajectory[i][1] = errorQuat.y();
    errorTrajectory[i][2] = errorQuat.z();
  }

  // Looping over trajectories summing squared errors
  double errorSum[3] = { 0.0, 0.0, 0.0 };
  for (int i = startIndex; i < trajectoryLength; i++)
  {
    errorSum[0] += pow(errorTrajectory[i][0], 2);
    errorSum[1] += pow(errorTrajectory[i][1], 2);
    errorSum[2] += pow(errorTrajectory[i][2], 2);
  }
  // Averaging
  int numSamples = trajectoryLength - startIndex + 1;
  errorSum[0] /= numSamples;
  errorSum[1] /= numSamples;
  errorSum[2] /= numSamples;
  // Square rooting to obtain RMS
  error[0] = sqrt(errorSum[0]);
  error[1] = sqrt(errorSum[1]);
  error[2] = sqrt(errorSum[2]);
}

void generateRotationalTrajectorySinusoidal(double quatTrajectory[][4], double omegaTrajectory[][3], const int trajectoryLength)
{
  // Generating quaternion trajectories
  double roll, pitch, yaw;
  for (int i = 0; i < trajectoryLength; i++) {
    // Generating euler angles
    roll = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_X);
    pitch = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_Y);
    yaw = ROT_TRAJECTORY_AMPLITUDE * sin( 2 * M_PI * i * ROT_TRAJECTORY_DT * ROT_TRAJECTORY_FREQ + ROT_TRAJECTORY_PHASE_OFFSET_Z);
    // Converting to quaternions
    euler2quat(roll, pitch, yaw, &quatTrajectory[i][0], &quatTrajectory[i][1], &quatTrajectory[i][2], &quatTrajectory[i][3]);
  }

  // Generating the omega trajectories through numeric differentiation
  Eigen::Quaterniond q_k, q_k_1;
  for (int i = 0; i < trajectoryLength - 1; i++)
  {
    // Generating quaternions
    q_k = Eigen::Quaterniond(quatTrajectory[i + 1][0], quatTrajectory[i + 1][1], quatTrajectory[i + 1][2], quatTrajectory[i + 1][3]);
    q_k_1 = Eigen::Quaterniond(quatTrajectory[i][0], quatTrajectory[i][1], quatTrajectory[i][2], quatTrajectory[i][3]);
    // Calculating quaternion derivative
    Eigen::Quaterniond diff = Eigen::Quaterniond(
        (q_k.coeffs() - q_k_1.coeffs()) / ROT_TRAJECTORY_DT);
    Eigen::Quaterniond omega = Eigen::Quaterniond(2 * (q_k_1.inverse() * diff).coeffs());
    // Writing to the trajectory
    omegaTrajectory[i][0] = omega.x();
    omegaTrajectory[i][1] = omega.y();
    omegaTrajectory[i][2] = omega.z();
  }
}

TEST(rotationalEstimator, sinusoidal_clean)
{
  // Creating the estimator
  viconEstimator::RotationalEstimator rotationalEstimator;

  // Setting the estimator gains
  viconEstimator::RotationalEstimatorParameters rotationalEstimatorParameters;
  rotationalEstimatorParameters.dt = ROT_TRAJECTORY_DT;
  rotationalEstimatorParameters.dQuatHatInitialCovariance = 1;
  rotationalEstimatorParameters.dOmegaHatInitialCovariance = 1;
  rotationalEstimatorParameters.dQuatProcessCovariance = 0.01;
  rotationalEstimatorParameters.dOmegaProcessCovariance = 1000 * 1;
  rotationalEstimatorParameters.quatMeasurementCovariance = 0.0005;
  rotationalEstimator.setParameters(rotationalEstimatorParameters);
  rotationalEstimator.reset();

  // Generating the trajectory over which to test the estimator
  const int trajectoryLength = static_cast<int>(ROT_TRAJECTORY_PERIOD / ROT_TRAJECTORY_DT) + 1;
  double quatTrajectory[trajectoryLength][4], omegaTrajectory[trajectoryLength][3];
  generateRotationalTrajectorySinusoidal(quatTrajectory, omegaTrajectory, trajectoryLength);

  // Looping over trajectory and retrieving estimates
  double quatEstTrajectory[trajectoryLength][4];
  double omegaEstTrajectory[trajectoryLength][3];
  for (int i = 0; i < trajectoryLength; i++) {
    // Constructing input
    Eigen::Quaterniond input(quatTrajectory[i][0], quatTrajectory[i][1], quatTrajectory[i][2], quatTrajectory[i][3]);
    // Updating the estimate with the measurement
    rotationalEstimator.updateEstimate(input);
    // Getting the position and velocity estimates
    Eigen::Quaterniond estimatedOrientation = rotationalEstimator.getEstimatedOrientation();
    Eigen::Vector3d estimatedAngularVelocity = rotationalEstimator.getEstimatedAngularVelocity();
    // Moving values to arrays
    quatEstTrajectory[i][0] = estimatedOrientation.w();
    quatEstTrajectory[i][1] = estimatedOrientation.x();
    quatEstTrajectory[i][2] = estimatedOrientation.y();
    quatEstTrajectory[i][3] = estimatedOrientation.z();
    omegaEstTrajectory[i][0] = estimatedAngularVelocity.x();
    omegaEstTrajectory[i][1] = estimatedAngularVelocity.y();
    omegaEstTrajectory[i][2] = estimatedAngularVelocity.z();
  }

  // Start index for error calculation
  const int startIndex = trajectoryLength / 2;

  // Calculating position estimate errors
  double quatError[3];
  calculate3dRmsError(quatError, quatTrajectory, quatEstTrajectory, trajectoryLength, startIndex);

  // Performing test
  EXPECT_NEAR(quatError[0], 0, QUAT_ERROR_THRESHOLD) << "X errorquaternion estimate error too great";
  EXPECT_NEAR(quatError[1], 0, QUAT_ERROR_THRESHOLD) << "Y errorquaternion estimate error too great";
  EXPECT_NEAR(quatError[2], 0, QUAT_ERROR_THRESHOLD) << "Z errorquaternion estimate error too great";

  // Calculating velocity estimate errors
  double omegaError[3];
  calculate3dRmsError(omegaError, omegaTrajectory, omegaEstTrajectory, trajectoryLength,
                      startIndex);

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

/*
 *	GTests Main
 */

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
