


#include "test_helper_library.h"

void euler2quat(double roll, double pitch, double yaw, double* q)
{
  // Compute quaternion
  Eigen::Quaterniond q_ =  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) ;
  // Writing quaternion components to array output
  q[0] = q_.w() ;
  q[1] = q_.x() ;
  q[2] = q_.y() ;
  q[3] = q_.z() ;
}


void calculate3dRmsError(double truth[][3], double est[][3], const int trajectoryLength, const int startIndex, double* error)
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

void calculateQuatRmsError(double truth[][4], double est[][4], const int trajectoryLength, const int startIndex, double* error)
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


