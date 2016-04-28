

#include "test_helper_library.h"

void euler2quat(double roll, double pitch, double yaw, double* q) {
  // Compute quaternion
  Eigen::Quaterniond q_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  // Writing quaternion components to array output
  q[0] = q_.w();
  q[1] = q_.x();
  q[2] = q_.y();
  q[3] = q_.z();
}

void calculate3dRmsError(double truth[][3], double est[][3],
                         const int trajectory_length, const int start_index,
                         double* error) {
  // Looping over trajectories summing squared errors
  double error_sum[3] = {0.0, 0.0, 0.0};
  for (int i = start_index; i < trajectory_length; i++) {
    error_sum[0] += pow(truth[i][0] - est[i][0], 2);
    error_sum[1] += pow(truth[i][1] - est[i][1], 2);
    error_sum[2] += pow(truth[i][2] - est[i][2], 2);
  }
  // Averaging
  int num_samples = trajectory_length - start_index + 1;
  error_sum[0] /= num_samples;
  error_sum[1] /= num_samples;
  error_sum[2] /= num_samples;
  // Square rooting to obtain RMS
  error[0] = sqrt(error_sum[0]);
  error[1] = sqrt(error_sum[1]);
  error[2] = sqrt(error_sum[2]);
}

void calculateQuaternionRmsError(double truth[][4], double est[][4],
                                 const int trajectory_length,
                                 const int start_index, double* error) {
  // Generating the error trajectory
  double error_trajectory[trajectory_length][3];
  for (int i = 0; i < trajectory_length; i++) {
    // Turning vectors into quaternions
    Eigen::Quaterniond orientation_truth(truth[i][0], truth[i][1], truth[i][2],
                                         truth[i][3]);
    Eigen::Quaterniond orientation_estimate(est[i][0], est[i][1], est[i][2],
                                            est[i][3]);
    // Calculating the error quaternion
    Eigen::Quaterniond error_quaternion =
        orientation_estimate.inverse() * orientation_truth;
    // Extracting the three meaningful components of the error quaternion
    error_trajectory[i][0] = error_quaternion.x();
    error_trajectory[i][1] = error_quaternion.y();
    error_trajectory[i][2] = error_quaternion.z();
  }

  // Looping over trajectories summing squared errors
  double error_sum[3] = {0.0, 0.0, 0.0};
  for (int i = start_index; i < trajectory_length; i++) {
    error_sum[0] += pow(error_trajectory[i][0], 2);
    error_sum[1] += pow(error_trajectory[i][1], 2);
    error_sum[2] += pow(error_trajectory[i][2], 2);
  }
  // Averaging
  int num_samples = trajectory_length - start_index + 1;
  error_sum[0] /= num_samples;
  error_sum[1] /= num_samples;
  error_sum[2] /= num_samples;
  // Square rooting to obtain RMS
  error[0] = sqrt(error_sum[0]);
  error[1] = sqrt(error_sum[1]);
  error[2] = sqrt(error_sum[2]);
}
