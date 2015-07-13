
#include "vicon_odometry_estimator.h"

namespace vicon_estimation {

/*
* --------------------------------------------------------------------
* Vicon Odometry Estimator
* --------------------------------------------------------------------
*/

ViconOdometryEstimator::ViconOdometryEstimator() : 
  translationalEstimator(),
  rotationalEstimator() {
  //std::cout << "Initializing ViconOdometryEstimator" << std::endl;
}

void ViconOdometryEstimator::updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured)
{
	// Updating the translational and rotation sub-estimates
	translationalEstimator.updateEstimate(pos_measured);
	rotationalEstimator.updateEstimate(quat_measured);
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedPosition() const
{
	return translationalEstimator.getEstimatedPosition();
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedVelocity() const
{
	return translationalEstimator.getEstimatedVelocity();
}

Eigen::Quaterniond ViconOdometryEstimator::getEstimatedOrientation() const
{
	return rotationalEstimator.getEstimatedOrientation();
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedAngularVelocity() const
{
	return rotationalEstimator.getEstimatedAngularVelocity();
}

/*
* --------------------------------------------------------------------
* Translational Estimator
* --------------------------------------------------------------------
*/


TranslationalEstimator::TranslationalEstimator() : 
  pos_hat(0.0, 0.0, 0.0),
  vel_hat(0.0, 0.0, 0.0),
  estimator_parameters_()
{
  //std::cout << "Initializing TranslationalEstimator" << std::endl;
}

void TranslationalEstimator::updateEstimate(const Eigen::Vector3d& pos_measured)
{
	//std::cout << "Updating translational estimate" << std::endl;
  // Constructing the full state
  Eigen::Matrix<double, 6, 1> x_hat;
  x_hat << pos_hat, vel_hat;
  // Constructing the system matrix
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  A.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  A.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();
  A.block<3,3>(0, 3) = estimator_parameters_.dt*Eigen::Matrix3d::Identity();
  // Constructing the measurement matrix
  Eigen::Matrix<double, 3, 6> C;
  C.setZero();
  C.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  // Constructing the luenberger gain matrix
  Eigen::Matrix<double, 6, 3> L_gain;
  L_gain << estimator_parameters_.kp*Eigen::Matrix3d::Identity(), estimator_parameters_.kv*Eigen::Matrix3d::Identity();
  // Correction using the luenberger equations + gain
  x_hat = (A-L_gain*C*A)*x_hat + L_gain*pos_measured ;
  // Extracting state components
  pos_hat = x_hat.block<3,1>(0, 0) ;
  vel_hat = x_hat.block<3,1>(3, 0) ;
}

Eigen::Vector3d TranslationalEstimator::getEstimatedPosition() const {
  return pos_hat ;
}  

Eigen::Vector3d TranslationalEstimator::getEstimatedVelocity() const {
  return vel_hat ;
}  

/*
* --------------------------------------------------------------------
* Rotational Estimator
* --------------------------------------------------------------------
*/

RotationalEstimator::RotationalEstimator() : 
  quat_hat(0.0, 0.0, 0.0, 0.0),
  omega_hat(0.0, 0.0, 0.0),
  estimator_parameters_()
{
  //std::cout << "Initializing RotationalEstimator" << std::endl;
}

void RotationalEstimator::updateEstimate(const Eigen::Quaterniond& quat_measured)
{
	//std::cout << "Updating rotational estimate" << std::endl;
	// Propagating the estimate


}

Eigen::Quaterniond RotationalEstimator::getEstimatedOrientation() const {
  return quat_hat ;
}  

Eigen::Vector3d RotationalEstimator::getEstimatedAngularVelocity() const {
  return omega_hat ;
}  


}