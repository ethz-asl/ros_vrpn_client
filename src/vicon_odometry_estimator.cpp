
#include "vicon_odometry_estimator.h"

//namespace vicon_estimation {

ViconOdometryEstimator::ViconOdometryEstimator(ros::NodeHandle& nh) :
	viconEstimator_()
{
	// Creating publisher for intermediate estimator values
  publisher_ = nh.advertise<ros_vrpn_client::rotationalEstimator>("rotationalEstimator", 100);
}

void ViconOdometryEstimator::initializeParameters(ros::NodeHandle& nh)
{
  // Setting translational estimator parameters with values from the parameter server
  nh.getParam("tranEst_kp", viconEstimator_.translationalEstimator.estimator_parameters_.kp);
  nh.getParam("tranEst_kv", viconEstimator_.translationalEstimator.estimator_parameters_.kv);
  // Setting rotational estimator parameters with values from the parameter server
  nh.getParam("dt", viconEstimator_.rotationalEstimator.estimator_parameters_.dt);
  nh.getParam("dQuat_hat_initialCovariance", viconEstimator_.rotationalEstimator.estimator_parameters_.dQuat_hat_initialCovariance);
  nh.getParam("dOmega_hat_initialCovariance", viconEstimator_.rotationalEstimator.estimator_parameters_.dOmega_hat_initialCovariance);
  nh.getParam("dQuat_processCovariance", viconEstimator_.rotationalEstimator.estimator_parameters_.dQuat_processCovariance);
  nh.getParam("dOmega_processCovariance", viconEstimator_.rotationalEstimator.estimator_parameters_.dOmega_processCovariance);
  nh.getParam("quat_measurementCovariance", viconEstimator_.rotationalEstimator.estimator_parameters_.quat_measurementCovariance);
}

void ViconOdometryEstimator::reset()
{
	viconEstimator_.reset();
}

void ViconOdometryEstimator::publishResults(ros::Time timestamp)
{
	// Creating estimator message
  ros_vrpn_client::rotationalEstimator msg ;

  // Attaching the vprn timestamp
  msg.header.stamp = timestamp ;

  // Writing the measurement to the message object
  msg.quat_measured.w = viconEstimator_.rotationalEstimator.estimator_results_.quat_measured.w() ;
  msg.quat_measured.x = viconEstimator_.rotationalEstimator.estimator_results_.quat_measured.x() ;
  msg.quat_measured.y = viconEstimator_.rotationalEstimator.estimator_results_.quat_measured.y() ;
  msg.quat_measured.z = viconEstimator_.rotationalEstimator.estimator_results_.quat_measured.z() ;

	// Writing the old estimates to the message object
	msg.quat_old.w = viconEstimator_.rotationalEstimator.estimator_results_.quat_old.w() ;
  msg.quat_old.x = viconEstimator_.rotationalEstimator.estimator_results_.quat_old.x() ;
  msg.quat_old.y = viconEstimator_.rotationalEstimator.estimator_results_.quat_old.y() ;
  msg.quat_old.z = viconEstimator_.rotationalEstimator.estimator_results_.quat_old.z() ;
  msg.omega_old.x = viconEstimator_.rotationalEstimator.estimator_results_.omega_old.x() ;
  msg.omega_old.y = viconEstimator_.rotationalEstimator.estimator_results_.omega_old.y() ;
  msg.omega_old.z = viconEstimator_.rotationalEstimator.estimator_results_.omega_old.z() ;
  msg.dQuat_old.x = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_old.x() ;
  msg.dQuat_old.y = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_old.y() ;
  msg.dQuat_old.z = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_old.z() ;
  msg.dOmega_old.x = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_old.x() ;
  msg.dOmega_old.y = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_old.y() ;
  msg.dOmega_old.z = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_old.z() ;

	// Priori results
	msg.quat_p.w = viconEstimator_.rotationalEstimator.estimator_results_.quat_p.w() ;
	msg.quat_p.x = viconEstimator_.rotationalEstimator.estimator_results_.quat_p.x() ;
	msg.quat_p.y = viconEstimator_.rotationalEstimator.estimator_results_.quat_p.y() ;
	msg.quat_p.z = viconEstimator_.rotationalEstimator.estimator_results_.quat_p.z() ;
	msg.omega_p.x = viconEstimator_.rotationalEstimator.estimator_results_.omega_p.x() ;
	msg.omega_p.y = viconEstimator_.rotationalEstimator.estimator_results_.omega_p.y() ;
	msg.omega_p.z = viconEstimator_.rotationalEstimator.estimator_results_.omega_p.z() ;

  msg.dQuat_p.x = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_p.x() ;
  msg.dQuat_p.y = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_p.y() ;
  msg.dQuat_p.z = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_p.z() ;
  msg.dOmega_p.x = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_p.x() ;
  msg.dOmega_p.y = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_p.y() ;
  msg.dOmega_p.z = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_p.z() ;

	// Posteriori results
  msg.dQuat_m.x = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_m.x() ;
  msg.dQuat_m.y = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_m.y() ;
  msg.dQuat_m.z = viconEstimator_.rotationalEstimator.estimator_results_.dQuat_m.z() ;
  msg.dOmega_m.x = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_m.x() ;
  msg.dOmega_m.y = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_m.y() ;
  msg.dOmega_m.z = viconEstimator_.rotationalEstimator.estimator_results_.dOmega_m.z() ;

	msg.quat_m.w = viconEstimator_.rotationalEstimator.estimator_results_.quat_m.w() ;	
  msg.quat_m.x = viconEstimator_.rotationalEstimator.estimator_results_.quat_m.x() ;
  msg.quat_m.y = viconEstimator_.rotationalEstimator.estimator_results_.quat_m.y() ;
  msg.quat_m.z = viconEstimator_.rotationalEstimator.estimator_results_.quat_m.z() ;
  msg.omega_m.x = viconEstimator_.rotationalEstimator.estimator_results_.omega_m.x() ;
  msg.omega_m.y = viconEstimator_.rotationalEstimator.estimator_results_.omega_m.y() ;
  msg.omega_m.z = viconEstimator_.rotationalEstimator.estimator_results_.omega_m.z() ;

	// Publishing estimator message
  publisher_.publish(msg);
}

void ViconOdometryEstimator::updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured)
{
	viconEstimator_.updateEstimate(pos_measured, quat_measured);
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedPosition() const
{
	return viconEstimator_.getEstimatedPosition() ;
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedVelocity() const
{
	return viconEstimator_.getEstimatedVelocity() ;
}

Eigen::Quaterniond ViconOdometryEstimator::getEstimatedOrientation() const
{
	return viconEstimator_.getEstimatedOrientation() ;
}

Eigen::Vector3d ViconOdometryEstimator::getEstimatedAngularVelocity() const
{
	return viconEstimator_.getEstimatedAngularVelocity() ;
}


//}