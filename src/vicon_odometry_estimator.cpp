
#include "vicon_odometry_estimator.h"

//namespace vicon_estimation {

ViconOdometryEstimator::ViconOdometryEstimator(ros::NodeHandle& nh) :
	viconEstimator_()
{
	// Creating publisher for intermediate estimator values
  publisher_ = nh.advertise<ros_vrpn_client::viconEstimator>("viconEstimator", 100);
}

void ViconOdometryEstimator::initializeParameters(ros::NodeHandle& nh)
{
  // Recovering translational estimator parameters values from the parameter server
  TranslationalEstimatorParameters translationalEstimatorParameters;
  nh.getParam("tranEst_dt", translationalEstimatorParameters.dt);
  nh.getParam("tranEst_kp", translationalEstimatorParameters.kp);
  nh.getParam("tranEst_kv", translationalEstimatorParameters.kv);
  // Recovering rotational estimator parameters values from the parameter server
  RotationalEstimatorParameters rotationalEstimatorParameters;
  nh.getParam("rotEst_dt", rotationalEstimatorParameters.dt);
  nh.getParam("rotEst_dQuat_hat_initialCovariance", rotationalEstimatorParameters.dQuat_hat_initialCovariance);
  nh.getParam("rotEst_dOmega_hat_initialCovariance", rotationalEstimatorParameters.dOmega_hat_initialCovariance);
  nh.getParam("rotEst_dQuat_processCovariance", rotationalEstimatorParameters.dQuat_processCovariance);
  nh.getParam("rotEst_dOmega_processCovariance", rotationalEstimatorParameters.dOmega_processCovariance);
  nh.getParam("rotEst_quat_measurementCovariance", rotationalEstimatorParameters.quat_measurementCovariance);
  // Setting parameters in estimator
  viconEstimator_.setParameters(translationalEstimatorParameters, rotationalEstimatorParameters);
}

void ViconOdometryEstimator::reset()
{
	viconEstimator_.reset();
}

void ViconOdometryEstimator::publishResults(ros::Time timestamp)
{

  TranslationalEstimatorResults translationalEstimatorResults;
  RotationalEstimatorResults rotationalEstimatorResults;
  viconEstimator_.getIntermediateResults(&translationalEstimatorResults, &rotationalEstimatorResults);

	// Creating estimator message
  ros_vrpn_client::viconEstimator msg ;
  // Attaching the vprn timestamp
  msg.header.stamp = timestamp ;

  // Writing the measurement to the message object
  msg.pos_measured.x = translationalEstimatorResults.pos_measured.x() ;
  msg.pos_measured.y = translationalEstimatorResults.pos_measured.y() ;
  msg.pos_measured.z = translationalEstimatorResults.pos_measured.z() ;
  // Writing the old estimates to the message object
  msg.pos_old.x = translationalEstimatorResults.pos_old.x() ;
  msg.pos_old.y = translationalEstimatorResults.pos_old.y() ;
  msg.pos_old.z = translationalEstimatorResults.pos_old.z() ;
  msg.vel_old.x = translationalEstimatorResults.vel_old.x() ;
  msg.vel_old.y = translationalEstimatorResults.vel_old.y() ;
  msg.vel_old.z = translationalEstimatorResults.vel_old.z() ;
  // Posteriori results
  msg.pos_est.x = translationalEstimatorResults.pos_est.x() ;
  msg.pos_est.y = translationalEstimatorResults.pos_est.y() ;
  msg.pos_est.z = translationalEstimatorResults.pos_est.z() ;
  msg.vel_est.x = translationalEstimatorResults.vel_est.x() ;
  msg.vel_est.y = translationalEstimatorResults.vel_est.y() ;
  msg.vel_est.z = translationalEstimatorResults.vel_est.z() ;


  // Writing the measurement to the message object
  msg.quat_measured.w = rotationalEstimatorResults.quat_measured.w() ;
  msg.quat_measured.x = rotationalEstimatorResults.quat_measured.x() ;
  msg.quat_measured.y = rotationalEstimatorResults.quat_measured.y() ;
  msg.quat_measured.z = rotationalEstimatorResults.quat_measured.z() ;
  // Writing the old estimates to the message object
	msg.quat_old.w = rotationalEstimatorResults.quat_old.w() ;
  msg.quat_old.x = rotationalEstimatorResults.quat_old.x() ;
  msg.quat_old.y = rotationalEstimatorResults.quat_old.y() ;
  msg.quat_old.z = rotationalEstimatorResults.quat_old.z() ;
  msg.omega_old.x = rotationalEstimatorResults.omega_old.x() ;
  msg.omega_old.y = rotationalEstimatorResults.omega_old.y() ;
  msg.omega_old.z = rotationalEstimatorResults.omega_old.z() ;
	// Posteriori results
	msg.quat_est.w = rotationalEstimatorResults.quat_est.w() ;	
  msg.quat_est.x = rotationalEstimatorResults.quat_est.x() ;
  msg.quat_est.y = rotationalEstimatorResults.quat_est.y() ;
  msg.quat_est.z = rotationalEstimatorResults.quat_est.z() ;
  msg.omega_est.x = rotationalEstimatorResults.omega_est.x() ;
  msg.omega_est.y = rotationalEstimatorResults.omega_est.y() ;
  msg.omega_est.z = rotationalEstimatorResults.omega_est.z() ;
  
	// Publishing estimator message
  publisher_.publish(msg);
}

void ViconOdometryEstimator::updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured)
{
  viconEstimator_.updateEstimate(pos_measured, quat_measured);
}

//}