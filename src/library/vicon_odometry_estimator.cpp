
#include "vicon_odometry_estimator.h"

namespace vicon_estimation {

/*
* --------------------------------------------------------------------
* Vicon Odometry Estimator
* --------------------------------------------------------------------
*/

ViconOdometryEstimator::ViconOdometryEstimator(ros::NodeHandle& nh) : 
  translationalEstimator(nh),
  rotationalEstimator(nh) {
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


TranslationalEstimator::TranslationalEstimator(ros::NodeHandle& nh) : 
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

RotationalEstimator::RotationalEstimator(ros::NodeHandle& nh) : 
  quat_hat(1.0, 0.0, 0.0, 0.0),
  omega_hat(0.0, 0.0, 0.0),
  dQuat_hat(0.0, 0.0, 0.0),
  dOmega_hat(0.0, 0.0, 0.0),
  estimator_parameters_()
{
	// Initializing the initial covariance
	covariance << estimator_parameters_.dQuat_hat_initialCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
								Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_hat_initialCovariance*Eigen::Matrix3d::Identity() ;
	// Constructing process and measurement covariance matrices
	processCovariance << 	estimator_parameters_.dQuat_processCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
												Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_processCovariance*Eigen::Matrix3d::Identity() ;
	measurementCovariance << 	estimator_parameters_.quat_measurementCovariance*Eigen::Matrix4d::Identity() ;
	// Creating publisher for intermediate estimator values
  publisher = nh.advertise<ros_vrpn_client::rotationalEstimator>("rotationalEstimator", 100);
}

void RotationalEstimator::updateEstimate(const Eigen::Quaterniond& quat_measured)
{
	// Creating estimator message
  ros_vrpn_client::rotationalEstimator msg ;

	// Writing the old estimates to the message object
	msg.quat_old.w = quat_hat.w() ;
  msg.quat_old.x = quat_hat.x() ;
  msg.quat_old.y = quat_hat.y() ;
  msg.quat_old.z = quat_hat.z() ;
  msg.omega_old.x = omega_hat.x() ;
  msg.omega_old.y = omega_hat.y() ;
  msg.omega_old.z = omega_hat.z() ;
  msg.dQuat_old.x = dQuat_hat.x() ;
  msg.dQuat_old.y = dQuat_hat.y() ;
  msg.dQuat_old.z = dQuat_hat.z() ;
  msg.dOmega_old.x = dOmega_hat.x() ;
  msg.dOmega_old.y = dOmega_hat.y() ;
  msg.dOmega_old.z = dOmega_hat.z() ;
  
	// Propagating the global state estimate
	Eigen::Matrix<double, 7, 1> x_old ;
	Eigen::Matrix<double, 7, 1> x_p ;
	x_old << quat_hat.coeffs(), omega_hat ;
	updateEstimate_propagateGlobalEstimate(&x_p, x_old, &msg) ;

	// Propagating the error state estimate
	Eigen::Matrix<double, 6, 1> dx_old ;
	Eigen::Matrix<double, 6, 1> dx_p ;
	dx_old << dQuat_hat, dOmega_hat ;
	updateEstimate_propagateErrorEstimate(&dx_p, dx_old, x_old, &msg);

	// Propagating the estimate covariance
	Eigen::Matrix<double, 6, 6> P_old ;
	Eigen::Matrix<double, 6, 6> P_p ;
	P_old = covariance ;
	updateEstimate_propagateErrorCovariance(&P_p, P_old, x_old, &msg);

	// Measurement Update
	Eigen::Matrix<double, 6, 1> dx_m ;
	Eigen::Matrix<double, 6, 6> P_m ;
	updateEstimate_updateErrorEstimate(&dx_m, &P_m, quat_measured, x_p, dx_p, P_p, &msg);

	// Global state correction
	Eigen::Matrix<double, 7, 1> x_m ;
	updateEstimate_recombineErrorGlobal(&x_m, &dx_m, x_p, &msg);

	// Extracting estimated quantities from the posteriori state
	quat_hat = Eigen::Quaterniond(x_m.block<4,1>(0,0));
	omega_hat = x_m.block<3,1>(4,0);
	covariance = P_m;
/*	quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0,0));
	omega_hat = x_old.block<3,1>(4,0);
	covariance = P_old;*/

/*	// Outputting the messages to the console
	std::cout << "quat_measured: " << std::endl << quat_measured.coeffs() << std::endl ;
  std::cout << "quat_hat: " << std::endl << quat_hat.coeffs() << std::endl ;
	std::cout << "omega_hat: " << std::endl << omega_hat << std::endl ;
*/
	// Publishing estimator message
  publisher.publish(msg) ;
}

Eigen::Quaterniond RotationalEstimator::getEstimatedOrientation() const
{
  return quat_hat ;
}  

Eigen::Vector3d RotationalEstimator::getEstimatedAngularVelocity() const
{
  return omega_hat ;
}

Eigen::Matrix3d RotationalEstimator::skewMatrix(const Eigen::Vector3d& vec ) const
{
	Eigen::Matrix3d vec_cross;
	vec_cross << 	0, 				-vec(2), 	vec(1),
    						vec(2), 	0, 				-vec(0),
    						-vec(1), 	vec(0), 	0;
	return vec_cross;
}

void RotationalEstimator::updateEstimate_propagateGlobalEstimate(Eigen::Matrix<double, 7, 1>* x_p, const Eigen::Matrix<double, 7, 1>& x_old, ros_vrpn_client::rotationalEstimator* msg)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	// Performing propagation
	Eigen::Quaterniond omega_hat_q = Eigen::Quaterniond(0, omega_hat.x(), omega_hat.y(), omega_hat.z());
	Eigen::Quaterniond quat_hat_p_ROC = Eigen::Quaterniond(0.5*(quat_hat*omega_hat_q).coeffs()); // Using hamilton definition
	Eigen::Vector3d omega_hat_p_ROC = Eigen::Vector3d::Zero();
	Eigen::Quaterniond quat_hat_p = Eigen::Quaterniond(quat_hat.coeffs() + quat_hat_p_ROC.coeffs()*estimator_parameters_.dt);
	Eigen::Vector3d omega_hat_p = omega_hat + omega_hat_p_ROC*estimator_parameters_.dt;
	// Writing to apriori state
	*x_p << quat_hat_p.coeffs(), omega_hat_p ;
	// Writing to the message object
	msg->quat_p.w = quat_hat_p.w() ;
  msg->quat_p.x = quat_hat_p.x() ;
  msg->quat_p.y = quat_hat_p.y() ;
  msg->quat_p.z = quat_hat_p.z() ;
  msg->omega_p.x = omega_hat_p.x() ;
  msg->omega_p.y = omega_hat_p.y() ;
  msg->omega_p.z = omega_hat_p.z() ;
}

void RotationalEstimator::updateEstimate_propagateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_p, const Eigen::Matrix<double, 6, 1>& dx_old, const Eigen::Matrix<double, 7, 1>& x_old,
																																ros_vrpn_client::rotationalEstimator* msg)
{
	// Extracting components of the states
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat = dx_old.block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat = dx_old.block<3,1>(3, 0) ;
	// Performing propagation
	Eigen::Vector3d dQuat_hat_p_ROC = -omega_hat.cross(dQuat_hat) + 0.5*dOmega_hat ; // Appears to agree with equations in papers
	Eigen::Vector3d dOmega_hat_p_ROC = Eigen::Vector3d::Zero() ;
	Eigen::Vector3d dQuat_hat_p = dQuat_hat + dQuat_hat_p_ROC*estimator_parameters_.dt ;
	Eigen::Vector3d dOmega_hat_p = dOmega_hat + dOmega_hat_p_ROC*estimator_parameters_.dt ;
	// Writing to apriori error state
	*dx_p << dQuat_hat_p, dOmega_hat_p ;
	// Writing to the message object
  msg->dQuat_p.x = dQuat_hat_p.x() ;
  msg->dQuat_p.y = dQuat_hat_p.y() ;
  msg->dQuat_p.z = dQuat_hat_p.z() ;
  msg->dOmega_p.x = dOmega_hat_p.x() ;
  msg->dOmega_p.y = dOmega_hat_p.y() ;
  msg->dOmega_p.z = dOmega_hat_p.z() ;
}

void RotationalEstimator::updateEstimate_propagateErrorCovariance(Eigen::Matrix<double, 6, 6>* P_p, Eigen::Matrix<double, 6, 6>& P_old, const Eigen::Matrix<double, 7, 1>& x_old,
																																	ros_vrpn_client::rotationalEstimator* msg)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	// Constructing linearized system matrices
  Eigen::Matrix<double, 6, 6> A;
	Eigen::Matrix<double, 6, 6> L;
	A << skewMatrix(omega_hat), 0.5*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() ; // Should be accordance with the hamilton definition
	L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() ;
	// Performing propagation
	*P_p = P_old + ( A*P_old + P_old*A + L*processCovariance*L.transpose() )*estimator_parameters_.dt ;
}

void RotationalEstimator::updateEstimate_updateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_m, Eigen::Matrix<double, 6, 6>* P_m, const Eigen::Quaterniond& quat_measured,
																														 const Eigen::Matrix<double, 7, 1>& x_p, const Eigen::Matrix<double, 6, 1>& dx_p, const Eigen::Matrix<double, 6,6>& P_p,
																														 ros_vrpn_client::rotationalEstimator* msg)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat_p = Eigen::Quaterniond(x_p.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat_p = x_p.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat_p = dx_p.block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat_p = dx_p.block<3,1>(3, 0) ;
	// Constructing linearized measurement matrix
	Eigen::Matrix<double, 4, 3>  H_dq ;
	Eigen::Matrix<double, 4, 6>  H ;
	H_dq << -quat_hat_p.vec().transpose(),
					quat_hat_p.w()*Eigen::Matrix<double, 3, 3>::Identity() + skewMatrix(quat_hat_p.vec()) ;
	H << H_dq, Eigen::Matrix<double, 4, 3>::Zero() ;
	// Predicting the measurement
	Eigen::Quaterniond quat_predicted = Eigen::Quaterniond(H_dq*dQuat_hat_p + quat_hat_p.coeffs()) ;
	// Computing the Kalman gain
	Eigen::Matrix<double, 6, 4> K ;
	K = P_p*H.transpose()*(H*P_p*H.transpose() + measurementCovariance).inverse() ;
	// Correcting the state
	*dx_m = dx_p + K*(quat_measured.coeffs() - quat_predicted.coeffs()) ; // TODO: Everything is going to be the wrong way around here
	// Updating the covariance
	*P_m = (Eigen::Matrix<double, 6, 6>::Identity() - K*H)*P_p ;
	// Seperating the state parts
	Eigen::Matrix<double, 3, 1> dQuat_hat_m = dx_m->block<3,1>(0,0);
	Eigen::Matrix<double, 3, 1> dOmega_hat_m = dx_m->block<3,1>(3,0);
	// Writing to the message object
  msg->dQuat_m.x = dQuat_hat_m.x() ;
  msg->dQuat_m.y = dQuat_hat_m.y() ;
  msg->dQuat_m.z = dQuat_hat_m.z() ;
  msg->dOmega_m.x = dOmega_hat_m.x() ;
  msg->dOmega_m.y = dOmega_hat_m.y() ;
  msg->dOmega_m.z = dOmega_hat_m.z() ;
}

void RotationalEstimator::updateEstimate_recombineErrorGlobal(Eigen::Matrix<double, 7, 1>* x_m, Eigen::Matrix<double, 6, 1>* dx_m, const Eigen::Matrix<double, 7, 1> x_p,
																															ros_vrpn_client::rotationalEstimator* msg)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat_p = Eigen::Quaterniond(x_p.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat_p = x_p.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat_m = dx_m->block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat_m = dx_m->block<3,1>(3, 0) ;
	// Completing the error quaternion
	Eigen::Quaterniond dQuat_hat_m_q = Eigen::Quaterniond(1.0, dQuat_hat_m.x(), dQuat_hat_m.y(), dQuat_hat_m.z()) ;
	// Correction
	Eigen::Quaterniond quat_hat_m = dQuat_hat_m_q*quat_hat_p ;
	Eigen::Matrix<double, 3, 1> omega_hat_m = omega_hat_p + dOmega_hat_m;
	// Normalizing the posteriori quaternion
	quat_hat_m.normalize() ;
	// Writing to posteriori global state and error states
	*x_m << quat_hat_m.coeffs(), omega_hat_m ;
	*dx_m = Eigen::Matrix<double, 6, 1>::Zero() ;
	// Writing to the message object
  msg->quat_m.x = quat_hat_m.x() ;
  msg->quat_m.y = quat_hat_m.y() ;
  msg->quat_m.z = quat_hat_m.z() ;
  msg->omega_m.x = omega_hat_m.x() ;
  msg->omega_m.y = omega_hat_m.y() ;
  msg->omega_m.z = omega_hat_m.z() ;
}



}