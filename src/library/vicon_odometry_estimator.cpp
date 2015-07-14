
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
}

void RotationalEstimator::updateEstimate(const Eigen::Quaterniond& quat_measured)
{
	// Propagating the global state estimate
	Eigen::Matrix<double, 7, 1> x_old ;
	Eigen::Matrix<double, 7, 1> x_p ;
	x_old << quat_hat.coeffs(), omega_hat ;
	updateEstimate_propagateGlobalEstimate(&x_p, x_old) ;
	
	// Propagating the error state estimate
	Eigen::Matrix<double, 6, 1> dx_old ;
	Eigen::Matrix<double, 6, 1> dx_p ;
	dx_old << dQuat_hat, dOmega_hat ;
	updateEstimate_propagateErrorEstimate(&dx_p, dx_old, x_old);

	// Propagating the estimate covariance
	Eigen::Matrix<double, 6, 6> P_old ;
	Eigen::Matrix<double, 6, 6> P_p ;
	P_old = covariance ;
	updateEstimate_propagateErrorCovariance(&P_p, P_old, x_old);

	// Measurement Update
	Eigen::Matrix<double, 6, 1> dx_m ;
	Eigen::Matrix<double, 6, 6> P_m ;
	updateEstimate_updateErrorEstimate(&dx_m, &P_m, quat_measured, x_p, dx_p, P_p);

	// Global state correction
	Eigen::Matrix<double, 7, 1> x_m ;
	updateEstimate_recombineErrorGlobal(&x_m, &dx_m, x_p);

	// Extracting estimated quantities from the posteriori state
	quat_hat = Eigen::Quaterniond(x_m.block<4,1>(0,0));
	omega_hat = x_m.block<3,1>(3,0);
	covariance = P_m;
}

Eigen::Quaterniond RotationalEstimator::getEstimatedOrientation() const {
  return quat_hat ;
}  

Eigen::Vector3d RotationalEstimator::getEstimatedAngularVelocity() const {
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

void RotationalEstimator::updateEstimate_propagateGlobalEstimate(Eigen::Matrix<double, 7, 1>* x_p, const Eigen::Matrix<double, 7, 1>& x_old)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	// Performing propagation
	Eigen::Quaterniond omega_hat_q = Eigen::Quaterniond(0, omega_hat.x(), omega_hat.y(), omega_hat.z());
	Eigen::Quaterniond quat_hat_p_ROC = Eigen::Quaterniond(0.5*(omega_hat_q*quat_hat).coeffs());
	Eigen::Vector3d omega_hat_p_ROC = Eigen::Vector3d::Zero();
	Eigen::Quaterniond quat_hat_p = Eigen::Quaterniond(quat_hat.coeffs() + quat_hat_p_ROC.coeffs()*estimator_parameters_.dt);
	Eigen::Vector3d omega_hat_p = omega_hat + omega_hat_p_ROC*estimator_parameters_.dt;
	// Writing to apriori state
	*x_p << quat_hat_p.coeffs(), omega_hat_p ;
}

void RotationalEstimator::updateEstimate_propagateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_p, const Eigen::Matrix<double, 6, 1>& dx_old, const Eigen::Matrix<double, 7, 1>& x_old)
{
	// Extracting components of the states
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat = dx_old.block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat = dx_old.block<3,1>(3, 0) ;
	// Performing propagation
	Eigen::Vector3d dQuat_hat_p_ROC = -omega_hat.cross(dQuat_hat) + 0.5*dOmega_hat ;
	Eigen::Vector3d dOmega_hat_p_ROC = Eigen::Vector3d::Zero() ;
	Eigen::Vector3d dQuat_hat_p = dQuat_hat + dQuat_hat_p_ROC*estimator_parameters_.dt ;
	Eigen::Vector3d dOmega_hat_p = dOmega_hat + dOmega_hat_p_ROC*estimator_parameters_.dt ;
	// Writing to apriori error state
	*dx_p << dQuat_hat_p, dOmega_hat_p ;
}

void RotationalEstimator::updateEstimate_propagateErrorCovariance(Eigen::Matrix<double, 6, 6>* P_p, Eigen::Matrix<double, 6, 6>& P_old, const Eigen::Matrix<double, 7, 1>& x_old)
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat = x_old.block<3,1>(4, 0) ;
	// Constructing linearized system matrices
  Eigen::Matrix<double, 6, 6> A;
	Eigen::Matrix<double, 6, 6> L;
	A << skewMatrix(omega_hat), 0.5*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() ;
	L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() ;
	// Performing propagation
	*P_p = covariance + ( A*P_old + P_old*A + L*processCovariance*L.transpose() )*estimator_parameters_.dt ;
}

void RotationalEstimator::updateEstimate_updateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_m, Eigen::Matrix<double, 6, 6>* P_m, const Eigen::Quaterniond& quat_measured,
																														 const Eigen::Matrix<double, 7, 1>& x_p, const Eigen::Matrix<double, 6, 1>& dx_p, const Eigen::Matrix<double, 6,6>& P_p)
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
}

void RotationalEstimator::updateEstimate_recombineErrorGlobal(Eigen::Matrix<double, 7, 1>* x_m, Eigen::Matrix<double, 6, 1>* dx_m, const Eigen::Matrix<double, 7, 1> x_p)
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
}



}