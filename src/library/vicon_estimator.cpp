
#include "vicon_estimator.h"

//namespace vicon_estimation {

/*
* --------------------------------------------------------------------
* Vicon Odometry Estimator
* --------------------------------------------------------------------
*/

ViconEstimator::ViconEstimator() : //ros::NodeHandle& nh
  translationalEstimator_(),
  rotationalEstimator_()
{

}

void ViconEstimator::updateEstimate(const Eigen::Vector3d& pos_measured, const Eigen::Quaterniond& quat_measured)
{
	// Updating the translational and rotation sub-estimates
	translationalEstimator_.updateEstimate(pos_measured);
	rotationalEstimator_.updateEstimate(quat_measured);
}

void ViconEstimator::reset()
{
	// Resetting the translational and rotation
	translationalEstimator_.reset();
	rotationalEstimator_.reset();
}

void ViconEstimator::setParameters(const TranslationalEstimatorParameters& translationalEstimatorParameters, const RotationalEstimatorParameters& rotationalEstimatorParameters)
{
	translationalEstimator_.setParameters(translationalEstimatorParameters);
	rotationalEstimator_.setParameters(rotationalEstimatorParameters);
}

void ViconEstimator::getIntermediateResults(TranslationalEstimatorResults* translationalEstimatorResults, RotationalEstimatorResults* rotationalEstimatorResults)
{
	*translationalEstimatorResults = translationalEstimator_.getResults();
	*rotationalEstimatorResults = rotationalEstimator_.getResults();
}


/*
* --------------------------------------------------------------------
* Translational Estimator
* --------------------------------------------------------------------
*/


TranslationalEstimator::TranslationalEstimator() : //ros::NodeHandle& nh 
  pos_hat_(0.0, 0.0, 0.0),
  vel_hat_(0.0, 0.0, 0.0),
  estimator_parameters_(),
  estimator_results_()
{

}

void TranslationalEstimator::updateEstimate(const Eigen::Vector3d& pos_measured)
{
	// Saving the measurement to the intermediate results
	estimator_results_.pos_measured = pos_measured;
	// Saving the old state to the intermediate results
	estimator_results_.pos_old = pos_hat_;
	estimator_results_.vel_old = vel_hat_;
  // Constructing the full state
  Eigen::Matrix<double, 6, 1> x_hat;
  x_hat << pos_hat_, vel_hat_;
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
  pos_hat_ = x_hat.block<3,1>(0, 0) ;
  vel_hat_ = x_hat.block<3,1>(3, 0) ;
  // Saving estimate to intermediate results
  estimator_results_.pos_est = pos_hat_;
  estimator_results_.vel_est = vel_hat_;
}

void TranslationalEstimator::reset()
{
  pos_hat_ = Eigen::Vector3d::Zero() ;
  vel_hat_ = Eigen::Vector3d::Zero() ;
}

void TranslationalEstimator::setParameters(const TranslationalEstimatorParameters& translationalEstimatorParameters)
{
	estimator_parameters_.kp = translationalEstimatorParameters.kp;
	estimator_parameters_.kv = translationalEstimatorParameters.kv;
}

/*
* --------------------------------------------------------------------
* Rotational Estimator
* --------------------------------------------------------------------
*/

RotationalEstimator::RotationalEstimator() : //ros::NodeHandle& nh
  quat_hat_(1.0, 0.0, 0.0, 0.0),
  omega_hat_(0.0, 0.0, 0.0),
  dQuat_hat_(0.0, 0.0, 0.0),
  dOmega_hat_(0.0, 0.0, 0.0),
  estimator_parameters_(),
  estimator_results_()
{
	// Initializing the initial covariance
	covariance_ << estimator_parameters_.dQuat_hat_initialCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
								Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_hat_initialCovariance*Eigen::Matrix3d::Identity() ;
	// Constructing process and measurement covariance matrices
	processCovariance_ << 	estimator_parameters_.dQuat_processCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
												Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_processCovariance*Eigen::Matrix3d::Identity() ;
	measurementCovariance_ << 	estimator_parameters_.quat_measurementCovariance*Eigen::Matrix4d::Identity() ;
}

void RotationalEstimator::reset()
{
	quat_hat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0) ;
  omega_hat_ = Eigen::Matrix<double, 3, 1>::Zero() ;
  dQuat_hat_ = Eigen::Matrix<double, 3, 1>::Zero() ;
  dOmega_hat_ = Eigen::Matrix<double, 3, 1>::Zero() ;
  // Reseting to initial covariance
	covariance_ << estimator_parameters_.dQuat_hat_initialCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
							Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_hat_initialCovariance*Eigen::Matrix3d::Identity() ;
	// Constructing process and measurement covariance matrices
	processCovariance_ << 	estimator_parameters_.dQuat_processCovariance*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
												Eigen::Matrix3d::Zero(), estimator_parameters_.dOmega_processCovariance*Eigen::Matrix3d::Identity() ;
	measurementCovariance_ << 	estimator_parameters_.quat_measurementCovariance*Eigen::Matrix4d::Identity() ;
}

void RotationalEstimator::setParameters(const RotationalEstimatorParameters& rotationalEstimatorParameters)
{
	estimator_parameters_.dt = rotationalEstimatorParameters.dt ;
	estimator_parameters_.dQuat_hat_initialCovariance = rotationalEstimatorParameters.dQuat_hat_initialCovariance ;
	estimator_parameters_.dOmega_hat_initialCovariance = rotationalEstimatorParameters.dOmega_hat_initialCovariance ;
	estimator_parameters_.dQuat_processCovariance = rotationalEstimatorParameters.dQuat_processCovariance ;
	estimator_parameters_.dOmega_processCovariance = rotationalEstimatorParameters.dOmega_processCovariance ;
	estimator_parameters_.quat_measurementCovariance = rotationalEstimatorParameters.quat_measurementCovariance ;
}

void RotationalEstimator::updateEstimate(const Eigen::Quaterniond& quat_measured)
{ 
	// Writing the raw measurement to the intermediate results structure
	estimator_results_.quat_measured = quat_measured ;

	// Writing the old estimate to the intermediate results structure
	estimator_results_.quat_old = quat_hat_ ;
	estimator_results_.omega_old = omega_hat_ ;

	// Propagating the global state estimate
	Eigen::Matrix<double, 7, 1> x_old ;
	Eigen::Matrix<double, 7, 1> x_p ;
	x_old << quat_hat_.coeffs(), omega_hat_ ;
	updateEstimate_propagateGlobalEstimate(&x_p, x_old) ; //&msg

	// Propagating the error state estimate
	Eigen::Matrix<double, 6, 1> dx_old ;
	Eigen::Matrix<double, 6, 1> dx_p ;
	dx_old << dQuat_hat_, dOmega_hat_ ;
	updateEstimate_propagateErrorEstimate(&dx_p, dx_old, x_old); //&msg

	// Propagating the estimate covariance
	Eigen::Matrix<double, 6, 6> P_old ;
	Eigen::Matrix<double, 6, 6> P_p ;
	P_old = covariance_ ;
	updateEstimate_propagateErrorCovariance(&P_p, P_old, x_old); //&msg

	// Measurement Update
	Eigen::Matrix<double, 6, 1> dx_m ;
	Eigen::Matrix<double, 6, 6> P_m ;
	updateEstimate_updateErrorEstimate(&dx_m, &P_m, quat_measured, x_p, dx_p, P_p); //&msg

	// Global state correction
	Eigen::Matrix<double, 7, 1> x_m ;
	updateEstimate_recombineErrorGlobal(&x_m, &dx_m, x_p); //&msg

	// Extracting estimated quantities from the posteriori state
	quat_hat_ = Eigen::Quaterniond(x_m.block<4,1>(0,0));
	omega_hat_ = x_m.block<3,1>(4,0);
	covariance_ = P_m;

	// Writing the old estimate to the intermediate results structure
	estimator_results_.quat_est = quat_hat_ ;
	estimator_results_.omega_est = omega_hat_ ;

}

Eigen::Matrix3d RotationalEstimator::skewMatrix(const Eigen::Vector3d& vec ) const
{
	Eigen::Matrix3d vec_cross;
	vec_cross << 	0, 				-vec(2), 	vec(1),
    						vec(2), 	0, 				-vec(0),
    						-vec(1), 	vec(0), 	0;
	return vec_cross;
}

void RotationalEstimator::updateEstimate_propagateGlobalEstimate(Eigen::Matrix<double, 7, 1>* x_p, const Eigen::Matrix<double, 7, 1>& x_old) //ros_vrpn_client::rotationalEstimator* msg
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat_ = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat_ = x_old.block<3,1>(4, 0) ;
	// Performing propagation
	Eigen::Quaterniond omega_hat__q = Eigen::Quaterniond(0, omega_hat_.x(), omega_hat_.y(), omega_hat_.z());
	Eigen::Quaterniond quat_hat__p_ROC = Eigen::Quaterniond(0.5*(quat_hat_*omega_hat__q).coeffs()); // Using hamilton definition
	Eigen::Vector3d omega_hat__p_ROC = Eigen::Vector3d::Zero();
	Eigen::Quaterniond quat_hat__p = Eigen::Quaterniond(quat_hat_.coeffs() + quat_hat__p_ROC.coeffs()*estimator_parameters_.dt);
	Eigen::Vector3d omega_hat__p = omega_hat_ + omega_hat__p_ROC*estimator_parameters_.dt;
	// Writing to apriori state
	*x_p << quat_hat__p.coeffs(), omega_hat__p ;
	// Saving the intermediate results
	//estimator_results_.quat_p = quat_hat_p ;
	//estimator_results_.omega_p = omega_hat_p ;
}

void RotationalEstimator::updateEstimate_propagateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_p, const Eigen::Matrix<double, 6, 1>& dx_old, const Eigen::Matrix<double, 7, 1>& x_old) //ros_vrpn_client::rotationalEstimator* msg
{
	// Extracting components of the states
	Eigen::Quaterniond quat_hat_ = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat_ = x_old.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat_ = dx_old.block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat_ = dx_old.block<3,1>(3, 0) ;
	// Performing propagation
	Eigen::Vector3d dQuat_hat__p_ROC = -omega_hat_.cross(dQuat_hat_) + 0.5*dOmega_hat_ ; // Appears to agree with equations in papers
	Eigen::Vector3d dOmega_hat__p_ROC = Eigen::Vector3d::Zero() ;
	Eigen::Vector3d dQuat_hat__p = dQuat_hat_ + dQuat_hat__p_ROC*estimator_parameters_.dt ;
	Eigen::Vector3d dOmega_hat__p = dOmega_hat_ + dOmega_hat__p_ROC*estimator_parameters_.dt ;
	// Writing to apriori error state
	*dx_p << dQuat_hat__p, dOmega_hat__p ;
	// Saving the intermediate results
	//estimator_results_.dQuat_p = dQuat_hat_p ;
	//estimator_results_.dOmega_p = dOmega_hat_p ;
}

void RotationalEstimator::updateEstimate_propagateErrorCovariance(Eigen::Matrix<double, 6, 6>* P_p, Eigen::Matrix<double, 6, 6>& P_old, const Eigen::Matrix<double, 7, 1>& x_old) //ros_vrpn_client::rotationalEstimator* msg
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat_ = Eigen::Quaterniond(x_old.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat_ = x_old.block<3,1>(4, 0) ;
	// Constructing linearized system matrices
  Eigen::Matrix<double, 6, 6> A;
	Eigen::Matrix<double, 6, 6> L;
	A << -1*skewMatrix(omega_hat_), 0.5*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero() ; // Should be accordance with the hamilton definition
	L << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() ;
	// Performing propagation
	*P_p = P_old + ( A*P_old + P_old*A.transpose() + L*processCovariance_*(L.transpose()) )*estimator_parameters_.dt ;
	// Saving the intermediate results
}

void RotationalEstimator::updateEstimate_updateErrorEstimate(Eigen::Matrix<double, 6, 1>* dx_m, Eigen::Matrix<double, 6, 6>* P_m, const Eigen::Quaterniond& quat_measured,
																														 const Eigen::Matrix<double, 7, 1>& x_p, const Eigen::Matrix<double, 6, 1>& dx_p, const Eigen::Matrix<double, 6,6>& P_p) //ros_vrpn_client::rotationalEstimator* msg
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat__p = Eigen::Quaterniond(x_p.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat__p = x_p.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat__p = dx_p.block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat__p = dx_p.block<3,1>(3, 0) ;
	// Constructing linearized measurement matrix
	Eigen::Matrix<double, 4, 3>  H_dq ;
	Eigen::Matrix<double, 4, 6>  H ;
	H_dq << quat_hat__p.w()*Eigen::Matrix<double, 3, 3>::Identity() + skewMatrix(quat_hat__p.vec()), 
					-quat_hat__p.vec().transpose() ;
	H << H_dq, Eigen::Matrix<double, 4, 3>::Zero() ;
	// Predicting the measurement
	Eigen::Quaterniond quat_predicted = Eigen::Quaterniond(H_dq*dQuat_hat__p + quat_hat__p.coeffs()) ; //shit around the wrong way
	// Computing the Kalman gain
	Eigen::Matrix<double, 6, 4> K ;
	K = P_p*H.transpose()*((H*P_p*H.transpose() + measurementCovariance_).inverse()) ;
	// Correcting the state
	*dx_m = dx_p + K*(quat_measured.coeffs() - quat_predicted.coeffs()) ; // TODO: Everything is going to be the wrong way around here
	// Updating the covariance
	*P_m = (Eigen::Matrix<double, 6, 6>::Identity() - K*H)*P_p ;
	//*P_m = (Eigen::Matrix<double, 6, 6>::Identity() - K*H)*P_p*((Eigen::Matrix<double, 6, 6>::Identity() - K*H).transpose()) + K*measurementCovariance*K.transpose() ;
	// Seperating the state parts
	Eigen::Matrix<double, 3, 1> dQuat_hat__m = dx_m->block<3,1>(0,0);
	Eigen::Matrix<double, 3, 1> dOmega_hat__m = dx_m->block<3,1>(3,0);
	// Saving the intermediate results
	//estimator_results_.dQuat_m = dQuat_hat_m ;
	//estimator_results_.dOmega_m = dOmega_hat_m ;
}

void RotationalEstimator::updateEstimate_recombineErrorGlobal(Eigen::Matrix<double, 7, 1>* x_m, Eigen::Matrix<double, 6, 1>* dx_m, const Eigen::Matrix<double, 7, 1> x_p)//ros_vrpn_client::rotationalEstimator* msg
{
	// Extracting components of the state
	Eigen::Quaterniond quat_hat__p = Eigen::Quaterniond(x_p.block<4,1>(0, 0)) ;
	Eigen::Matrix<double, 3, 1> omega_hat__p = x_p.block<3,1>(4, 0) ;
	Eigen::Matrix<double, 3, 1> dQuat_hat__m = dx_m->block<3,1>(0, 0) ;
	Eigen::Matrix<double, 3, 1> dOmega_hat__m = dx_m->block<3,1>(3, 0) ;
	// Completing the error quaternion
	Eigen::Quaterniond dQuat_hat__m_q = Eigen::Quaterniond(1.0, dQuat_hat__m.x(), dQuat_hat__m.y(), dQuat_hat__m.z()) ;
	// Correction
	Eigen::Quaterniond quat_hat__m = quat_hat__p*dQuat_hat__m_q ; // Hamilton definition
	Eigen::Matrix<double, 3, 1> omega_hat__m = omega_hat__p + dOmega_hat__m;
	// Normalizing the posteriori quaternion
	quat_hat__m.normalize() ;
	// Writing to posteriori global state and error states
	*x_m << quat_hat__m.coeffs(), omega_hat__m ;
	*dx_m = Eigen::Matrix<double, 6, 1>::Zero() ;
	// Saving the intermediate results
	//estimator_results_.quat_m = quat_hat_m ;
	//estimator_results_.omega_m = omega_hat_m ;
}



//}