Header header

geometry_msgs/Vector3 		pos_measured 	# the measured body position
geometry_msgs/Vector3 		pos_old 			# the old body position
geometry_msgs/Vector3 		vel_old  			# the old body velocity
geometry_msgs/Vector3 		pos_est				# the posteriori body position
geometry_msgs/Vector3 		vel_est				# the posteriori body velocity

geometry_msgs/Quaternion 	quat_measured # the measured body orientation
geometry_msgs/Quaternion  quat_measured_corrected # the measured body orientation
geometry_msgs/Quaternion 	quat_old 			# the old body orientation
geometry_msgs/Vector3 		omega_old  		# the old body rate
geometry_msgs/Quaternion 	quat_est			# the posteriori body orientation
geometry_msgs/Vector3 		omega_est			# the posteriori body rate

std_msgs/Bool             outlier_flag  # Flag indicating if the measurement at this timestep was detected as being an outlier
std_msgs/Bool             measurement_flip_flag  # Flag indicating if the measurement from vicon has undergone a redundant flipping.

geometry_msgs/Quaternion  q_Z_Z1        # the measurement to last measurement error quaternion
std_msgs/Float64          q_Z_Z1_magnitude # The magnitude of the messurement error

geometry_msgs/Quaternion  q_Z_B         # the measurement to body error quaternion
std_msgs/Float64          q_Z_B_magnitude # The magnitude of the messurement error
