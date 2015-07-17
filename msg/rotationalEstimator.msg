Header header

geometry_msgs/Quaternion quat_measured 	# the measured body orientation

geometry_msgs/Quaternion quat_old 			# the old body orientation
geometry_msgs/Vector3 omega_old  				# the old body rate

geometry_msgs/Vector3 dQuat_old  				# the old body orientation error
geometry_msgs/Vector3 dOmega_old  			# the old body rate error

geometry_msgs/Quaternion quat_p 				# the apriori body orientation
geometry_msgs/Vector3 omega_p  					# the apriori body rate

geometry_msgs/Vector3 dQuat_p  					# the apriori body orientation error
geometry_msgs/Vector3 dOmega_p  				# the apriori body rate error

geometry_msgs/Quaternion quat_m 				# the posteriori body orientation
geometry_msgs/Vector3 omega_m  					# the posteriori body rate

geometry_msgs/Vector3 dQuat_m  					# the posteriori body orientation error
geometry_msgs/Vector3 dOmega_m  				# the posteriori body rate error