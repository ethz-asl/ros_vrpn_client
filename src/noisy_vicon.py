#!/usr/bin/env python

import rospy
import random
from math import *

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, TransformStamped
from std_msgs.msg import Float64
import tf
import sys

class NoisyVicon:
  def __init__(self):
    self.mav_name = rospy.get_param('~mav_name', 'mav')
    # Altitude source ('vicon'/'external')
    self.altitude_input = rospy.get_param('~altitude_input', 'vicon')
    # Whether to publish pose message or not (True/False)
    self.publish_pose = rospy.get_param('~publish_pose', True)
    # Whether to publish transform message or not (True/False)
    self.publish_transform = rospy.get_param('~publish_transform', True)
    # Maximum noise radius (polar coordinates) [m]
    self.max_noise_radius = rospy.get_param('~max_noise_radius', 0.0) # max tested: 0.15
    # Timeout between noise sampling updates [s]
    self.publish_timeout = rospy.get_param('~publish_timeout', 0.2)

    self.fix = NavSatFix()
    self.fix.header.frame_id = 'fcu'
    self.fix.status.status = 1
    self.fix.status.service = 1
    self.fix.latitude = 44.0
    self.fix.longitude = 44.0
    self.fix.altitude = 0.0
    # TODO: fill GPS covariance

    self.latest_altitude_message = Float64()
    self.latest_imu_message = Imu()

    self.pwc = PoseWithCovarianceStamped()
    self.pwc.header.frame_id = 'vicon' # doesn't really matter to MSF
    self.pwc.pose.covariance[6 * 0 + 0] = 0.000025;
    self.pwc.pose.covariance[6 * 1 + 1] = 0.000025;
    self.pwc.pose.covariance[6 * 2 + 2] = 0.000025;
    self.pwc.pose.covariance[6 * 3 + 3] = 0.000025;
    self.pwc.pose.covariance[6 * 4 + 4] = 0.000025;
    self.pwc.pose.covariance[6 * 5 + 5] = 0.000025;

    self.point = PointStamped()
    self.point.header = self.pwc.header

    self.transform = TransformStamped()
    self.transform.header = self.pwc.header
    self.child_frame_id = self.mav_name + '_noisy'

    self.R_noise = 0.0
    self.theta_noise = 0.0
    self.timer_pub = None
    self.timer_resample = None
    self.got_odometry = False

    self.spoofed_gps_pub = rospy.Publisher('spoofed_gps', NavSatFix, queue_size=1)
    self.pose_pub = rospy.Publisher('noisy_vicon_pose', PoseWithCovarianceStamped, queue_size=1,  tcp_nodelay=True)
    self.point_pub = rospy.Publisher('noisy_vicon_point', PointStamped, queue_size=1,  tcp_nodelay=True)
    self.transform_pub = rospy.Publisher('noisy_vicon_transform', TransformStamped, queue_size=1,  tcp_nodelay=True)

    self.altitude_sub = rospy.Subscriber('laser_altitude', Float64, self.altitude_callback, tcp_nodelay=True)
    self.gps_sub = rospy.Subscriber('gps', NavSatFix, self.gps_callback, tcp_nodelay=True)
    self.estimated_odometry_sub = rospy.Subscriber('estimated_odometry', Odometry, self.odometry_callback, queue_size=1, tcp_nodelay=True)
    self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=1, tcp_nodelay=True)

  def gps_callback(self, data):
    self.fix.header.stamp = data.header.stamp
    self.spoofed_gps_pub.publish(self.fix)

  def odometry_callback(self, data):
    #print "Got odometry!"
    #print "Max noise radius = ", self.max_noise_radius
    self.pwc.header.stamp = rospy.Time.now()
    self.point.header = self.pwc.header

    # Take x and y from vicon
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    self.pwc.pose.pose.position.x = x
    self.pwc.pose.pose.position.y = y
    self.point.point.x = x
    self.point.point.y = y
    self.transform.transform.translation.x = x
    self.transform.transform.translation.y = y

    # Take z based on chosen altitude input
    if (self.altitude_input == 'vicon'):
        self.pwc.pose.pose.position.z = data.pose.pose.position.z
        self.point.point.z = data.pose.pose.position.z
        self.transform.transform.translation.z = data.pose.pose.position.z
    elif (self.altitude_input == 'external'):
        self.pwc.pose.pose.position.z = self.latest_altitude_message.data
        self.point.point.z = self.latest_altitude_message.data
        self.transform.transform.translation.z = self.latest_altitude_message.data
    else:
        rospy.signal_shutdown("Unknown altitude input parameter")
        sys.exit()

    # Take orientation from IMU (pose only)
    self.pwc.pose.pose.orientation = self.latest_imu_message.orientation
    self.transform.transform.rotation = self.latest_imu_message.orientation

    if not self.got_odometry:
        print "NoisyVicon: initializing timers"
        self.got_odometry = True
        self.timer_resample = rospy.Timer(rospy.Duration(5), self.sample_noise)
        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_timeout), self.publish_noisy_positions)

  def imu_callback(self, data):
    self.latest_imu_message = data

  def altitude_callback(self, data):
    self.latest_altitude_message = data

  def sample_noise(self, event):
    #print "Resampling noise"
    # Generate noise in x and y
    self.R_noise = random.uniform(0, self.max_noise_radius)
    self.theta_noise = random.uniform(0, 2*pi)

  def publish_noisy_positions(self, event):
    #print "Publishing odometry"
    self.pwc.pose.pose.position.x += self.R_noise*cos(self.theta_noise)
    self.pwc.pose.pose.position.y += self.R_noise*sin(self.theta_noise)
    self.point.point.x += self.R_noise*cos(self.theta_noise)
    self.point.point.y += self.R_noise*sin(self.theta_noise)

    if (self.publish_pose):
        self.pose_pub.publish(self.pwc)
    if (self.publish_transform):
        self.transform_pub.publish(self.transform)
    self.point_pub.publish(self.point)

if __name__ == '__main__':

  try:
    rospy.init_node('noisy_vicon', anonymous=True)
    gs = NoisyVicon()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
