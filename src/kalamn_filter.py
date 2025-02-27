#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(object):

    def __init__(self):

        self.odom_sub_ = rospy.Subscriber("mobilebot_controller/odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallabck)
        self.odom_pub_ = rospy.Publisher("mobilebot_controller/odom_kalman", Odometry, queue_size=10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.is_first_odom_ = True
        self.angular_z_ = 0.0
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0
        self.kalman_odom_ = Odometry()
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

    def imuCallabck(self, imu):
        self.angular_z_ = imu.angular_velocity.z

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            return

        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpadte()

        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)

        self.last_angular_z_ = odom.twist.twist.angular.z

    def measurementUpadte(self):

        self.mean_ = (self.measurement_variance_*self.mean_ + self.variance_*self.angular_z_) / (self.measurement_variance_ + self.variance_)
        self.variance_ = (self.measurement_variance_ * self. variance_) / (self.measurement_variance_ + self.variance_)

    def statePrediction(self):

        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self. motion_variance_



if __name__ == "__main__":
    rospy.init_node('kalman_filter_node')
    filter = KalmanFilter()
    rospy.spin()