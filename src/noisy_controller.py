#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import math
from nav_msgs.msg import Odometry
import tf_conversions
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class NoisyController(object):

    def __init__(self, wheel_rad, wheel_sep):

      rospy.loginfo("The wheel radius is %f" % wheel_rad)
      rospy.loginfo("The wheel seperation is %f" % wheel_sep)

      self.wheel_rad_ = wheel_rad
      self.wheel_sep_ = wheel_sep
      self.left_prev_pos_ = 0.0
      self.right_prev_pos_ = 0.0
      self.prev_time_ = rospy.Time.now()

      self.x_ = 0.0
      self.y_ = 0.0
      self.theta_ = 0.0
      self.odom_msg_ = Odometry()
      self.odom_msg_.header.frame_id = "odom"
      self.odom_msg_.child_frame_id = "base_footprint_efl"
      self.odom_msg_.pose.pose.orientation.x = 0
      self.odom_msg_.pose.pose.orientation.y = 0
      self.odom_msg_.pose.pose.orientation.z = 0
      self.odom_msg_.pose.pose.orientation.w = 1

      self.br_ = TransformBroadcaster()
      self.transform_stamped_ = TransformStamped()
      self.transform_stamped_.header.frame_id = 'odom'
      self.transform_stamped_.child_frame_id = "base_footprint_noisy"

      self.odom_pub_ = rospy.Publisher("mobilebot_controller/odom_noisy", Odometry, queue_size=10)

      self.joint_pos = rospy.Subscriber("joint_states", JointState, self.jointcallback)


    def jointcallback(self, msg):

      wheel_left_noise = msg.position[0] + np.random.normal(0, 0.005)
      wheel_right_noise = msg.position[1] + np.random.normal(0, 0.005)
      dp_left = wheel_left_noise - self.left_prev_pos_
      dp_right = wheel_right_noise - self.right_prev_pos_
      dt = (msg.header.stamp - self.prev_time_).to_sec()

      self.left_prev_pos_ = msg.position[0]
      self.right_prev_pos_ = msg.position[1]
      self.prev_time_ = msg.header.stamp

      fi_left = dp_left / dt
      fi_right = dp_right / dt

      linear = (self.wheel_rad_*fi_right + self.wheel_rad_*fi_left) / 2
      angular = (self.wheel_rad_*fi_right - self.wheel_rad_*fi_left) / self.wheel_sep_

      d_s = (self.wheel_rad_*dp_right + self.wheel_rad_*dp_left) / 2
      d_theta = (self.wheel_rad_*dp_right - self.wheel_rad_*dp_left) / self.wheel_sep_

      self.theta_ += d_theta
      self.x_ += d_s * math.cos(self.theta_)
      self.y_ += d_s * math.sin(self.theta_)

      q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_)
      self.odom_msg_.pose.pose.orientation.x = q[0]
      self.odom_msg_.pose.pose.orientation.y = q[1]
      self.odom_msg_.pose.pose.orientation.z = q[2]
      self.odom_msg_.pose.pose.orientation.w = q[3]
      self.odom_msg_.header.stamp = rospy.Time.now()
      self.odom_msg_.pose.pose.position.x = self.x_
      self.odom_msg_.pose.pose.position.y = self.y_
      self.odom_msg_.twist.twist.linear.x = linear
      self.odom_msg_.twist.twist.angular.z = angular

      self.odom_pub_.publish(self.odom_msg_)

      self.transform_stamped_.transform.translation.x = self.x_
      self.transform_stamped_.transform.translation.y = self.y_
      self.transform_stamped_.transform.rotation.x = q[0]
      self.transform_stamped_.transform.rotation.y = q[1]
      self.transform_stamped_.transform.rotation.z = q[2]
      self.transform_stamped_.transform.rotation.w = q[3]
      self.transform_stamped_.header.stamp = rospy.Time.now()

      self.br_.sendTransform(self.transform_stamped_)


      #rospy.loginfo("Linear: %f  Angular: %f", linear, angular)
      #rospy.loginfo("X: %f" %self.x_)
      #rospy.loginfo("Y: %f" %self.y_)
      #rospy.loginfo("Theta: %f" %self.theta_)





if __name__ == "__main__":
    
    rospy.init_node('Noisy_controller')

    wheel_rad = rospy.get_param('~wheel_rad')
    wheel_sep = rospy.get_param('~wheel_sep')
    controller = NoisyController( wheel_rad, wheel_sep)

    rospy.spin()




