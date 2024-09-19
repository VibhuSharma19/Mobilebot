#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped



class MobilebotTrajectory(object):

    def __init__(self):

        self.odom_sub_ = rospy.Subscriber("mobilebot_controller/odom", Odometry, self.odomCallback)

        self.trajec_pub_ = rospy.Publisher("mobilebot_controller/trajectory", Path, queue_size=10)

        self.trajectory_ = Path()

    def odomCallback(self, msg):

        self.trajectory_.header.frame_id = "base_footprint"

        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'odom'
        curr_pose.header.stamp = msg.header.stamp
        curr_pose.pose = msg.pose.pose

        self.trajectory_.poses.append(curr_pose)

        self.trajec_pub_.publish(self.trajectory_)


if __name__ == "__main__":
    rospy.init_node("mobilebot_trajectory")
    trajectory = MobilebotTrajectory()

    rospy.spin()