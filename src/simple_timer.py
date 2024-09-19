#!/usr/bin/python3

import rospy

def timerCallback(evwnt=None):
    rospy.loginfo("Called Timer Callback Function")

if __name__ == "__main__":
    rospy.init_node('Simple_timer', anonymous=True)
    timer_dur = rospy.Duration(1)
    rospy.Timer(timer_dur, timerCallback)
    rospy.spin()