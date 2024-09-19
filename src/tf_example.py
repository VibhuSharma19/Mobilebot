#!/usr/bin/python3

import rospy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from mobilebot.srv import GetTransform, GetTransformResponse
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

class TfExample(object):


    def __init__(self):

        self.static_broadcaster_ = StaticTransformBroadcaster()
        self.dynamic_broadcaster_ = TransformBroadcaster()
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        self.rotation_counter_ = 0
        self.last_quaternion_ = quaternion_from_euler(0, 0, 0)
        self.quaternion_increment_ = quaternion_from_euler(0, 0, 0.25)

        self.static_transform_stamped_.header.stamp = rospy.Time.now()
        self.static_transform_stamped_.header.frame_id = "mobilebot_base"
        self.static_transform_stamped_.child_frame_id = "mobilebot_top"

        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0
        self.static_transform_stamped_.transform.rotation.y = 0
        self.static_transform_stamped_.transform.rotation.z = 0
        self.static_transform_stamped_.transform.rotation.w = 1

        self.static_broadcaster_.sendTransform(self.static_transform_stamped_)
        rospy.loginfo("The static transform between %s and %s is being published", self.static_transform_stamped_.header.frame_id, 
                        self.static_transform_stamped_.child_frame_id)

        self.get_transform_srv_ = rospy.Service('get_transform', GetTransform, self.gettransformCallback)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_)


    def timerCallback(self, event):
        self.dynamic_transform_stamped_.header.stamp = rospy.Time.now()
        self.dynamic_transform_stamped_.header.frame_id = "odem"
        self.dynamic_transform_stamped_.child_frame_id = "mobilebot_base"

        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

       #self.dynamic_transform_stamped_.transform.rotation.x = 0.0
       #self.dynamic_transform_stamped_.transform.rotation.y = 0.0
       #self.dynamic_transform_stamped_.transform.rotation.z = 0.0
       #self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        q = quaternion_multiply(self.last_quaternion_, self.quaternion_increment_)
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        self.dynamic_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x
        self.rotation_counter_ += 1
        self.last_quaternion_ = q

        if self.rotation_counter_ >= 100:
            self.quaternion_increment_ = quaternion_inverse(self.quaternion_increment_)
            self.rotation_counter_ = 0


    def gettransformCallback(self, req):
        rospy.loginfo("Resquested Transform between %s and %s", req.frame_id, req.child_frame_id)
        res = GetTransformResponse()
        resquested_transform = TransformStamped()

        try:
            resquested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rospy.Time())
        except Exception as e:
            rospy.logerr('Error getting transformation')
            res.success = False
            return res

        rospy.loginfo('The requested tranform is: %s', resquested_transform)
        res.transform = resquested_transform
        res.success = True
        return res




if __name__ == "__main__":
    rospy.init_node('tf_example')
    tfExample = TfExample()
    rospy.spin()