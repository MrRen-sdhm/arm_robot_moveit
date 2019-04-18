#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('object_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #(trans,rot) = listener.lookupTransform('/base_link', '/recognized_object', rospy.Time(0))
            (trans,rot) = listener.lookupTransform('/base_link', '/bottle1', rospy.Time(0))
            #print "trans:", trans, "\n", "rot:", rot, "\n"
            rospy.loginfo("recognized object pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", str(trans),str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()