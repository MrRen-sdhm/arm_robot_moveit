#!/usr/bin/env python
import roslib
import rospy

import tf
import geometry_msgs

recognized_object_pose = geometry_msgs.msg.PoseStamped()
recognized_object_pose.header.frame_id = "recognized_object"
recognized_object_pose.header.stamp = rospy.Time(0)

recognized_object_pose.pose.position.x = -0.201525211334
recognized_object_pose.pose.position.y = 0.0937595963478
recognized_object_pose.pose.position.z = 0.783895611763
recognized_object_pose.pose.orientation.x = -0.654610931873
recognized_object_pose.pose.orientation.y = 0.752546012402
recognized_object_pose.pose.orientation.z = -0.06301407516
recognized_object_pose.pose.orientation.w = -0.0344694405794

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((recognized_object_pose.pose.position.x, recognized_object_pose.pose.position.y, recognized_object_pose.pose.position.z),
                     (recognized_object_pose.pose.orientation.x, recognized_object_pose.pose.orientation.y, recognized_object_pose.pose.orientation.z
                     recognized_object_pose.pose.orientation.w),
                     rospy.Time.now(),
                     "kinect2_rgb_optical_frame",
                     "recognized_object")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.spin()
