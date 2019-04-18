#!/usr/bin/env python
#coding=utf-8
import rospy
import tf

# from std_msgs.msg import String  
from object_recognition_msgs.msg import RecognizedObjectArray

def handle_obj_pose(data):
  br = tf.TransformBroadcaster()
  if len(data.objects)>0:
    object_pose = data.objects[0].pose.pose.pose
    # print "object detected~~~~~~~~~~~~~~~~~~~~"
    # rosmsg show object_recognition_msgs/RecognizedObjectArray
    rospy.loginfo('recognized object pose reference to kinect2:\n%s\n', object_pose)
    
    br.sendTransform((object_pose.position.x, object_pose.position.y, object_pose.position.z),
                     (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z,
                     object_pose.orientation.w),
                     rospy.Time.now(),
                     "recognized_object",
                     "kinect2_rgb_optical_frame"
                     )

  else:
    # print "nothing detected................."
    pass


def listener():
  # In ROS, nodes are uniquely named. If two nodes with the same
  # name are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.

  rospy.init_node('object_tf_broadcaster', anonymous=True)
  rospy.Subscriber('/pickup/recognized_object_array', RecognizedObjectArray, handle_obj_pose)
  # print ("................")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
if __name__ == '__main__':
  listener()


""" 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
object_recognition_msgs/RecognizedObject[] objects
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  object_recognition_msgs/ObjectType type
    string key
    string db
  float32 confidence
  sensor_msgs/PointCloud2[] point_clouds
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint32 height
    uint32 width
    sensor_msgs/PointField[] fields
      uint8 INT8=1
      uint8 UINT8=2
      uint8 INT16=3
      uint8 UINT16=4
      uint8 INT32=5
      uint8 UINT32=6
      uint8 FLOAT32=7
      uint8 FLOAT64=8
      string name
      uint32 offset
      uint8 datatype
      uint32 count
    bool is_bigendian
    uint32 point_step
    uint32 row_step
    uint8[] data
    bool is_dense
  shape_msgs/Mesh bounding_mesh
    shape_msgs/MeshTriangle[] triangles
      uint32[3] vertex_indices
    geometry_msgs/Point[] vertices
      float64 x
      float64 y
      float64 z
  geometry_msgs/Point[] bounding_contours
    float64 x
    float64 y
    float64 z
  geometry_msgs/PoseWithCovarianceStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
float32[] cooccurrence
 """