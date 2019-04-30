#!/usr/bin/env python
# -*- coding: utf-8 -*-

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
    group_name = "arm_left"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    group_name_gripper = 'left_gripper'
    gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

    gripper.set_joint_value_target([-0.6])
    gripper.go()

    reference_frame = 'head_link'
    group.set_pose_reference_frame(reference_frame)

    # 当运动规划失败后，允许重新规划
    group.allow_replanning(True)
        
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)

    # Get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "\n[ INFO] Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "[ INFO] End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "[ INFO] Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.reference_frame = reference_frame
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def add_box(self, timeout=0.5):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    # 等待场景准备就绪
    rospy.sleep(0.5)

    # 设置场景物体的名称 
    table_id = 'table'  
    # 设置桌面的高度
    table_ground = 0.65
    # 设置table的三维尺寸[长, 宽, 高]
    table_size = [0.4, 0.6, 0.5]
    scene.remove_world_object(table_id)
    # 将个物体加入场景当中
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = 0.4 + table_size[0]/2
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = table_ground - table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    scene.add_box(table_id, table_pose, table_size)


    # 设置场景物体的名称 
    wall_id = 'wall'  
    # 设置wall的三维尺寸[长, 宽, 高]
    wall_size = [0.01, 0.8, 0.4]
    scene.remove_world_object(wall_id)
    # 将个物体加入场景当中
    wall_pose = geometry_msgs.msg.PoseStamped()
    wall_pose.header.frame_id = 'base_link'
    wall_pose.pose.position.x = -0.08
    wall_pose.pose.position.y = 0.0
    wall_pose.pose.position.z = 0.9
    wall_pose.pose.orientation.w = 1.0
    scene.add_box(wall_id, wall_pose, wall_size)


    # 设置场景物体的名称 
    top_wall_id = 'top_wall'  
    # 设置top_wall的三维尺寸[长, 宽, 高]
    top_wall_size = [0.3, 1.0, 0.01]
    scene.remove_world_object(top_wall_id)
    # 将个物体加入场景当中
    top_wall_pose = geometry_msgs.msg.PoseStamped()
    top_wall_pose.header.frame_id = 'base_link'
    top_wall_pose.pose.position.x = 0.07
    top_wall_pose.pose.position.y = 0.0
    top_wall_pose.pose.position.z = 1.4
    top_wall_pose.pose.orientation.w = 1.0
    scene.add_box(top_wall_id, top_wall_pose, top_wall_size)

    # 设置场景物体的名称 
    left_wall_id = 'left_wall'  
    # 设置left_wall的三维尺寸[长, 宽, 高]
    left_wall_size = [0.5, 0.01, 0.8]
    scene.remove_world_object(left_wall_id)
    # 将个物体加入场景当中
    left_wall_pose = geometry_msgs.msg.PoseStamped()
    left_wall_pose.header.frame_id = 'base_link'
    left_wall_pose.pose.position.x = 0.2
    left_wall_pose.pose.position.y = 0.6
    left_wall_pose.pose.position.z = 0.7
    left_wall_pose.pose.orientation.w = 1.0
    # scene.add_box(left_wall_id, left_wall_pose, left_wall_size)

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
      box_name = self.box_name
      scene = self.scene

      start = rospy.get_time()
      seconds = rospy.get_time()
      while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
          return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

      # If we exited the while loop without returning then we timed out
      return False

 
  def go_to_pose_goal(self):
    group = self.group

    listener = tf.TransformListener()
    while not rospy.is_shutdown():
      try:
          (obj_position, obj_orientation) = listener.lookupTransform('/head_link', '/grasp', rospy.Time(0))
          rospy.loginfo("Selected grasp pose reference to head_link:\nposition:\n %s\norientation:\n %s\n", 
            str(obj_position), str(obj_orientation))
          break
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue

    ## We can plan a motion for this group to a desired pose for the end-effector:
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = self.reference_frame
    pose_goal.header.stamp = rospy.Time.now()

    pose_goal.pose.position.x = obj_position[0]
    pose_goal.pose.position.y = obj_position[1]
    pose_goal.pose.position.z = obj_position[2]

    pose_goal.pose.orientation.x = obj_orientation[0]
    pose_goal.pose.orientation.y = obj_orientation[1]
    pose_goal.pose.orientation.z = obj_orientation[2]
    pose_goal.pose.orientation.w = obj_orientation[3]

    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal, self.eef_link)

    ## Now, we call the planner to compute the plan and execute it.
    traj = group.plan()  
    # print "\n[ INFO] Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()
    group.execute(traj)

    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    group.clear_pose_targets()

    # For testing:
    current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    tutorial.add_box()

    tutorial.go_to_pose_goal()

    print "[ INFO] Complete!", "\n"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  # rospy.init_node('object_tf_listener')

  # listener = tf.TransformListener()
  # obj_position = []
  # obj_orientation = []

  # rate = rospy.Rate(100.0)
  # while not rospy.is_shutdown():
  #   try:
  #       (obj_position,obj_orientation) = listener.lookupTransform('/base_link', '/recognized_object', rospy.Time(0))
  #       rospy.loginfo("recognized object pose reference to base_link:\nposition:\n %s\norientation:\n %s\n", 
  #         str(obj_position),str(obj_orientation))
  #   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  #       continue

  #   rate.sleep()

  main()

