#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # 当运动规划失败后，允许重新规划
    # group.allow_replanning(True)
        
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.05)
    group.set_goal_orientation_tolerance(0.05)

    ## Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def add_box(self, timeout=0.5):
    box_name = self.box_name
    scene = self.scene

    # 等待场景准备就绪
    # rospy.sleep(0.5)

    # # 设置场景物体的名称 
    # table_id = 'table'  
    # # 设置桌面的高度
    # table_ground = 0.6
    # # 设置table的三维尺寸[长, 宽, 高]
    # table_size = [0.3, 0.5, 0.01]
    # scene.remove_world_object(table_id)
    # # 将个物体加入场景当中
    # table_pose = geometry_msgs.msg.PoseStamped()
    # table_pose.header.frame_id = 'base_link'
    # table_pose.pose.position.x = 0.5 + table_size[0]/2
    # table_pose.pose.position.y = 0.0
    # table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    # table_pose.pose.orientation.w = 1.0
    # scene.add_box(table_id, table_pose, table_size)

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
    wall_pose.pose.position.z = 0.8
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

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    # Ensuring Collision Updates Are Receieved
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def go_to_joint_state(self):
    group = self.group

    # Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_named(self, pose_name):
    group = self.group

    group.set_named_target(pose_name)

    group.go()

    group.stop()
    # It is always good to clear your targets after planning with poses.
    group.clear_pose_targets()


  def go_to_pose_goal(self):
    group = self.group

    ## We can plan a motion for this group to a desired pose for the end-effector:
    
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.0
    # pose_goal.position.y = 0.0
    # pose_goal.position.z = 0.0

    # euler = [0, 0, pi/4]
    # q = quaternion_from_euler(euler[0], euler[1], euler[2])
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]
    # group.set_pose_target(pose_goal)

    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = 'base_link'
    pose_goal.header.stamp = rospy.Time.now()

    pose_goal.pose.position.x = 0.500
    pose_goal.pose.position.y = 0.000
    pose_goal.pose.position.z = 0.500

    euler = [0, 0, 0]
    # euler = [0, -pi, 0]
    q = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]

    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal, self.eef_link)

    plan = group.go(wait=True)
    group.stop()

    # traj = group.plan()
    # print "============ Press `Enter` to execute ..."
    # raw_input()
    # group.execute(traj)
    # group.stop()

    # It is always good to clear your targets after planning with poses.
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    # listener = tf.TransformListener()
    # rate = rospy.Rate(1.0)
    # while not rospy.is_shutdown():
    #     try:
    #         (trans,rot) = listener.lookupTransform('/base_link', '/gripper', rospy.Time(0))
    #         print trans
    #         print rot
    #         print ("")
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    # exit()

    # tutorial.add_box()

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement using a pose name ..."
    # raw_input()
    # tutorial.go_to_pose_named('test2')

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
