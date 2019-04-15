#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

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
import actionlib
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
)
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
body_hight = 0.43
left_grip_open_pos = -2.5
left_grip_close_pos = -2
right_grip_open_pos = -2.5
right_grip_close_pos = -2

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

def right_gripper_callback(data):
  print "get right gripper jointstates"

def left_gripper_callback(data):
  print "get left gripper jointstates"


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "left_arm"
    group_left = moveit_commander.MoveGroupCommander(group_name)
    group_name = "right_arm"
    group_right = moveit_commander.MoveGroupCommander(group_name)
    group_name = "left_grip"
    group_left_grip = moveit_commander.MoveGroupCommander(group_name)
    group_name = "right_grip"
    group_right_grip = moveit_commander.MoveGroupCommander(group_name)
    group_name = "body"
    group_body = moveit_commander.MoveGroupCommander(group_name)
    group_name = "arms"
    group_arms = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    rospy.Subscriber('hands/left_gripper', moveit_msgs.msg.sensor_msgs.msg.JointState, left_gripper_callback)
    rospy.Subscriber('hands/right_gripper', moveit_msgs.msg.sensor_msgs.msg.JointState, right_gripper_callback)

    body_pos_publisher = rospy.Publisher('body_hight_control', Float64, queue_size = 10)

    left_gripper_action = actionlib.SimpleActionClient("left_gripper_controller/left_gripper_action", GripperCommandAction)
    right_gripper_action = actionlib.SimpleActionClient("right_gripper_controller/right_gripper_action", GripperCommandAction)
    
    left_arm_action = actionlib.SimpleActionClient('/left_arm_trajectory', FollowJointTrajectoryAction)
    right_arm_action = actionlib.SimpleActionClient('/right_arm_trajectory', FollowJointTrajectoryAction)

    # rate = rospy.Rate(1)
    # i = 2
    # while i!=0:
    #   i=i-1
    #   print "body_hight_control pub"
    #   # body_hight = 0.43
    #   # rospy.loginfo(body_hight)
    #   pub.publish(body_hight)
    #   rate.sleep()
    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame_arms = group_arms.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame_arms
    planning_frame_l = group_left.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame_l
    planning_frame_r = group_right.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame_r

    planning_grip_l = group_left_grip.get_planning_frame()
    print "============ Reference frame: %s" % planning_grip_l
    planning_grip_r = group_right_grip.get_planning_frame()
    print "============ Reference frame: %s" % planning_grip_r

    planning_body = group_body.get_planning_frame()
    print "============ Reference frame: %s" % planning_body


    # We can also print the name of the end-effector link for this group:
    eef_link_arms = group_arms.has_end_effector_link() 
    print "============ arms End effector: %s" % eef_link_arms
    eef_link_l = group_left.get_end_effector_link()
    print "============ Left End effector: %s" % eef_link_l
    eef_link_r = group_right.get_end_effector_link()
    print "============ Right End effector: %s" % eef_link_r

    eef_grip_l = group_left_grip.get_end_effector_link()
    print "============ Left grip effector: %s" % eef_grip_l
    eef_grip_r = group_right_grip.get_end_effector_link()
    print "============ Right grip effector: %s" % eef_grip_r

    eef_body = group_body.get_end_effector_link()
    print "============ Right grip effector: %s" % eef_body

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group_left = group_left
    self.group_right = group_right
    self.group_left_grip = group_left_grip
    self.group_right_grip = group_right_grip
    self.group_body = group_body
    self.group_arms = group_arms
    self.display_trajectory_publisher = display_trajectory_publisher
    self.body_pos_publisher = body_pos_publisher
    self.left_gripper_action = left_gripper_action
    self.right_gripper_action = right_gripper_action
    self.left_arm_action = left_arm_action
    self.right_arm_action = right_arm_action
    self.planning_frame_l = planning_frame_l
    self.planning_frame_r = planning_frame_r
    self.planning_grip_l = planning_grip_l
    self.planning_grip_r = planning_grip_r
    self.eef_link_l = eef_link_l
    self.eef_link_r = eef_link_r
    self.eef_grip_l = eef_grip_l
    self.eef_grip_r = eef_grip_r
    self.eef_body = eef_body
    self.group_names = group_names

    self.group_left.allow_replanning(True)
    self.group_right.allow_replanning(True)
    self.group_left.set_planning_time(5)
    self.group_right.set_planning_time(5)

  # 升降柱到达设定值：body_hight
  def body_goto_position(self):
    # group_body
    print "body is planning"
    body_pos_publisher = self.body_pos_publisher
    body_pos_publisher.publish(body_hight)
  
  # 左夹爪到指定位置：grip_position
  def left_grip_goto_position(self, left_position):
    self.left_gripper_action.wait_for_server()
    rospy.loginfo("...left gripper connected.")

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 1.0
    gripper_goal.command.position = float(left_position)

    self.left_gripper_action.send_goal(gripper_goal)
    self.left_gripper_action.wait_for_result(rospy.Duration(2.0))
    print(self.left_gripper_action.get_result())
    rospy.loginfo("...left gripper done")

  # 右夹爪到指定角度：grip_position
  def right_grip_goto_position(self, right_position):
    self.right_gripper_action.wait_for_server()
    rospy.loginfo("...right gripper connected.")

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 1.0
    gripper_goal.command.position = float(right_position)

    self.right_gripper_action.send_goal(gripper_goal)
    self.right_gripper_action.wait_for_result(rospy.Duration(2.0))
    print(self.right_gripper_action.get_result())
    rospy.loginfo("...right gripper done")

  # 左臂到起始位置
  def left_arm_startposition(self):
    self.left_arm_action.wait_for_server()
    rospy.loginfo("...left arm connected")

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()

    for i in range(7):
        goal.trajectory.joint_names.append("left_joint" + str(i + 1))
    
    dur = float(4)
    points_length = int(300)
    joint2_pos = float(-42.9718)
    joint4_pos = float(-25.7831)
    joint5_pos = float(180)
    joint6_pos = float(22.9183)

    for t in range(points_length + 1):
        percent = float(t) / points_length
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0, percent * joint2_pos / 180 * math.pi, 0, percent * joint4_pos / 180 * math.pi, percent * joint5_pos / 180 * math.pi, percent * joint6_pos / 180 * math.pi, 0]
        # if percent <= 0.5:
        #     p.positions = [0, 0, 0, 0, 0, 0, percent / 0.5 * pos / 180 * math.pi]
        # else:
        #     p.positions = [0, 0, 0, 0, 0, 0, (1 - percent) / 0.5 * pos / 180 * math.pi]
        p.time_from_start = rospy.Time.from_sec(percent * dur)
        goal.trajectory.points.append(p)

    # Sends the goal to the action server.
    self.left_arm_action.send_goal(goal)

    # Waits for the server to finish performing the action.
    self.left_arm_action.wait_for_result(timeout=rospy.Duration(30.0))

    # Prints out the result of executing the action
    return self.left_arm_action.get_result()   

  # 右臂到起始位置
  def right_arm_startposition(self):
    self.right_arm_action.wait_for_server()
    rospy.loginfo("...right arm connected")

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()

    for i in range(7):
        goal.trajectory.joint_names.append("right_joint" + str(i + 1))
    
    dur = float(4)
    points_length = int(300)
    joint2_pos = float(42.9718)
    joint4_pos = float(25.7831)
    joint5_pos = float(180)
    joint6_pos = float(-22.9183)

    for t in range(points_length + 1):
        percent = float(t) / points_length
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0, percent * joint2_pos / 180 * math.pi, 0, percent * joint4_pos / 180 * math.pi, percent * joint5_pos / 180 * math.pi, percent * joint6_pos / 180 * math.pi, 0]
        # if percent <= 0.5:
        #     p.positions = [0, 0, 0, 0, 0, 0, percent / 0.5 * pos / 180 * math.pi]
        # else:
        #     p.positions = [0, 0, 0, 0, 0, 0, (1 - percent) / 0.5 * pos / 180 * math.pi]
        p.time_from_start = rospy.Time.from_sec(percent * dur)
        goal.trajectory.points.append(p)

    # Sends the goal to the action server.
    self.right_arm_action.send_goal(goal)

    # Waits for the server to finish performing the action.
    self.right_arm_action.wait_for_result(timeout=rospy.Duration(30.0))

    # Prints out the result of executing the action
    return self.right_arm_action.get_result()   

  # 左臂到指定位置
  ### 通过插补各关节点完成特定的任务
  def left_arm_goto_position(self, plan_l):
    self.left_arm_action.wait_for_server()
    rospy.loginfo("...left arm connected")

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = plan_l.joint_trajectory
    # for i in range(7):
    #     goal.trajectory.joint_names.append("left_joint" + str(i + 1))

    # for t in range(20):
    #     p = trajectory_msgs.msg.JointTrajectoryPoint()
    #     p.positions = [t / 10 / 180 * math.pi] * 7
    #     p.time_from_start = rospy.Time.from_sec(t / 10)
    #     goal.trajectory.points.append(p)

    # Sends the goal to the action server.
    self.left_arm_action.send_goal(goal)

    # Waits for the server to finish performing the action.
    self.left_arm_action.wait_for_result(timeout=rospy.Duration(30.0))

    # Prints out the result of executing the action
    return self.left_arm_action.get_result()   

  # 右臂到指定位置
  def right_arm_goto_position(self, plan_r):
    self.right_arm_action.wait_for_server()
    rospy.loginfo("...right arm connected")

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = plan_r.joint_trajectory
    # for i in range(7):
    #     goal.trajectory.joint_names.append("right_joint" + str(i + 1))

    # for t in range(20):
    #     p = trajectory_msgs.msg.JointTrajectoryPoint()
    #     p.positions = [t / 10 / 180 * math.pi] * 7
    #     p.time_from_start = rospy.Time.from_sec(t / 10)
    #     goal.trajectory.points.append(p)

    # Sends the goal to the action server.
    self.right_arm_action.send_goal(goal)

    # Waits for the server to finish performing the action.
    self.right_arm_action.wait_for_result(timeout=rospy.Duration(30.0))

    # Prints out the result of executing the action
    return self.right_arm_action.get_result()   

  # 左爪子打开
  def left_grip_open(self):
    # group_left_grip
    print "left_grip is planning"
    group = self.group_left_grip
    # BEGIN_SUB_TUTORIAL plan_to_joint_state
    #
    # Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    print len(joint_goal)
    joint_goal[0] = -0.5

    self.left_grip_goto_position(left_grip_open_pos)      # 发送action到单片机
    group.go(joint_goal, wait=True)

    group.stop()
    current_joints = self.group_left_grip.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  
  # 左爪子关闭
  def left_grip_close(self):
    # group_left_grip
    print "left_grip is planning"
    group = self.group_left_grip
    # BEGIN_SUB_TUTORIAL plan_to_joint_state
    #
    # Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    print len(joint_goal)
    joint_goal[0] = 0

    self.left_grip_goto_position(left_grip_close_pos)      # 发送action到单片机
    group.go(joint_goal, wait=True)

    group.stop()
    current_joints = self.group_left_grip.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  
  # 右爪子打开
  def right_grip_open(self):
    # group_right_grip
    print "right_grip is planning"
    group = self.group_right_grip
    # BEGIN_SUB_TUTORIAL plan_to_joint_state
    #
    # Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    print len(joint_goal)
    joint_goal[0] = -0.5

    self.right_grip_goto_position(right_grip_open_pos)      # 发送action到单片机
    group.go(joint_goal, wait=True)

    group.stop()
    current_joints = self.group_right_grip.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  
  # 右爪子关闭
  def right_grip_close(self):
    # group_right_grip
    print "right_grip is planning"
    group = self.group_right_grip
    # BEGIN_SUB_TUTORIAL plan_to_joint_state
    #
    # Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    print len(joint_goal)
    joint_goal[0] = 0

    self.right_grip_goto_position(right_grip_close_pos)      # 发送action到单片机
    group.go(joint_goal, wait=True)

    group.stop()
    current_joints = self.group_right_grip.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state(self):
    ###### Left Arm Move  ########
    group = self.group_arms
    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    joint_goal = group.get_current_joint_values()
    print len(joint_goal)
    joint_goal[0] = 0
    joint_goal[1] = -0.75
    joint_goal[2] = 0
    joint_goal[3] = -0.45
    joint_goal[4] = 3.14
    joint_goal[5] = 0.4
    joint_goal[6] = 0

    joint_goal[7] = 0
    joint_goal[8] = 0.75
    joint_goal[9] = 0
    joint_goal[10] = 0.45
    joint_goal[11] = 3.14
    joint_goal[12] = -0.4
    joint_goal[13] = 0

    self.left_arm_startposition()      # 发送action到单片机
    self.right_arm_startposition()      # 发送action到单片机
    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group_arms.get_current_joint_values()

    return all_close(joint_goal, current_joints, 0.01)

  #### 到达指定的三维坐标系中（绝对坐标系）,无法产生轨迹点，不可用。
  def go_to_pose_goal(self):
    group = self.group_arms

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    line = raw_input("Please input goal leftarm position(x,y,z):")
    goal_x, goal_y, goal_z = (float(x) for x in line.split(' '))

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = goal_x
    pose_goal.position.y = goal_y
    pose_goal.position.z = goal_z
    group.set_pose_target(pose_goal, "left_link7")

    line = raw_input("Please input goal rightarm position(x,y,z):")
    goal_x, goal_y, goal_z = (float(x) for x in line.split(' '))

    pose_goal1 = geometry_msgs.msg.Pose()
    pose_goal1.orientation.w = 1.0
    pose_goal1.position.x = goal_x
    pose_goal1.position.y = goal_y
    pose_goal1.position.z = goal_z
    group.set_pose_target(pose_goal1, "right_link7")

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL
    current_pose = self.group_arms.get_current_pose("left_link7").pose
    current_pose = self.group_arms.get_current_pose("right_link7").pose

    return all_close(pose_goal, current_pose, 0.01)

  # 笛卡尔坐标系下移动（相对坐标系）
  def plan_cartesian_path(self, l_des, r_des, scale=1):
    group = self.group_left

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    # wpose.position.z -= scale * 0.3  # First move up (z)
    # wpose.position.y -= scale * 0.3  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * l_des[0]  # Second move forward/backwards in (x)
    wpose.position.y += scale * l_des[1]  # Second move forward/backwards in (x)
    wpose.position.z += scale * l_des[2]  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.3  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan_l, fraction_l) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    group = self.group_right

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    # wpose.position.z -= scale * 0.3  # First move up (z)
    # wpose.position.y += scale * 0.3  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * r_des[0]  # Second move forward/backwards in (x)
    wpose.position.y += scale * r_des[1]  # Second move forward/backwards in (y)
    wpose.position.z += scale * r_des[2]  # Second move forward/backwards in (z)
    waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.3  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan_r, fraction_r) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan_l, fraction_l, plan_r, fraction_r
    ## END_SUB_TUTORIAL

  # RVIZ中显示轨迹
  def display_trajectory(self, plan_l, plan_r):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan_l)
    display_trajectory.trajectory.append(plan_r)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL

  # 执行规划
  def execute_plan(self, plan_l, plan_r):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group_left

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan_l, wait=True)

    group = self.group_right

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan_r, wait=True)
    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
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
    ## END_SUB_TUTORIAL

  def pass_stuff(self):
    cartesian_plan_l, fraction_l, cartesian_plan_r, fraction_r = self.plan_cartesian_path([-0.3,0,0], [0.3,0,0])
    self_trajectory_l = cartesian_plan_l.joint_trajectory.points
    print "First list length : ", len(self_trajectory_l)    
    # for index in range(len(self_trajectory_l)):
    #   print "self_trajectory_l[" , index , "]_positions : " , self_trajectory_l[index].positions[0] , "   " , self_trajectory_l[index].positions[1] , "   " , self_trajectory_l[index].positions[2] , "   " , self_trajectory_l[index].positions[3] , "   " , self_trajectory_l[index].positions[4] , "   " , self_trajectory_l[index].positions[5] , "   " , self_trajectory_l[index].positions[6]
    #   print "self_trajectory_l[" , index , "]_velocities : " , self_trajectory_l[index].velocities[0] , "   " , self_trajectory_l[index].velocities[1] , "   " , self_trajectory_l[index].velocities[2]
    #   print "self_trajectory_l[" , index , "]accelerations : " , self_trajectory_l[index].accelerations[0] , "   " , self_trajectory_l[index].accelerations[1] , "   " , self_trajectory_l[index].accelerations[2] , "\n"

    self_trajectory_r = cartesian_plan_r.joint_trajectory.points
    print "First list length : ", len(self_trajectory_r)    
    # for index in range(len(self_trajectory_r)):
    #   print "self_trajectory_r[" , index , "]_positions : " , self_trajectory_r[index].positions[0] , "   " , self_trajectory_r[index].positions[1] , "   " , self_trajectory_r[index].positions[2] , "   " , self_trajectory_r[index].positions[3] , "   " , self_trajectory_r[index].positions[4] , "   " , self_trajectory_r[index].positions[5] , "   " , self_trajectory_r[index].positions[6]
    #   print "self_trajectory_r[" , index , "]_velocities : " , self_trajectory_r[index].velocities[0] , "   " , self_trajectory_r[index].velocities[1] , "   " , self_trajectory_r[index].velocities[2]
    #   print "self_trajectory_r[" , index , "]accelerations : " , self_trajectory_r[index].accelerations[0] , "   " , self_trajectory_r[index].accelerations[1] , "   " , self_trajectory_r[index].accelerations[2] , "\n"

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan_l,cartesian_plan_r)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    self.left_arm_goto_position(cartesian_plan_l)
    self.right_arm_goto_position(cartesian_plan_r)
    self.execute_plan(cartesian_plan_l, cartesian_plan_r)
    print "============ Pass movement complete!"
    
  def back_start(self):
    cartesian_plan_l, fraction_l, cartesian_plan_r, fraction_r = self.plan_cartesian_path([0.3,0,0], [-0.3,0,0])
    self_trajectory_l = cartesian_plan_l.joint_trajectory.points
    print "First list length : ", len(self_trajectory_l)    
    # for index in range(len(self_trajectory_l)):
    #   print "self_trajectory_l[" , index , "]_positions : " , self_trajectory_l[index].positions[0] , "   " , self_trajectory_l[index].positions[1] , "   " , self_trajectory_l[index].positions[2] , "   " , self_trajectory_l[index].positions[3] , "   " , self_trajectory_l[index].positions[4] , "   " , self_trajectory_l[index].positions[5] , "   " , self_trajectory_l[index].positions[6]
    #   print "self_trajectory_l[" , index , "]_velocities : " , self_trajectory_l[index].velocities[0] , "   " , self_trajectory_l[index].velocities[1] , "   " , self_trajectory_l[index].velocities[2]
    #   print "self_trajectory_l[" , index , "]accelerations : " , self_trajectory_l[index].accelerations[0] , "   " , self_trajectory_l[index].accelerations[1] , "   " , self_trajectory_l[index].accelerations[2] , "\n"

    self_trajectory_r = cartesian_plan_r.joint_trajectory.points
    print "First list length : ", len(self_trajectory_r)    
    # for index in range(len(self_trajectory_r)):
    #   print "self_trajectory_r[" , index , "]_positions : " , self_trajectory_r[index].positions[0] , "   " , self_trajectory_r[index].positions[1] , "   " , self_trajectory_r[index].positions[2] , "   " , self_trajectory_r[index].positions[3] , "   " , self_trajectory_r[index].positions[4] , "   " , self_trajectory_r[index].positions[5] , "   " , self_trajectory_r[index].positions[6]
    #   print "self_trajectory_r[" , index , "]_velocities : " , self_trajectory_r[index].velocities[0] , "   " , self_trajectory_r[index].velocities[1] , "   " , self_trajectory_r[index].velocities[2]
    #   print "self_trajectory_r[" , index , "]accelerations : " , self_trajectory_r[index].accelerations[0] , "   " , self_trajectory_r[index].accelerations[1] , "   " , self_trajectory_r[index].accelerations[2] , "\n"

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan_l,cartesian_plan_r)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    self.left_arm_goto_position(cartesian_plan_l)
    self.right_arm_goto_position(cartesian_plan_r)
    self.execute_plan(cartesian_plan_l, cartesian_plan_r)
    print "============ Pass Back movement complete!"
    
def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()
    tutorial.left_grip_open()
    tutorial.right_grip_open()
    # tutorial.left_grip_goto_position()

    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # tutorial.go_to_pose_goal()

    print "============ Press `Enter` to begin a movement passing a stuff ..."
    raw_input()
    tutorial.left_grip_close()
    tutorial.pass_stuff()

    print "============ Press `Enter` to begin right gripper close and left gripper!"
    raw_input()
    tutorial.right_grip_close()
    tutorial.left_grip_open()

    print "============ Press `Enter` to move right arm begin!"
    raw_input()
    tutorial.back_start()
    tutorial.right_grip_open()

    print "============ Python tutorial demo complete!"

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan_l, fraction_l, cartesian_plan_r, fraction_r = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan_l, cartesian_plan_r)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

