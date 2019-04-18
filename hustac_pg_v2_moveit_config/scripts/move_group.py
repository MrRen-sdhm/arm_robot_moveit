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
    group_name = "arm_left"
    group = moveit_commander.MoveGroupCommander(group_name)

    # 当运动规划失败后，允许重新规划
    # group.allow_replanning(True)
        
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.05)
    group.set_goal_orientation_tolerance(0.1)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
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
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
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

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


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


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_named(self, pose_name, go=False):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    group.set_named_target(pose_name)
    if go:
      # plan + execute
      group.go()
    else:
      # plan
      traj = group.plan()
      
      # print "============ Press `Enter` to display_trajectory ..."
      # raw_input()
      # self.display_trajectory(traj)

      # print "============ Press `Enter` to execute ..."
      # raw_input()
      # execute
      group.execute(traj)

    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()


  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
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
    # pose_goal.pose.position.x = 0.5
    # pose_goal.pose.position.y = 0.0
    # pose_goal.pose.position.z = 0.7

    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.0 # 0.342
    pose_goal.pose.position.z = 0.7 # 0.462

    # pose_goal.pose.position.x = 0.191995
    # pose_goal.pose.position.y = 0.213868
    # pose_goal.pose.position.z = 0.520436

    euler = [-3.14/2, 0, -3.14/2]
    q = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]

    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal, self.eef_link)

    ## Now, we call the planner to compute the plan and execute it.
    # plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    # group.stop()

    traj = group.plan()
    print "============ Press `Enter` to execute ..."
    raw_input()
    group.execute(traj)
    # Calling `stop()` ensures that there is no residual movement
    # group.stop()


    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    tutorial.add_box()

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()

    # tutorial.go_to_pose_named('left_arm_startpose', True)
    # rospy.sleep(1)
    # tutorial.go_to_pose_named('left_cali_startpose', True)
    # rospy.sleep(0.5)
    tutorial.go_to_pose_goal()



    print "============ Complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
