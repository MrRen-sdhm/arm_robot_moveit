#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy, sys, tf
import moveit_commander
import geometry_msgs.msg
import actionlib
import moveit_msgs.msg
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm_left'
GROUP_NAME_GRIPPER = 'left_gripper'

GRIPPER_FRAME = 'left_ee_link'

GRIPPER_OPEN = [-0.6]
GRIPPER_CLOSED = [0.0]

REFERENCE_FRAME = 'base_link'


# 定义夹爪的开合角度
left_grip_open_pos = -3.0
# left_grip_close_pos = -2.15
left_grip_close_pos = -2.5


def left_gripper_callback(data):
  print "get left gripper jointstates"


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



class MoveItPickAndPlaceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_pick_and_place_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        self.scene = scene
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # 创建一个发布抓取姿态的发布者
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        self.arm = arm

        group_name = "left_gripper"
        group_left_grip = moveit_commander.MoveGroupCommander(group_name)
        self.group_left_grip = group_left_grip

        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        self.end_effector_link = end_effector_link

        print ("start")

        # rospy.Subscriber('hands/left_gripper', moveit_msgs.msg.sensor_msgs.msg.JointState, left_gripper_callback)

        planning_grip_l = group_left_grip.get_planning_frame()
        print "============ Reference frame: %s" % planning_grip_l

        # We can also print the name of the end-effector link for this group:
        eef_link_arms = arm.has_end_effector_link() 
        print "============ arms End effector: %s" % eef_link_arms
        eef_link_l = group_left_grip.get_end_effector_link()
        print "============ Left End effector: %s" % eef_link_l
  
        left_gripper_action = actionlib.SimpleActionClient("hand/left", GripperCommandAction)
        # right_gripper_action = actionlib.SimpleActionClient("hand/right/gripper_action", GripperCommandAction)
        rospy.loginfo("...left gripper wait_for_server.")
        left_gripper_action.wait_for_server()
        rospy.loginfo("...left gripper action connected.")
        # right_gripper_action.wait_for_server()
        # rospy.loginfo("...right gripper action connected.")
        self.left_gripper_action = left_gripper_action
        
        # self.left_grip_open()
        self.left_grip_close()

        print ("exit")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    # 左夹爪到指定位置：grip_position
    def left_grip_goto_position(self, left_position):
        # self.left_gripper_action.wait_for_server()
        # rospy.loginfo("...left gripper action connected.")

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 1.0
        gripper_goal.command.position = float(left_position)

        self.left_gripper_action.send_goal(gripper_goal)
        # self.left_gripper_action.wait_for_result(rospy.Duration(2.0))
        # print(self.left_gripper_action.get_result())
        rospy.loginfo("...left gripper done")

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

        # group.stop()
        current_joints = self.group_left_grip.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    # 左爪子关闭
    def left_grip_close(self):
        # group_left_grip
        print "left_grip is planning"
        group = self.group_left_grip
        # BEGIN_SUB_TUTORIAL plan_to_joint_state
        #
        joint_goal = group.get_current_joint_values()
        print joint_goal
        joint_goal[0] = 0

        self.left_grip_goto_position(left_grip_close_pos)      # 发送action到单片机
        # group.go(joint_goal, wait=True)

        # group.stop()
        # current_joints = self.group_left_grip.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)
        
    def go_to_pose_goal(self):
        group = self.arm

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.header.stamp = rospy.Time.now() 

        pose_goal.pose.position.x = 0.4
        pose_goal.pose.position.y = 0.0
        pose_goal.pose.position.z = 0.75

        q = quaternion_from_euler(-3.14/2, 0, -3.14/2)
        # q = quaternion_from_euler(3.14, 0, -3.14/2)
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal, self.end_effector_link)

        traj = group.plan()  
        group.execute(traj)

        group.stop()
        group.clear_pose_targets()



if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
