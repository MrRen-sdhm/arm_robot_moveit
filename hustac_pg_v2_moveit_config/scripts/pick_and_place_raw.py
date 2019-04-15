#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 规划运动到目标位置，而不是利用pick函数

import rospy, sys
import moveit_commander
import geometry_msgs.msg
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

GRIPPER_OPEN = [-0.4]
GRIPPER_CLOSED = [0.0]

REFERENCE_FRAME = 'base_link'

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
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        self.end_effector_link = end_effector_link
 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.02)
        arm.set_goal_orientation_tolerance(0.1)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(5)

        # 设置桌面的高度
        self.table_ground = 0.65
        # 设置桌子的三维尺寸[长, 宽, 高]
        self.table_size = [0.4, 0.6, 0.5]

        # 添加场景物体
        self.add_scene()
        
        target_id = 'target'
        # 设置目标物体的尺寸
        target_size = [0.02, 0.01, 0.12]  
        # 移除场景中之前与机器臂绑定的物体
        scene.remove_attached_object(GRIPPER_FRAME, target_id)
        scene.remove_world_object(target_id)
        # 设置目标物体的位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.50
        target_pose.pose.position.y = 0.15
        target_pose.pose.position.z = self.table_ground + target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0
        # 将抓取的目标物体加入场景中
        # scene.add_box(target_id, target_pose, target_size)

        rospy.sleep(0.5)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('left_pick_startpose')
        arm.set_named_target('left_arm_startpose')
        arm.go()

        self.go_to_pose_goal(target_pose.pose.position)
                
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def go_to_pose_goal(self, target_position):
        group = self.arm

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.header.stamp = rospy.Time.now() 

        pose_goal.pose.position.x = target_position.x - 0.05
        pose_goal.pose.position.y = target_position.y
        pose_goal.pose.position.z = target_position.z

        print("Aim pose:", target_position.x - 0.05, target_position.y, target_position.z)

        q = quaternion_from_euler(-3.14/2, 0, -3.14/2)
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


    def add_scene(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        # 等待场景准备就绪
        rospy.sleep(0.5)

        table_id = 'table'
        scene.remove_world_object(table_id)
        # 将个桌子加入场景当中
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.35 + self.table_size[0]/2
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = self.table_ground - self.table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, self.table_size)
        # 设置支持的外观
        self.arm.set_support_surface_name(table_id)

        # 设置场景物体的名称 
        wall_id = 'wall'  
        # 设置wall的三维尺寸[长, 宽, 高]
        wall_size = [0.01, 1.2, 0.8]
        scene.remove_world_object(wall_id)
        # 将个物体加入场景当中
        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = 'base_link'
        wall_pose.pose.position.x = -0.2
        wall_pose.pose.position.y = 0.0
        wall_pose.pose.position.z = 0.85
        wall_pose.pose.orientation.w = 1.0
        scene.add_box(wall_id, wall_pose, wall_size)

        # 设置场景物体的名称 
        wall_1_id = 'wall_1'  
        # 设置wall_1的三维尺寸[长, 宽, 高]
        wall_1_size = [0.05, 0.3, 0.4]
        scene.remove_world_object(wall_1_id)
        # 将个物体加入场景当中
        wall_1_pose = geometry_msgs.msg.PoseStamped()
        wall_1_pose.header.frame_id = 'base_link'
        wall_1_pose.pose.position.x = -0.0
        wall_1_pose.pose.position.y = 0.0
        wall_1_pose.pose.position.z = 1.0
        wall_1_pose.pose.orientation.w = 1.0
        # scene.add_box(wall_1_id, wall_1_pose, wall_1_size)

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
        top_wall_pose.pose.position.z = 1.45
        top_wall_pose.pose.orientation.w = 1.0
        scene.add_box(top_wall_id, top_wall_pose, top_wall_size)

        # 设置场景物体的名称 
        left_wall_id = 'left_wall'  
        # 设置left_wall的三维尺寸[长, 宽, 高]
        left_wall_size = [0.5, 0.01, 1.2]
        scene.remove_world_object(left_wall_id)
        # 将个物体加入场景当中
        left_wall_pose = geometry_msgs.msg.PoseStamped()
        left_wall_pose.header.frame_id = 'base_link'
        left_wall_pose.pose.position.x = 0.2
        left_wall_pose.pose.position.y = 0.7
        left_wall_pose.pose.position.z = 0.7
        left_wall_pose.pose.orientation.w = 1.0
        # scene.add_box(left_wall_id, left_wall_pose, left_wall_size)

        # rospy.sleep(0.5)


if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
