#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arms'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'grasping_frame'

GRIPPER_OPEN = [0.004]
GRIPPER_CLOSED = [0.01]

REFERENCE_FRAME = 'base_link'

class MoveItPickAndPlaceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_pick_and_place_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # 创建一个发布抓取姿态的发布者
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        # rospy.sleep(2)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # # 初始化需要使用move group控制的机械臂中的gripper group
        # gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
 
        # 设置场景物体的名称 
        table_id = 'table'
                
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        
        # 移除场景中之前与机器臂绑定的物体
        # scene.remove_attached_object(GRIPPER_FRAME, target_id)  

        # 等待场景准备就绪
        rospy.sleep(1)
        
        # 设置桌面的高度
        table_ground = 0.5
        
        # 设置table、box1和box2的三维尺寸[长, 宽, 高]
        table_size = [0.7, 0.2, 0.01]
        
        # 将三个物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = -0.35
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
            
        # 将桌子设置成红色
        self.setColor(table_id, 1.0, 1.0, 0, 1.0)

        # 将场景中的颜色设置发布
        self.sendColors()

        # 设置支持的外观
        arm.set_support_surface_name(table_id)

        # 等待外观准备就绪
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
      
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
