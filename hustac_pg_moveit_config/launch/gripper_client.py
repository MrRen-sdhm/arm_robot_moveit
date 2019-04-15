#!/usr/bin/env python
import sys, rospy, actionlib
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)

if __name__ == "__main__":
    rospy.init_node("gripper")
    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
      print("usage: gripper.py GRASP_WIDTH")
      sys.exit(1)
    
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    rospy.loginfo("...connected.")

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = float(args[1])

    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(2.0))
    print(gripper.get_result())
    rospy.loginfo("...done")

#!/usr/bin/env python

# """
# Baxter RSDK Gripper Action Client Example
# """
# import sys
# import argparse

# import rospy

# import actionlib

# from control_msgs.msg import (
#     GripperCommandAction,
#     GripperCommandGoal,
# )

# import baxter_interface

# from baxter_interface import CHECK_VERSION


# class GripperClient(object):
#     def __init__(self, gripper):
#         ns = 'robot/end_effector/' + gripper + '_gripper/'
#         self._client = actionlib.SimpleActionClient(
#             ns + "gripper_action",
#             GripperCommandAction,
#         )
#         self._goal = GripperCommandGoal()

#         # Wait 10 Seconds for the gripper action server to start or exit
#         if not self._client.wait_for_server(rospy.Duration(10.0)):
#             rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
#                          (gripper.capitalize(),))
#             rospy.signal_shutdown("Action Server not found")
#             sys.exit(1)
#         self.clear()

#     def command(self, position, effort):
#         self._goal.command.position = position
#         self._goal.command.max_effort = effort
#         self._client.send_goal(self._goal)

#     def stop(self):
#         self._client.cancel_goal()

#     def wait(self, timeout=5.0):
#         self._client.wait_for_result(timeout=rospy.Duration(timeout))
#         return self._client.get_result()

#     def clear(self):
#         self._goal = GripperCommandGoal()


# def main():
#     """RSDK Gripper Example: Action Client
#     Demonstrates creating a client of the Gripper Action Server,
#     which enables sending commands of standard action type
#     control_msgs/GripperCommand.
#     The example will command the grippers to a number of positions
#     while specifying moving force or vacuum sensor threshold. Be sure
#     to start Baxter's gripper_action_server before running this example.
#     """
#     arg_fmt = argparse.RawDescriptionHelpFormatter
#     parser = argparse.ArgumentParser(formatter_class=arg_fmt,
#                                      description=main.__doc__)
#     parser.add_argument(
#         '-g', '--gripper', dest='gripper', required=True,
#         choices=['left', 'right'],
#         help='which gripper to send action commands'
#     )
#     args = parser.parse_args(rospy.myargv()[1:])
#     gripper = args.gripper

#     print("Initializing node... ")
#     rospy.init_node("rsdk_gripper_action_client_%s" % (gripper,))
#     print("Getting robot state... ")
#     rs = baxter_interface.RobotEnable(CHECK_VERSION)
#     print("Enabling robot... ")
#     rs.enable()
#     print("Running. Ctrl-c to quit")

#     gc = GripperClient(gripper)
#     gc.command(position=0.0, effort=50.0)
#     gc.wait()
#     gc.command(position=100.0, effort=50.0)
#     gc.wait()
#     gc.command(position=25.0, effort=40.0)
#     gc.wait()
#     gc.command(position=75.0, effort=20.0)
#     gc.wait()
#     gc.command(position=0.0, effort=30.0)
#     gc.wait()
#     gc.command(position=100.0, effort=40.0)
#     print gc.wait()
#     print "Exiting - Gripper Action Test Example Complete"

# if __name__ == "__main__":
#     main()