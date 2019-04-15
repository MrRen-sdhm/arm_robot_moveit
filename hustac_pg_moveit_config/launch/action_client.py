#! /usr/bin/env python

from __future__ import print_function
import math

import rospy
# Brings in the SimpleActionClient
import actionlib

import trajectory_msgs.msg

from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
)

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/arm_trajectory', FollowJointTrajectoryAction)

    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()
    for i in range(7):
        goal.trajectory.joint_names.append("left_joint" + str(i + 1))

    for t in range(20):
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [t / 10 / 180 * math.pi] * 7
        p.time_from_start = rospy.Time.from_sec(t / 10)
        goal.trajectory.points.append(p)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(timeout=rospy.Duration(30.0))

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('action_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)