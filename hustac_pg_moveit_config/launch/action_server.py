#! /usr/bin/env python

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
)

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, _goal):
        r = rospy.Rate(1)
        success = True

        # start executing the action
        for i in range(0, 2):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False

            self._feedback.feedback.header.seq = 1
            self._as.publish_feedback(self._feedback.feedback)

            if success:
                self._result.header = self._feedback.header
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result.result)


if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()