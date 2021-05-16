#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goal_list = [(-1,3), (0, 1), (4.5, -2), (1, 0), (2, -4), (1, 1), (0, -1), (-1, 0)]
# goal_list = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def format_goal_obj(pos):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pos[0]
    goal.target_pose.pose.position.y = pos[1]
    goal.target_pose.pose.orientation.w = 1
    return goal


def get_next_goal():
    goal_list.append(goal_list.pop(0))
    rospy.logerr("Identified next goal: (%s,%s)" % (goal_list[0][0], goal_list[0][1]))
    return goal_list[0]


if __name__ == '__main__':
    rospy.sleep(5)
    rospy.init_node('vrviz_test_navigation')

    rospy.logerr("Initialising Client")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.sleep(1)  # TODO: find out if there is an actual function for this

    def move_complete(status, _):
        rospy.logerr("Status: %s" % status)
        client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)

    rospy.logerr("Sending Initial Goal")
    client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)

    rospy.spin()
