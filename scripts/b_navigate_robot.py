#!/usr/bin/env python

# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# https://answers.ros.org/question/47973/publishing-to-move_base_simplegoal/

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def format_goal_obj(pos):
    rospy.loginfo("Formatted goal object");
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.seq = 100
    goal.target_pose.pose.position.x = pos[0]
    goal.target_pose.pose.position.y = pos[1]
    goal.target_pose.pose.orientation.w = 1
    return goal

# goal_list = [(0, 3), (5, 1), (-5, 1)]
goal_list = [(0, 1), (1, 0), (0, -1), (-1, 0)]
def get_next_goal():
    goal_list.append(goal_list.pop(0))
    rospy.loginfo("Identified next goal: (%s,%s)" % (goal_list[0][0], goal_list[0][1]))
    return goal_list[0]

if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('vrviz_test_navigation')
    rospy.loginfo("Node inited")
    rospy.sleep(1)

    # rospy.loginfo("Creating publisher")
    # p = rospy.Publisher("hello", String, queue_size=5)
    # rospy.sleep(1)

    # rospy.loginfo("Publishing message")
    # p.publish(String("hello there"))
    # rospy.sleep(1)

    # rospy.loginfo("Publishing done")

    rospy.loginfo("Defining Client")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.sleep(1)

    rospy.loginfo("Defining Response Function")
    def move_complete(s,r):
        rospy.loginfo("Result: %s" % s)
        client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)
    rospy.sleep(1)

    rospy.loginfo("Sending Initial Goal")
    client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)
    rospy.sleep(1)

    rospy.spin()
