#!/usr/bin/env python

# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# https://answers.ros.org/question/47973/publishing-to-move_base_simplegoal/

import rospy
from std_msgs.msg import String
# import actionlib
# from geometry_msgs.msg import Twist
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#
# def format_goal_obj(pos):
#     print("format goal obj");
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.header.seq = 100
#     goal.target_pose.pose.position.x = pos[0]
#     goal.target_pose.pose.position.y = pos[1]
#     goal.target_pose.pose.orientation.w = 1
#     return goal
#
# goal_list = [(0, 3), (5, 1), (-5, 1)]
# def get_next_goal():
#     print("get next goal");
#     goal_list.append(goal_list.pop(0))
#     return goal_list[0]

if __name__ == '__main__':
    rospy.init_node('vrviz_test_navigation')
    print("Node inited")

    print("Creating publisher")
    p = rospy.Publisher("hello", String, queue_size=5)

    print("Publishing message")
    p.publish(String("hello there"))

    print("Publishing done")

    # client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # def move_complete(s,r): print("%s|%s"%(s,r)); client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)
    # client.send_goal(format_goal_obj(get_next_goal()), done_cb=move_complete)

    rospy.spin()
