#!/usr/bin/env python

from rospy import init_node, Publisher
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback

"""
goal_list = [...]
while not rospy.is_shutdown():
    status = robot.is_at_goal()
    if status == success:
        movebase.publish(random_sample(goal_list))
"""


#self.subscriberF1 = rospy.Subscriber("/move_base/status", GoalStatusArray, self.listenerF1)
#LISTENERS_DATA.GOAL_STATUS = data.status_list[len(data.status_list)-1].text
#self.publisher_GOAL = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 2)

	# def publish_GOAL2(self, new_x, new_y, angle, move_type="move"):
	# 	goal = MoveBaseActionGoal()
	# 	goal.goal.target_pose.header.seq = 0
	# 	goal.goal.target_pose.header.stamp = rospy.Time.now()
	# 	goal.goal.target_pose.header.frame_id = 'map'
	# 	goal.goal.target_pose.pose.position.x = new_x
	# 	goal.goal.target_pose.pose.position.y = new_y
	# 	goal.goal.target_pose.pose.orientation.w = 1
	# 	self.publisher_GOAL.publish(goal)

