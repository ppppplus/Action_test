#!/usr/bin/env python3


import rospy  
import smach

import std_srvs.srv 
import turtlesim.srv
import smach_ros
from smach_ros import ServiceState
from smach_ros import SimpleActionState
import turtle_actionlib.msg
import turtlesim.msg
import actionlib
from actionlib_msgs.msg import *

from action_panda.msg import box_approachAction, box_approachGoal, box_approachResult, box_approachFeedback #导入自定义动作
from action_panda.msg import box_focousAction, box_focousGoal, box_focousResult, box_focousFeedback #导入自定义动作


def main():
	rospy.init_node('smach_usecase_executive')
	
	sm_root = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

	with sm_root:
		def box_approach_result_cb(userdata, status, result):
			if status == GoalStatus.SUCCEEDED:
				userdata.first_rot_output = result.first_rot
				return 'succeeded'

		def box_focous_goal_cb(userdata, goal):
			box_focous_goal = box_focousGoal()
			box_focous_goal.car_id = 2
			box_focous_goal.first_rot = userdata.first_rot_input
			return box_focous_goal
			

		smach.StateMachine.add('BOX_APPROACH', 
		SimpleActionState('box_approach',box_approachAction,
		 goal=box_approachGoal(car_id=1),
		 result_cb=box_approach_result_cb, 
		 output_keys=['first_rot_output']),
		transitions={'succeeded': 'BOX_FOCOUS'},
		remapping={'first_rot_output':'first_rot'})

		smach.StateMachine.add('BOX_FOCOUS',
		SimpleActionState('box_focous', box_focousAction,
		#  goal=box_focousGoal(car_id=1),
		 goal_cb=box_focous_goal_cb,
		 input_keys=['first_rot_input']),
		transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'},
		remapping={'first_rot_input':'first_rot'})
	
	# start  introspection server to use smach_viewr.p
	sis = smach_ros.IntrospectionServer('server_name',sm_root,'/SM_ROOT')
	sis.start()

	outcome = sm_root.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()

