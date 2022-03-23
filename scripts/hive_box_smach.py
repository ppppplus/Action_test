#!/usr/bin/env python3


import rospy  
import smach
import time

import std_srvs.srv 
import turtlesim.srv
import smach_ros
from smach_ros import ServiceState
from smach_ros import SimpleActionState
from smach_ros import MonitorState
import actionlib
from actionlib_msgs.msg import *

from action_panda.msg import box_approachAction, box_approachGoal, box_approachResult, box_approachFeedback #导入自定义动作
from action_panda.msg import box_focousAction, box_focousGoal, box_focousResult, box_focousFeedback #导入自定义动作
from action_panda.srv import *

def CurrentTimeMillis():
    return int(round(time.time()*1000))

def main():
	rospy.init_node('smach_usecase_executive')
	
	sm_root = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

	with sm_root:

		def task_info_request_cb(userdata, request):
			task_info_request = task_infoRequest()
			task_info_request.ready = True
			return task_info_request
		
		def task_info_response_cb(userdata, response):
			if response.type == "delivery":
				userdata.picX = response.indoorX
				userdata.picY = response.indoorY	# 均为像素坐标
				rospy.loginfo("Receive pic point:x[%s] y[%s]", userdata.picX, userdata.picY)
				return 'succeeded'
			else:
				rospy.logwarn("Cannot receive goal!")
				return 'aborted'

		def box_approach_result_cb(userdata, status, result):
			if status == GoalStatus.SUCCEEDED:
				userdata.first_rot_output = result.first_rot
				return 'succeeded'

		def box_focous_goal_cb(userdata, goal):
			box_focous_goal = box_focousGoal()
			box_focous_goal.car_id = 1
			box_focous_goal.first_rot = userdata.first_rot_input
			return box_focous_goal

		def box_open_request_cb(userdata, request):
			box_open_request = box_controlRequest()
			box_open_request.actionTime = CurrentTimeMillis()
			box_open_request.deviceId = "1"
			box_open_request.type = "open"
			return box_open_request

		def outside_nav_request_cb(userdata, request):
			outside_nav_request = outside_navRequest()
			outside_nav_request.car_id = 1
			return outside_nav_request

		def elevator_request_cb(userdata, request):
			elevator_request = elevatorRequest()
			elevator_request.car_id = 1
			return elevator_request

		def inside_nav_request_cb(userdata, request):
			inside_nav_request = inside_navRequest()
			inside_nav_request.car_id = 1
			inside_nav_request.pic_x = userdata.picX
			inside_nav_request.pic_y = userdata.picY
			return inside_nav_request

		smach.StateMachine.add('DELIVERY_READY',
			ServiceState('task_info',
			task_info,
			request_cb = task_info_request_cb,
			result_cb = task_info_response_cb),
			transitions={'succeeded':'BOX_APPROACH', 'aborted':'aborted', 'preempted':'DELIVERY_READY'}
			)
        
		# smach.StateMachine.add('BOX_APPROACH', 
		# SimpleActionState('box_approach',box_approachAction,
		#  goal=box_approachGoal(car_id=1),
		#  result_cb=box_approach_result_cb, 
		#  output_keys=['first_rot_output']),
		# transitions={'succeeded': 'BOX_FOCOUS'},
		# remapping={'first_rot_output':'first_rot'})

		# smach.StateMachine.add('BOX_FOCOUS',
		# SimpleActionState('box_focous', box_focousAction,
		# #  goal=box_focousGoal(car_id=1),
		#  goal_cb=box_focous_goal_cb,
		#  input_keys=['first_rot_input']),
		# transitions={'succeeded':'OUTSIDE_NAV_RESET'},
		# remapping={'first_rot_input':'first_rot'})

		smach.StateMachine.add('BOX_OPEN',
			ServiceState('box_control',
			box_control,
			request_cb = box_open_request_cb),
			transitions={'succeeded':'OUTSIDE_NAV'}
			)

		smach.StateMachine.add('OUTSIDE_NAV',
			ServiceState('outside_nav',
			outside_nav,
			request_cb = outside_nav_request_cb),
			transitions={'succeeded':'ELEVATOR'}
		 	)

		smach.StateMachine.add('ELEVATOR',
			ServiceState('elevator',
			elevator,
			request_cb = elevator_request_cb),
			transitions={'succeeded':'INSIDE_NAV'}
			)

		smach.StateMachine.add('INSIDE_NAV',
			ServiceState('inside_nav',
			inside_nav,
			request_cb = inside_nav_request_cb,
			input_keys=['picX', 'picY']),
			transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'}
			)

	# start  introspection server to use smach_viewr.p
	sis = smach_ros.IntrospectionServer('server_name',sm_root,'/SM_ROOT')
	sis.start()

	outcome = sm_root.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()

