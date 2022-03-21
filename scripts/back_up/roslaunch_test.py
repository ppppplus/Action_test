#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import roslaunch
import rospy
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from action_panda.srv import *
import os
import subprocess
from subprocess import Popen

def inside_navCallback(req):
	# 显示请求数据
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nics/panda_ws/src/segwayrmp/launch/trans/move_base_map.launch"])
    # launch.start()
    #subprocess.run (["roslaunch action_panda turtlesim.launch"],shell=False)  
    # r_ph = Popen(["roslaunch action_panda turtlesim.launch"])
    # p = subprocess.Popen(['gnome-terminal', '-x', 'bash',  '-c', 'roslaunch action_panda turtlesim.launch'])
    p = subprocess.Popen(['bash',  '-c', 'roslaunch action_panda turtlesim.launch'])
    rospy.loginfo("************************************started")
    # move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    # rospy.loginfo("Waiting for move_base action server...")  
  
    # # Wait 60 seconds for the action server to become available  
    # move_base.wait_for_server(rospy.Duration(60)) 
    # rospy.loginfo("move_base start!!!")
    # finished_time = move_base.wait_for_result(rospy.Duration(300))
    # if not finished_time:
    #     move_base.cancel_goal()  
    #     rospy.loginfo("Timed out achieving goal")  
    # else:  
    #     state = move_base.get_state()  
    #     if state == GoalStatus.SUCCEEDED:  
    #         rospy.loginfo("Goal succeeded!")  
    #     else:  
    #         rospy.loginfo("Goal failed with error code: " + str(goal_states[state])) 
	# 反馈数据
    rospy.sleep(5)
    p.terminate()
    rospy.loginfo("************************************close")
    return inside_navResponse(True)

def inside_nav_server():
	# ROS节点初始化
    rospy.init_node('inside_nav_server_node')
    s = rospy.Service('/inside_nav', inside_nav, inside_navCallback)

    rospy.spin()

if __name__ == "__main__":
    inside_nav_server()
