#!/usr/bin/env python3
# encoding: utf-8

import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import *
import time

class GoalPub():
    def __init__(self, robot_name, x, y, yaw):
        self.robot_name = robot_name
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)
        
        time.sleep(8)
        # 发布第一个目标点
        first_pose = PoseStamped()
        first_pose.header.frame_id = self.robot_name + '/map'
        first_pose.header.stamp = rospy.Time.now()
        first_pose.pose.position.x = self.goal_x
        first_pose.pose.position.y = self.goal_y
        first_pose.pose.orientation.z = math.sin(self.goal_yaw/2) # z=sin(yaw/2)
        first_pose.pose.orientation.w = math.cos(self.goal_yaw/2) # w=cos(yaw/2)
        self.goal_pub.publish(first_pose)
        rospy.loginfo("goal send!!!x:[%f] y[%f] yaw[%f]", self.goal_x, self.goal_y, self.goal_yaw)
        # goal_status_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.pose_callback) #用于订阅是否到达目标点状态
 
if __name__ == '__main__':
    rospy.init_node('goal_pub_node') #初始化节点
    robot_name = rospy.get_param('~robot_name', 'AI_1')
    goal_x = rospy.get_param('~goal_x', 0)
    goal_y = rospy.get_param('~goal_y', 0)
    goal_yaw = rospy.get_param('~goal_yaw', 0)
    rospy.logwarn("robot_name:%s", robot_name)
    node = GoalPub(robot_name, goal_x, goal_y, goal_yaw)
    rospy.spin()