#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_approachAction.h"
#include <stdlib.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<action_panda::box_approachAction> Server;

#define ALIGNMENT_TOLERANCE_LR 0.02
#define ALIGNMENT_TOLERANCE_FB 0.2
#define CONFIRMATION_TIMES 10
#define PREDIS 1.0

int first_rot;
float goal_x, goal_y, goal_yaw;
ros::Publisher cmd_pub;

double* QUT2RPY(float x, float y, float z, float w)
{
  double* rpy = new double[3];
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  m.getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

void spin_around(int n)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0.1*n;
  cmd_pub.publish(cmd_vel);
}

bool send_goal(float x, float z, float pitch)
{
  MoveBaseClient ac("panda/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }
  float goal_x, goal_y;
  goal_x = z-cos(pitch)*PREDIS;
  goal_y = -(x-sin(pitch)*PREDIS);
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "panda/base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal [x:%f];[y:%f]", goal_x, goal_y);
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved 1 meter forward");
    return true;
  }
  else
  {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    return false;
  }
}


void PoseCallback(geometry_msgs::PoseStamped poseMsg)
{
  float x, y, z;
  x = poseMsg.pose.position.x;
  y = poseMsg.pose.position.y;
  z = poseMsg.pose.position.z;
    
  
  double* rpy;
  rpy = QUT2RPY(poseMsg.pose.orientation.x,
        poseMsg.pose.orientation.y,
        poseMsg.pose.orientation.z,
        poseMsg.pose.orientation.w);
  ROS_INFO("x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f", x, y, z, rpy[0], rpy[1], rpy[2]);
  goal_x = x;
  goal_y = z;
  goal_yaw = rpy[1];
  first_rot = x>0?-1:1;
  // if (arr_flag == true)
  //   fine_tuning_alignment(first_rot, x, z);
  // else
  // {
  //   first_rot = x>0?-1:1;
  //   send_goal(x, z, rpy[1]);
  // }

}


void MySigintHandler(int sig)
{
    ROS_INFO("shut down!");
    spin_around(0);
    ros::shutdown();
    exit(0);
}

void execute(const action_panda::box_approachGoalConstPtr& goal, Server* as)
{
  if(goal->car_id == 1)
  {
    ROS_INFO("app car_id:%d", goal->car_id);
    // system("roslaunch segwayrmp carto.launch");
    // system("roslaunch segwayrmp move_base_teb.launch");
    system("roslaunch aruco_ros single.launch");
    action_panda::box_approachFeedback feedback;
    action_panda::box_approachResult result;
    feedback.detect_flag = 1;
    as->publishFeedback(feedback);
    feedback.mb_flag = send_goal(goal_x, goal_y, goal_yaw);
    as->publishFeedback(feedback);
    result.first_rot = first_rot;
    ROS_INFO("Car %d finish approaching.", goal->car_id);
    as->setSucceeded(result);
  }
}

int main(int argc, char **argv)
{
	
	  ros::init(argc, argv, "box_approach_server");
	  ros::NodeHandle n;
    signal(SIGINT, MySigintHandler);

    // action //
     // 定义一个服务器
    Server server(n, "box_approach", boost::bind(&execute, _1, &server), false);
    ros::Subscriber pose_sub = n.subscribe("/aruco_single/pose", 10, PoseCallback);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/panda/cmd_vel", 1);
    // 服务器开始运行
    server.start();
	  ros::spin();
	  return 0;
}
