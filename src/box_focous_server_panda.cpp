#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_focousAction.h"



#define ALIGNMENT_TOLERANCE_LR 0.02
#define ALIGNMENT_TOLERANCE_FB 0.2
#define CONFIRMATION_TIMES 10
#define PREDIS 1.0

class BoxFocousAction
{
  protected:
    bool if_focous = false;
    int first_rot = 1;
    std::string action_name;

    ros::NodeHandle n;
    ros::Publisher cmd_pub;
    ros::Subscriber pose_sub;

    actionlib::SimpleActionServer<action_panda::box_focousAction> as_;
    action_panda::box_focousFeedback feedback_;

  public:
    BoxFocousAction(std::string name):
     as_(n, name, boost::bind(&BoxFocousAction::execute, this, _1), false),
     action_name(name)
    {
      as_.start();
    }

    ~BoxFocousAction(void)
    {}

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

    void moving_forward(float z)
    {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0;
      cmd_pub.publish(cmd_vel);
      ros::Duration(5*z).sleep();
      cmd_vel.linear.x = 0;
      cmd_pub.publish(cmd_vel);
    }

    void fine_tuning_alignment(int first_rot, float x, float z)
    {
      if(first_rot != 0)
      {
        spin_around(first_rot);
        first_rot = 0;
      }   

      if (x > ALIGNMENT_TOLERANCE_LR)
      {
        spin_around(-1);
      }
      else if (x < -ALIGNMENT_TOLERANCE_LR)
      {
        spin_around(1);
      }
      else
      {
        spin_around(0);
        ROS_INFO("Confirm that the alignment of L&R is completed");
        moving_forward(z);
        ROS_INFO("Confirm that the alignment of F&B is completed");
        if_focous = true;
      }
    }

    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
    {
      float x, y, z;
      x = poseMsg->pose.position.x;
      y = poseMsg->pose.position.y;
      z = poseMsg->pose.position.z;


      double* rpy;
      rpy = QUT2RPY(poseMsg->pose.orientation.x,
            poseMsg->pose.orientation.y,
            poseMsg->pose.orientation.z,
            poseMsg->pose.orientation.w);
      ROS_INFO("x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f", x, y, z, rpy[0], rpy[1], rpy[2]);

      fine_tuning_alignment(first_rot, x, z);

    }

    void execute(const action_panda::box_focousGoalConstPtr& goal)
    {
      cmd_pub = n.advertise<geometry_msgs::Twist>("/panda/cmd_vel", 1);
	    pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 1, boost::bind(&BoxFocousAction::PoseCallback,this,_1));
      ROS_INFO("foc car_id:%d", goal->car_id);
      first_rot = goal->first_rot;
      if(if_focous==1)
      {
        ROS_INFO("Car %d finish focousing.", goal->car_id);
        as_.setSucceeded();
        ros::shutdown();
      }
    }


};

// void MySigintHandler(int sig)
// {
//     ROS_INFO("shut down!");
//     spin_around(0);
//     ros::shutdown();
//     exit(0);
// }

int main(int argc, char **argv)
{
	  ros::init(argc, argv, "box_focous_server");
    // signal(SIGINT, MySigintHandler);
    BoxFocousAction boxfocousaction("box_focous");

	  ros::spin();
	  return 0;
}
