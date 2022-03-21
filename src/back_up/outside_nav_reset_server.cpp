#include "ros/ros.h"
#include "action_panda/outside_nav_reset.h"
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_pub;
bool back_flag = false;

bool execute(action_panda::outside_nav_reset::Request  &req,
         action_panda::outside_nav_reset::Response &res)
{
    ROS_INFO("car_id:%d", req.car_id);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -0.3;
    cmd_vel.angular.z = 0;
    cmd_pub.publish(cmd_vel);
    ros::Duration(2).sleep();
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0.5;
    cmd_pub.publish(cmd_vel);
    ros::Duration(8).sleep();
    cmd_vel.angular.z = 0;
    cmd_pub.publish(cmd_vel);
    back_flag = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outside_nav_reset_server");
    ros::NodeHandle n;
    cmd_pub = n.advertise<geometry_msgs::Twist>("/panda/cmd_vel", 1);
    ros::ServiceServer service = n.advertiseService("outside_nav_reset", execute);
    ROS_WARN("Outside_nav_reset_server(action) ready!");
    if(back_flag)
        ros::shutdown();

    ros::spin();

    return 0;
}