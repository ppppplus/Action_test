#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include <move_base_msgs/RecoveryStatus.h>
#include "action_panda/running_status.h"

class RunningStatusPublish
{
    private:
        // ros::NodeHandle n_;
        ros::Publisher running_status_pub;   // 发布运行状态
        ros::Subscriber sub_status;    // 订阅状态类型
        ros::Subscriber sub_pos;    // 订阅位置信息
        action_panda::running_status rs_;    // 运行状态msg
    public:
        ros::NodeHandle n_;

        long long getCurrentTime()
        {
            struct timeval tv;
            gettimeofday(&tv,NULL);
            // printf("tv:%lld,%ld\n",(long long)tv.tv_sec*1000,tv.tv_usec/1000);
            return (long long)tv.tv_sec * 1000 + (long long)tv.tv_usec / 1000;
        }

        RunningStatusPublish()
        {
            running_status_pub = n_.advertise<action_panda::running_status>("running_status", 1);
            sub_status = n_.subscribe("statusType", 1, &RunningStatusPublish::statusCallback, this);
            sub_pose = n_.subscribe("move_base/feedback", 1, &RunningStatusPublish::posCallback, this);
        }
        
        void statusCallback(const std::int& msg)
        {
            rs_.statusType = msg;
        }

        void posCallback(const move_base_msgs::MoveBaseFeedback& msg)
        {
            rs_.indoorX = msg->feedback.base_position.pose.position.posX;
            rs_.indoorY = msg->feedback.base_position.pose.position.posY;
            rs_.actionTime = getCurrentTime()
            running_status_pub.publish(rs_)
        }
}       

int main(int argc, char **argv)
{
	ros::init(argc, argv, "running_status_publisher");
    RunningStatusPublish rsp();

	ros::spin();
	return 0;
}