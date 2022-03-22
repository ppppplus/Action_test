#include <ros/ros.h>
#include <move_base_msgs/RecoveryStatus.h>
#include "action_panda/health_status.h"
#include "segway_msgs/bms_fb.h"

class HealthStatusPublish
{
    private:
        // ros::NodeHandle n_;
        ros::Publisher health_status_pub;   // 发布健康状态
        ros::Subscriber sub_bat;    // 订阅电池电量
        ros::Subscriber sub_pos;    // 订阅位置信息
        action_panda::health_status hs_;    // 健康状态msg
    public:
        ros::NodeHandle n_;

        long long getCurrentTime()
        {
            struct timeval tv;
            gettimeofday(&tv,NULL);
            // printf("tv:%lld,%ld\n",(long long)tv.tv_sec*1000,tv.tv_usec/1000);
            return (long long)tv.tv_sec * 1000 + (long long)tv.tv_usec / 1000;
        }

        HealthStatusPublish()
        {
            health_status_pub = n_.advertise<action_panda::health_status>("health_status", 1);
            sub_bat = n_.subscribe("bms_fb", 1, &HealthStatusPublish::batCallback, this);
            sub_pose = n_.subscribe("move_base/feedback", 1, &HealthStatusPublish::posCallback, this);
        }
        
        void batCallback(const segway_msgs::bms_fb& msg)
        {
            hs_.electricQuantity = string(msg->bat_soc);
        }

        void posCallback(const move_base_msgs::MoveBaseFeedback& msg)
        {
            hs_.indoorX = msg->feedback.base_position.pose.position.posX;
            hs_.indoorY = msg->feedback.base_position.pose.position.posY;
            hs_.actionTime = getCurrentTime()
            health_status_pub.publish(hs_)
        }
}       

int main(int argc, char **argv)
{
	ros::init(argc, argv, "health_status_publisher");
    HealthStatusPublish hsp();

	ros::spin();
	return 0;
}