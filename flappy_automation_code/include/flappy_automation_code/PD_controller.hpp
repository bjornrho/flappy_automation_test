#ifndef PD_CONTROLLER_H_
#define PD_CONTROLLER_H_
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"


class PDController
{
public:
    PDController(ros::NodeHandlePtr nh);
    ~PDController();

private:
    const float KP = 30;
    const float KD = 10;
    const float T = 1.0/30.0;

    ros::Subscriber sub_vel;
    ros::Subscriber sub_ref_pos;
    ros::Publisher pub_acc;
    
    geometry_msgs::Vector3 flappy_reference_position;
    geometry_msgs::Vector3 flappy_position;
    geometry_msgs::Vector3 flappy_velocity;
    geometry_msgs::Vector3 previous_error_value;
    geometry_msgs::Vector3 acc_cmd;

    void refPosSub(const geometry_msgs::Vector3::ConstPtr& msg);
    void velSubPDPub(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif