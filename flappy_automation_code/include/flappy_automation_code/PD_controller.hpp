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

    ros::Subscriber sub_vel;
    ros::Subscriber sub_ref_pos;
    ros::Publisher pub_acc;
    
    geometry_msgs::Vector3 flappy_reference_position;

    float YPositionControl(geometry_msgs::Vector3 velocity_measurement);
    void refPosSub(const geometry_msgs::Vector3::ConstPtr& msg);
    void velSubPDPub(const geometry_msgs::Vector3::ConstPtr& msg);
    
};

#endif