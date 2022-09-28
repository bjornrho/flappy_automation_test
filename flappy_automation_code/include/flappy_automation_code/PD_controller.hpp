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
    const float KP_POS = 30;
    const float KD_POS = 10;
    const float KP_VEL = 30;
    const float KD_VEL = 10;
    const float T = 1.0/30.0;

    ros::Subscriber sub_vel;
    ros::Subscriber sub_ref_pos;
    ros::Publisher pub_acc;
    
    float flappy_reference_position_y;
    float flappy_reference_velocity_x;
    float flappy_previous_velocity_x;

    float YPositionControl(geometry_msgs::Vector3 velocity_measurement);
    float XVelocityControl(geometry_msgs::Vector3 velocity_measurement);
    void refPosSub(const geometry_msgs::Vector3::ConstPtr& msg);
    void velSubPDPub(const geometry_msgs::Vector3::ConstPtr& msg);
    
};

#endif