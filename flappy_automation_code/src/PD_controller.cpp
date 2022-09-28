#include "ros/ros.h"
#include "flappy_automation_code/PD_controller.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"


PDController::PDController(ros::NodeHandlePtr nh) : 
    sub_vel{nh->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &PDController::velSubPDPub, this)},
    sub_ref_pos{nh->subscribe<geometry_msgs::Vector3>("/flappy_reference", 1, &PDController::refPosSub, this)},
    pub_acc{nh->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)}
{
    flappy_previous_velocity_x = 0.0;
}

PDController::~PDController(){}

float PDController::YPositionControl(geometry_msgs::Vector3 velocity_measurement){
    return KP_POS*flappy_reference_position_y - KD_POS*velocity_measurement.y;
}

float PDController::XVelocityControl(geometry_msgs::Vector3 velocity_measurement){
    float acceleration_x = (velocity_measurement.x - flappy_previous_velocity_x) / T;
    flappy_previous_velocity_x = velocity_measurement.x;

    return (KP_VEL*flappy_reference_velocity_x - KD_VEL*acceleration_x);
}


void PDController::refPosSub(const geometry_msgs::Vector3::ConstPtr& msg){
    flappy_reference_position_y = msg->y;
    flappy_reference_velocity_x = msg->x;
}

void PDController::velSubPDPub(const geometry_msgs::Vector3::ConstPtr& msg){
    geometry_msgs::Vector3 acc_cmd = geometry_msgs::Vector3();
    acc_cmd.y = YPositionControl(*msg);

    pub_acc.publish(acc_cmd);
}