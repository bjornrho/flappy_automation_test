#include "ros/ros.h"
#include "flappy_automation_code/PD_controller.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"


PDController::PDController(ros::NodeHandlePtr nh) : 
    sub_vel{nh->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &PDController::velSubPDPub, this)},
    sub_ref_pos{nh->subscribe<geometry_msgs::Vector3>("/flappy_ref_pos", 1, &PDController::refPosSub, this)},
    pub_acc{nh->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)}
{
    flappy_reference_position = geometry_msgs::Vector3();
}
PDController::~PDController()
{
}

float PDController::YPositionControl(geometry_msgs::Vector3 velocity_measurement){
    return KP*flappy_reference_position.y - KD*velocity_measurement.y;
}


void PDController::refPosSub(const geometry_msgs::Vector3::ConstPtr& msg){
    flappy_reference_position = *msg;
}

void PDController::velSubPDPub(const geometry_msgs::Vector3::ConstPtr& msg){
    geometry_msgs::Vector3 acc_cmd = geometry_msgs::Vector3();
    acc_cmd.y = YPositionControl(*msg);

    pub_acc.publish(acc_cmd);
}