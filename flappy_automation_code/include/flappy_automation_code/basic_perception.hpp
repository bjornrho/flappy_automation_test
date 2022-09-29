#ifndef BASIC_PERCEPTION_H_
#define BASIC_PERCEPTION_H_
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"


class BasicPerception
{
public:
    BasicPerception(ros::NodeHandlePtr nh);
    ~BasicPerception();

private:
    float WALL_WIDTH = 1.0;
    float TOP_SPEED = 4.0;
    float EMERGENCY_BREAK = 1.9;

    ros::Subscriber sub_laser_scan;
    ros::Publisher pub_ref_pos;
    ros::Publisher pub_acc;

    sensor_msgs::LaserScan flappy_laser_scan;
    float flappy_reference_position_y;
    float previous_wall_position;

    void laserSubRefencePub(const sensor_msgs::LaserScan::ConstPtr& msg);
    float estimateClosestWallPosition();
    float estimateReferencePositionY(const float wall_position);
    float estimateReferenceVelocityX(const float wall_position);
};

#endif