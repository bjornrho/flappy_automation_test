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
    float WALL_WIDTH = 0.4;

    ros::Subscriber sub_laser_scan;
    ros::Publisher pub_ref_pos;
    ros::Publisher pub_acc;

    sensor_msgs::LaserScan flappy_laser_scan;

    void laserSubRefencePub(const sensor_msgs::LaserScan::ConstPtr& msg);
    float estimateClosestWallPosition();
    float estimateReferencePositionY(const float wall_estimate);
};

#endif