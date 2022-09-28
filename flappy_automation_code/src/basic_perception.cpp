#include "ros/ros.h"
#include "flappy_automation_code/basic_perception.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

#include <iostream>
#include <vector>
#include <math.h>



BasicPerception::BasicPerception(ros::NodeHandlePtr nh) : 
    sub_laser_scan{nh->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &BasicPerception::laserSubRefencePub, this)},
    pub_ref_pos{nh->advertise<geometry_msgs::Vector3>("/flappy_ref_pos", 1)},
    pub_acc{nh->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)}
{
}
BasicPerception::~BasicPerception(){
}

void BasicPerception::laserSubRefencePub(const sensor_msgs::LaserScan::ConstPtr& msg){
    flappy_laser_scan = *msg;
    float wall_position = estimateClosestWallPosition();
    float pos_ref = estimateReferencePositionY(wall_position);
    geometry_msgs::Vector3 ref = geometry_msgs::Vector3();
    ref.y = pos_ref;
    pub_ref_pos.publish(ref);

}

float BasicPerception::estimateClosestWallPosition(){
    int number_of_rays = flappy_laser_scan.ranges.size();
    std::vector<float> valid_x_distance;

    for(int ray_n = 0; ray_n < number_of_rays; ray_n++){
        if (flappy_laser_scan.intensities[ray_n]){
            float scan_angle = flappy_laser_scan.angle_min + ray_n*flappy_laser_scan.angle_increment;
            float x_distance = flappy_laser_scan.ranges[ray_n]*cos(scan_angle);
            valid_x_distance.push_back(x_distance);
        }
    }

    sort(valid_x_distance.begin(), valid_x_distance.end());
    for(int element_n = valid_x_distance.size(); element_n > 0; element_n--){
        if (abs(valid_x_distance[0] - valid_x_distance[element_n]) > WALL_WIDTH){
            valid_x_distance.pop_back();
        }
    }

    if (valid_x_distance.empty()) {
        return -1;
    }

    float sum = 0.0;
    for (auto i : valid_x_distance) {
        sum += (float)i;
    }
    float avg =  sum / valid_x_distance.size();

    ROS_INFO("Estimated wall placement: %f", avg);
    //std::stringstream ss;
    //ss << "Data Retrieved: \n";
    //std::copy(valid_x_distance.begin(), valid_x_distance.end(), std::ostream_iterator<double>(ss, " "));
    //ss << std::endl;
    //ROS_INFO_STREAM(ss.str());
    

    return avg;
}

float BasicPerception::estimateReferencePositionY(float wall_estimate){
    if (wall_estimate == -1){
        return 0.0;
    }
    int number_of_rays = flappy_laser_scan.ranges.size();
    std::vector<float> possible_reference_points_y;

    for(int ray_n = 0; ray_n < number_of_rays; ray_n++){
        if (flappy_laser_scan.ranges[ray_n] > (wall_estimate + WALL_WIDTH)){
            float scan_angle = flappy_laser_scan.angle_min + ray_n*flappy_laser_scan.angle_increment;
            float y_reference = flappy_laser_scan.ranges[ray_n]*sin(scan_angle);
            possible_reference_points_y.push_back(y_reference);
        }
    }

    if (possible_reference_points_y.empty()) {
        return 0.0;
    }

    float sum = 0.0;
    for (auto i : possible_reference_points_y) {
        sum += (float)i;
    }
    float avg =  sum / possible_reference_points_y.size();

    ROS_INFO("Estimated ref point: %f", avg);
    //std::stringstream ss;
    //ss << "Data Retrieved: \n";
    //std::copy(possible_reference_points_y.begin(), possible_reference_points_y.end(), std::ostream_iterator<double>(ss, " "));
    //ss << std::endl;
    //ROS_INFO_STREAM(ss.str());

    if (flappy_laser_scan.ranges[0] < 0.1 && flappy_laser_scan.ranges[9] < 0.1){
        return 0.0;
    }

    return avg;
}
