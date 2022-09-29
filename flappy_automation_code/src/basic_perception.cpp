#include "ros/ros.h"
#include "flappy_automation_code/basic_perception.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"

#include <iostream>
#include <vector>
#include <math.h>

/* ====== HELPER FUNCTIONS ====== */
float meanOfVector(std::vector<float> data_vector){
    if(data_vector.empty()){
        return -1;
    }

    float sum = 0.0;
    for(auto data_point : data_vector){
        sum += (float)data_point;
    }

    return sum / data_vector.size();
}

/* ====== BasicPerception CLASS ====== */

BasicPerception::BasicPerception(ros::NodeHandlePtr nh) : 
    sub_laser_scan{nh->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &BasicPerception::laserSubRefencePub, this)},
    pub_ref_pos{nh->advertise<geometry_msgs::Vector3>("/flappy_reference", 1)},
    pub_acc{nh->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)}
{
    previous_wall_position = 0.0;
}
BasicPerception::~BasicPerception(){
}


void BasicPerception::laserSubRefencePub(const sensor_msgs::LaserScan::ConstPtr& msg){
    flappy_laser_scan = *msg;

    float wall_position = estimateClosestWallPosition();
    float ref_pos_y = estimateReferencePositionY(wall_position);
    float ref_vel_x = estimateReferenceVelocityX(wall_position);
    
    geometry_msgs::Vector3 ref = geometry_msgs::Vector3();
    if((wall_position != -1) && (previous_wall_position != -1) && (wall_position - previous_wall_position) > 1.5){
        ref.y = 0.0;
        ref.x = TOP_SPEED;
    }else{
        ref.y = ref_pos_y;
        ref.x = ref_vel_x;
    }

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

    return meanOfVector(valid_x_distance);
}

float BasicPerception::estimateReferencePositionY(float wall_position){
    if (wall_position == -1){
        return 0.0;
    }

    int number_of_rays = flappy_laser_scan.ranges.size();
    std::vector<float> possible_reference_points_y;

    for(int ray_n = 0; ray_n < number_of_rays; ray_n++){
        if (flappy_laser_scan.ranges[ray_n] > (wall_position + WALL_WIDTH)){
            float scan_angle = flappy_laser_scan.angle_min + ray_n*flappy_laser_scan.angle_increment;
            float y_reference = flappy_laser_scan.ranges[ray_n]*sin(scan_angle);
            possible_reference_points_y.push_back(y_reference);
        }
    }

    float mean = meanOfVector(possible_reference_points_y);
    
    return mean;
}

float BasicPerception::estimateReferenceVelocityX(const float wall_position){
    if(flappy_laser_scan.ranges[4] < EMERGENCY_BREAK || flappy_laser_scan.ranges[5] < EMERGENCY_BREAK || flappy_laser_scan.ranges[3] < EMERGENCY_BREAK){
        return 0.0;
    }
    return TOP_SPEED;
}
