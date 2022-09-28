#include "ros/ros.h"
#include "flappy_automation_code/PD_controller.hpp"
#include "flappy_automation_code/basic_perception.hpp"


int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  auto node_handle_ptr = ros::NodeHandlePtr{new ros::NodeHandle};
  auto pd_node = PDController{node_handle_ptr};
  auto bv_node = BasicPerception{node_handle_ptr};
  ros::spin();
  return 0;
}
