#include <ros/ros.h>
#include "planning_image_drawing/set_img_flag.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_flag");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<planning_image_drawing::set_img_flag>("/set_flag");
  planning_image_drawing::set_img_flag srv;
  srv.request.set_image_flag_time = ros::Time::now();
  if (client.call(srv))
  {
    std::cout << (bool)srv.response.set_success << "\n";
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}