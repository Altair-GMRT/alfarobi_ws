#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_station_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
