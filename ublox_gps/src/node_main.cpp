#include <ros/ros.h>

#include <ublox_gps/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  ublox_node::UbloxNode node;

  ros::spin();

  return 0;
}
