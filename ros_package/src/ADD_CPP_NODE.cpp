#include "PACKAGE_NAME/ADD_CPP_NODE.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ADD_CPP_NODE");

  PACKAGE_NAME::ADD_CPP_NODE__PASCAL node;
  ros::spin();

  return 0;
}
