#include "PACKAGE_NAME/ADD_CPP_NODE.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  // Start your custom node (node logic should be added to the ADD_CPP_NODE__PASCAL 
  // class in the PACKAGE_NAME library, not here)
  ros::init(argc, argv, "ADD_CPP_NODE");
  PACKAGE_NAME::ADD_CPP_NODE__PASCAL node;
  ros::spin();

  return 0;
}
