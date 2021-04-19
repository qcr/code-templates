#include "ros/ros.h"
#include "std_msgs/String.h"

namespace PACKAGE_NAME {

class ADD_CPP_NODE__PASCAL {
 public:
  ADD_CPP_NODE__PASCAL();

 private:
  void callback(const std_msgs::String &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
};

}  // namespace PACKAGE_NAME
