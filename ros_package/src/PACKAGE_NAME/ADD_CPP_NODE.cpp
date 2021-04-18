#include "PACKAGE_NAME/ADD_CPP_NODE.h"

namespace PACKAGE_NAME {
ADD_CPP_NODE::ADD_CPP_NODE() : nh_(), nh_private_("~") {
  // Read private parameters using the private node handle
  std::string my_param;
  nh_private_.param("my_topic_name", my_param, std::string("DEFAULT VALUE"));

  // Setup publishers & subscribers
  pub_ = nh_.advertise<std_msgs::String>("my_publisher_topic", 1, false);
  sub_ = nh_.subscribe("my_subscriber_topic", 1, &ADD_CPP_NODE::callback, this);
}

void ADD_CPP_NODE::callback(const std_msgs::String& msg) {
  ROS_INFO("Received message: %s", msg.data.c_str());
}

}  // namespace PACKAGE_NAME
