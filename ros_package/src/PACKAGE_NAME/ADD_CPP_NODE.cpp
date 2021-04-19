#include "PACKAGE_NAME/ADD_CPP_NODE.h"

namespace PACKAGE_NAME {
ADD_CPP_NODE__PASCAL::ADD_CPP_NODE__PASCAL() : nh_(), nh_private_("~") {
  // Read private parameters using the private node handle
  std::string my_param;
  nh_private_.param("my_topic_name", my_param, std::string("DEFAULT VALUE"));

  // Setup publishers & subscribers
  pub_ = nh_.advertise<std_msgs::String>("my_publisher_topic", 1, false);
  sub_ = nh_.subscribe("my_subscriber_topic", 1,
                       &ADD_CPP_NODE__PASCAL::callback, this);
}

void ADD_CPP_NODE__PASCAL::callback(const std_msgs::String& msg) {
  ROS_INFO("Received message: %s", msg.data.c_str());
}

void ADD_CPP_NODE__PASCAL::spin() {
  ROS_INFO("Publishing on '%s' & listening on '%s'", pub_.getTopic().c_str(),
           sub_.getTopic().c_str());
  ros::Rate r(1);
  while (!ros::isShuttingDown()) {
    std_msgs::String msg;
    msg.data = "tic";
    pub_.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
}

}  // namespace PACKAGE_NAME
