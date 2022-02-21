#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <ublox_msg_filters/exact_time.h>
#include <ublox_msgs/NavHPPOSLLH.h>
#include <ublox_msgs/NavRELPOSNED9.h>
#include <ublox_msgs/NavVELNED.h>

void callback(const ublox_msgs::NavHPPOSLLHConstPtr &msg1,
              const ublox_msgs::NavRELPOSNED9ConstPtr &msg2,
              const ublox_msgs::NavVELNEDConstPtr &msg3) {
  ROS_INFO("RX %u %u %u", msg1->iTOW, msg2->iTOW, msg3->iTOW);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ublox_sync");

  ros::NodeHandle nh;
  message_filters::Subscriber<ublox_msgs::NavHPPOSLLH> sub1(nh, "msg1", 1);
  message_filters::Subscriber<ublox_msgs::NavRELPOSNED9> sub2(nh, "msg2", 1);
  message_filters::Subscriber<ublox_msgs::NavVELNED> sub3(nh, "msg3", 1);

  typedef ublox_msg_filters::ExactTime<ublox_msgs::NavHPPOSLLH, ublox_msgs::NavRELPOSNED9, ublox_msgs::NavVELNED> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
  sync.registerCallback(boost::bind(callback, _1, _2, _3));

  ROS_INFO("Ready to receive");

  ros::spin();

  return 0;
}
