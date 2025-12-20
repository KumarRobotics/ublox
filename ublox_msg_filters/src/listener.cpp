#include <rclcpp/rclcpp.hpp>
#include <ublox_msgs/msg/nav_posllh.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>
#include <ublox_msg_filters/exact_time.h>
#include <message_filters/subscriber.h>

class Listener : public rclcpp::Node {
public:
  Listener() : Node("listener"),
    sub1_(this, "msg1"),
    sub2_(this, "msg2"),
    sub3_(this, "msg3"),
    sync_(MySyncPolicy(10), sub1_, sub2_, sub3_)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    sync_.registerCallback(std::bind(&Listener::callback, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Ready to receive");
  }

private:
  void callback(const ublox_msgs::msg::NavPOSLLH::ConstSharedPtr msg1,
                const ublox_msgs::msg::NavRELPOSNED9::ConstSharedPtr msg2,
                const ublox_msgs::msg::NavVELNED::ConstSharedPtr msg3) {
    RCLCPP_INFO(this->get_logger(), "RX %u %u %u", msg1->i_tow, msg2->i_tow, msg3->i_tow);
  }

private:
  message_filters::Subscriber<ublox_msgs::msg::NavPOSLLH> sub1_;
  message_filters::Subscriber<ublox_msgs::msg::NavRELPOSNED9> sub2_;
  message_filters::Subscriber<ublox_msgs::msg::NavVELNED> sub3_;
  typedef ublox_msg_filters::sync_policies::ExactTime<ublox_msgs::msg::NavPOSLLH,
                                                      ublox_msgs::msg::NavRELPOSNED9,
                                                      ublox_msgs::msg::NavVELNED> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
