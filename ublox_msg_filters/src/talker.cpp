#include <rclcpp/rclcpp.hpp>
#include <ublox_msgs/msg/nav_posllh.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {
    using namespace std::chrono_literals;
    pub1_ = this->create_publisher<ublox_msgs::msg::NavPOSLLH>("msg1", 2);
    pub2_ = this->create_publisher<ublox_msgs::msg::NavRELPOSNED9>("msg2", 2);
    pub3_ = this->create_publisher<ublox_msgs::msg::NavVELNED>("msg3", 2);
    timer_ = this->create_wall_timer(0.2s, std::bind(&Talker::publish, this));
    RCLCPP_INFO(this->get_logger(), "Ready to transmit");
  }

private:
  void publish() {
    RCLCPP_INFO(this->get_logger(), "TX %u %u %u", i_tow_, i_tow_ + 1, i_tow_ + 3);

    auto msg1 = ublox_msgs::msg::NavPOSLLH();
    msg1.i_tow = i_tow_;
    pub1_->publish(msg1);

    auto msg2 = ublox_msgs::msg::NavRELPOSNED9();
    msg2.i_tow = i_tow_ + 1;
    pub2_->publish(msg2);

    auto msg3 = ublox_msgs::msg::NavVELNED();
    msg3.i_tow = i_tow_ + 3;
    pub3_->publish(msg3);

    i_tow_++;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ublox_msgs::msg::NavPOSLLH>::SharedPtr pub1_;
  rclcpp::Publisher<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr pub2_;
  rclcpp::Publisher<ublox_msgs::msg::NavVELNED>::SharedPtr pub3_;
  uint32_t i_tow_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
