#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/node.hpp>

int main(int argc, char** argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<ublox_node::UbloxNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ublox_gps_node"), "Fatal error: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}
