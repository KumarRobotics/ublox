#ifndef UBLOX_GPS_TIM_PRODUCT_HPP
#define UBLOX_GPS_TIM_PRODUCT_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <ublox_msgs/msg/rxm_sfrbx.hpp>
#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for Time Sync products.
 * @todo partially implemented
 */
class TimProduct final : public virtual ComponentInterface {
 public:
  explicit TimProduct(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node);

  /**
   * @brief Get the Time Sync parameters.
   * @todo Currently unimplemented.
   */
  void getRosParams() override;

  /**
   * @brief Configure Time Sync settings.
   * @todo Currently unimplemented.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Adds diagnostic updaters for Time Sync status.
   * @todo Currently unimplemented.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Time Sync messages.
   *
   * @details Subscribes to RxmRAWX & RxmSFRBX messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  /**
   * @brief
   * @details Publish recieved TimTM2 messages if enabled
   */
  void callbackTimTM2(const ublox_msgs::msg::TimTM2 &m);

  sensor_msgs::msg::TimeReference t_ref_;

  rclcpp::Publisher<ublox_msgs::msg::TimTM2>::SharedPtr timtm2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr interrupt_time_pub_;
  rclcpp::Publisher<ublox_msgs::msg::RxmSFRBX>::SharedPtr rxm_sfrb_pub_;
  rclcpp::Publisher<ublox_msgs::msg::RxmRAWX>::SharedPtr rxm_raw_pub_;

  std::string frame_id_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  rclcpp::Node* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_TIM_PRODUCT_HPP
