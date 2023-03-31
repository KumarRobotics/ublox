#ifndef UBLOX_GPS_UBLOX_FIRMWARE7PLUS_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE7PLUS_HPP

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <ublox_msgs/msg/nav_pvt.hpp>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

/**
 * @brief Abstract class for Firmware versions >= 7.
 *
 * @details This class keeps track of the last NavPVT message uses it to
 * update the fix diagnostics. It is a template class because the NavPVT message
 * is a different length for firmware versions 7 and 8.
 *
 * @typedef NavPVT the NavPVT message type for the given firmware version
 */
template<typename NavPVT>
class UbloxFirmware7Plus : public UbloxFirmware {
 public:
  explicit UbloxFirmware7Plus(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node)
    : UbloxFirmware(updater, gnss, node), frame_id_(frame_id), freq_diag_(freq_diag) {
    // NavPVT publisher
    if (getRosBoolean(node_, "publish.nav.pvt")) {
      nav_pvt_pub_ = node_->create_publisher<NavPVT>("~/navpvt", 1);
    }

    fix_pub_ =
        node_->create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 1);
    vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/fix_velocity",
                                                                   1);
  }

  /**
   * @brief Publish a NavSatFix and TwistWithCovarianceStamped messages.
   *
   * @details If a fixed carrier phase solution is available, the NavSatFix
   * status is set to GBAS fixed. If NavPVT publishing is enabled, the message
   * is published. This function also calls the ROS diagnostics updater.
   * @param m the message to publish
   */
  void callbackNavPvt(const NavPVT& m) {
    if (getRosBoolean(node_, "publish.nav.pvt")) {
      // NavPVT publisher
      nav_pvt_pub_->publish(m);
    }

    //
    // NavSatFix message
    //
    sensor_msgs::msg::NavSatFix fix;
    fix.header.frame_id = frame_id_;
    // set the timestamp
    uint8_t valid_time = m.VALID_DATE | m.VALID_TIME | m.VALID_FULLY_RESOLVED;
    if (((m.valid & valid_time) == valid_time) &&
        (m.flags2 & m.FLAGS2_CONFIRMED_AVAILABLE)) {
      // Use NavPVT timestamp since it is valid
      // The time in nanoseconds from the NavPVT message can be between -1e9 and 1e9
      //  The ros time uses only unsigned values, so a negative nano seconds must be
      //  converted to a positive value
      if (m.nano < 0) {
        fix.header.stamp.sec = toUtcSeconds(m) - 1;
        fix.header.stamp.nanosec = static_cast<uint32_t>(m.nano + 1e9);
      }
      else {
        fix.header.stamp.sec = toUtcSeconds(m);
        fix.header.stamp.nanosec = static_cast<uint32_t>(m.nano);
      }
    } else {
      // Use ROS time since NavPVT timestamp is not valid
      fix.header.stamp = node_->now();
    }
    // Set the LLA
    fix.latitude = m.lat * 1e-7; // to deg
    fix.longitude = m.lon * 1e-7; // to deg
    fix.altitude = m.height * 1e-3; // to [m]
    // Set the Fix status
    bool fixOk = m.flags & m.FLAGS_GNSS_FIX_OK;
    if (fixOk && m.fix_type >= m.FIX_TYPE_2D) {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      if (m.flags & m.CARRIER_PHASE_FIXED) {
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      }
    } else {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    // Set the service based on GNSS configuration
    fix.status.service = fix_status_service_;

    // Set the position covariance
    const double var_h = pow(m.h_acc / 1000.0, 2); // to [m^2]
    const double var_v = pow(m.v_acc / 1000.0, 2); // to [m^2]
    fix.position_covariance[0] = var_h;
    fix.position_covariance[4] = var_h;
    fix.position_covariance[8] = var_v;
    fix.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    fix_pub_->publish(fix);

    //
    // Twist message
    //
    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    velocity.header.stamp = fix.header.stamp;
    velocity.header.frame_id = frame_id_;

    // convert to XYZ linear velocity [m/s] in ENU
    velocity.twist.twist.linear.x = m.vel_e * 1e-3;
    velocity.twist.twist.linear.y = m.vel_n * 1e-3;
    velocity.twist.twist.linear.z = -m.vel_d * 1e-3;
    // Set the covariance
    const double cov_speed = pow(m.s_acc * 1e-3, 2);
    const int cols = 6;
    velocity.twist.covariance[cols * 0 + 0] = cov_speed;
    velocity.twist.covariance[cols * 1 + 1] = cov_speed;
    velocity.twist.covariance[cols * 2 + 2] = cov_speed;
    velocity.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

    vel_pub_->publish(velocity);

    //
    // Update diagnostics
    //
    last_nav_pvt_ = m;
    freq_diag_->diagnostic->tick(fix.header.stamp);
  }

 protected:

  /**
   * @brief Update the fix diagnostics from Nav PVT message.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) override {
    // check the last message, convert to diagnostic
    if (last_nav_pvt_.fix_type ==
        ublox_msgs::msg::NavPVT::FIX_TYPE_DEAD_RECKONING_ONLY) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "Dead reckoning only";
    } else if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_2D) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "2D fix";
    } else if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_3D) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "3D fix";
    } else if (last_nav_pvt_.fix_type ==
               ublox_msgs::msg::NavPVT::FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "GPS and dead reckoning combined";
    } else if (last_nav_pvt_.fix_type ==
               ublox_msgs::msg::NavPVT::FIX_TYPE_TIME_ONLY) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Time only fix";
    }

    // If fix not ok (w/in DOP & Accuracy Masks), raise the diagnostic level
    if (!(last_nav_pvt_.flags & ublox_msgs::msg::NavPVT::FLAGS_GNSS_FIX_OK)) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message += ", fix not ok";
    }
    // Raise diagnostic level to error if no fix
    if (last_nav_pvt_.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_NO_FIX) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "No fix";
    }

    // append last fix position
    stat.add("iTOW [ms]", last_nav_pvt_.i_tow);
    stat.add("Latitude [deg]", last_nav_pvt_.lat * 1e-7);
    stat.add("Longitude [deg]", last_nav_pvt_.lon * 1e-7);
    stat.add("Altitude [m]", last_nav_pvt_.height * 1e-3);
    stat.add("Height above MSL [m]", last_nav_pvt_.h_msl * 1e-3);
    stat.add("Horizontal Accuracy [m]", last_nav_pvt_.h_acc * 1e-3);
    stat.add("Vertical Accuracy [m]", last_nav_pvt_.v_acc * 1e-3);
    stat.add("# SVs used", static_cast<int>(last_nav_pvt_.num_sv));
  }

  //! The last received NavPVT message
  NavPVT last_nav_pvt_;
  // Whether or not to enable the given GNSS
  //! Whether or not to enable GPS
  bool enable_gps_{false};
  //! Whether or not to enable GLONASS
  bool enable_glonass_{false};
  //! Whether or not to enable QZSS
  bool enable_qzss_{false};
  //! The QZSS Signal configuration, see CfgGNSS message
  uint32_t qzss_sig_cfg_{0};

  typename rclcpp::Publisher<NavPVT>::SharedPtr nav_pvt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;

  std::string frame_id_;
  std::shared_ptr<FixDiagnostic> freq_diag_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE7PLUS_HPP
