#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/mon_hw6.hpp>
#include <ublox_msgs/msg/nav_posllh.hpp>
#include <ublox_msgs/msg/nav_sol.hpp>
#include <ublox_msgs/msg/nav_svinfo.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware.hpp>
#include <ublox_gps/ublox_firmware6.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// U-Blox Firmware Version 6
//
UbloxFirmware6::UbloxFirmware6(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node)
  : UbloxFirmware(updater, gnss, node), frame_id_(frame_id), freq_diag_(freq_diag)
{
  if (getRosBoolean(node_, "publish.nav.posllh")) {
    nav_pos_llh_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavPOSLLH>("navposllh", 1);
  }

  fix_pub_ =
    node_->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

  if (getRosBoolean(node_, "publish.nav.velned")) {
    nav_vel_ned_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavVELNED>("navvelned", 1);
  }

  vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("fix_velocity",
                                                             1);

  if (getRosBoolean(node_, "publish.nav.sol")) {
    nav_sol_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavSOL>("navsol", 1);
  }
  if (getRosBoolean(node_, "publish.nav.svinfo")) {
    nav_svinfo_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavSVINFO>("navinfo", 1);
  }
  if (getRosBoolean(node_, "publish.mon.hw")) {
    mon_hw_pub_ =
      node_->create_publisher<ublox_msgs::msg::MonHW6>("monhw", 1);
  }
}

void UbloxFirmware6::getRosParams() {
  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  if (getRosBoolean(node_, "nmea.set")) {
    if (!getRosUint(node_, "nmea.version", cfg_nmea_.version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.version must be set");
    }
    if (!getRosUint(node_, "nmea.num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
                "true, therefore nmea.num_sv must be set");
    }

    // set flags
    cfg_nmea_.flags = getRosBoolean(node_, "nmea.compat") ? ublox_msgs::msg::CfgNMEA6::FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= getRosBoolean(node_, "nmea.consider") ? ublox_msgs::msg::CfgNMEA6::FLAGS_CONSIDER : 0;

    // set filter
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.pos") ? ublox_msgs::msg::CfgNMEA6::FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.msk_pos") ? ublox_msgs::msg::CfgNMEA6::FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.time") ? ublox_msgs::msg::CfgNMEA6::FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.date") ? ublox_msgs::msg::CfgNMEA6::FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.sbas") ? ublox_msgs::msg::CfgNMEA6::FILTER_SBAS_FILT : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.track") ? ublox_msgs::msg::CfgNMEA6::FILTER_TRACK : 0;
  }
}

bool UbloxFirmware6::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  RCLCPP_WARN(node_->get_logger(), "ublox_version < 7, ignoring GNSS settings");

  if (getRosBoolean(node_, "nmea.set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware6::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Always subscribes to these messages, but may not publish to ROS topic
  // Subscribe to Nav POSLLH
  gps->subscribe<ublox_msgs::msg::NavPOSLLH>(std::bind(
      &UbloxFirmware6::callbackNavPosLlh, this, std::placeholders::_1), 1);
  gps->subscribe<ublox_msgs::msg::NavSOL>(std::bind(
  // Subscribe to Nav SOL
      &UbloxFirmware6::callbackNavSol, this, std::placeholders::_1), 1);
  // Subscribe to Nav VELNED
  gps->subscribe<ublox_msgs::msg::NavVELNED>(std::bind(
      &UbloxFirmware6::callbackNavVelNed, this, std::placeholders::_1), 1);

  // Subscribe to Nav SVINFO
  if (getRosBoolean(node_, "publish.nav.svinfo")) {
    gps->subscribe<ublox_msgs::msg::NavSVINFO>([this](const ublox_msgs::msg::NavSVINFO &m) { nav_svinfo_pub_->publish(m); },
                                          kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean(node_, "publish.mon.hw")) {
    gps->subscribe<ublox_msgs::msg::MonHW6>([this](const ublox_msgs::msg::MonHW6 &m) { mon_hw_pub_->publish(m); },
                                       1);
  }
}

void UbloxFirmware6::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // Set the diagnostic level based on the fix status
  if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_sol_.gps_fix ==
             ublox_msgs::msg::NavSOL::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Time fix only";
  }
  // If fix is not ok (within DOP & Accuracy Masks), raise the diagnostic level
  if (!(last_nav_sol_.flags & ublox_msgs::msg::NavSOL::FLAGS_GPS_FIX_OK)) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message += ", fix not ok";
  }
  // Raise diagnostic level to error if no fix
  if (last_nav_sol_.gps_fix == ublox_msgs::msg::NavSOL::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  }

  // Add last fix position
  stat.add("iTOW [ms]", last_nav_pos_.i_tow);
  stat.add("Latitude [deg]", last_nav_pos_.lat * 1e-7);
  stat.add("Longitude [deg]", last_nav_pos_.lon * 1e-7);
  stat.add("Altitude [m]", last_nav_pos_.height * 1e-3);
  stat.add("Height above MSL [m]", last_nav_pos_.h_msl * 1e-3);
  stat.add("Horizontal Accuracy [m]", last_nav_pos_.h_acc * 1e-3);
  stat.add("Vertical Accuracy [m]", last_nav_pos_.v_acc * 1e-3);
  stat.add("# SVs used", static_cast<int>(last_nav_sol_.num_sv));
}

void UbloxFirmware6::callbackNavPosLlh(const ublox_msgs::msg::NavPOSLLH& m) {
  if (getRosBoolean(node_, "publish.nav.posllh")) {
    nav_pos_llh_pub_->publish(m);
  }

  // Position message
  if (m.i_tow == last_nav_vel_.i_tow) {
    fix_.header.stamp = velocity_.header.stamp; // use last timestamp
  } else {
    fix_.header.stamp = node_->now(); // new timestamp
  }

  fix_.header.frame_id = frame_id_;
  fix_.latitude = m.lat * 1e-7;
  fix_.longitude = m.lon * 1e-7;
  fix_.altitude = m.height * 1e-3;

  if (last_nav_sol_.gps_fix >= ublox_msgs::msg::NavSOL::GPS_2D_FIX) {
    fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  // Convert from mm to m
  const double var_h = pow(m.h_acc / 1000.0, 2);
  const double var_v = pow(m.v_acc / 1000.0, 2);

  fix_.position_covariance[0] = var_h;
  fix_.position_covariance[4] = var_h;
  fix_.position_covariance[8] = var_v;
  fix_.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix_pub_->publish(fix_);
  last_nav_pos_ = m;
  //  update diagnostics
  freq_diag_->diagnostic->tick(fix_.header.stamp);
  updater_->force_update();
}

void UbloxFirmware6::callbackNavVelNed(const ublox_msgs::msg::NavVELNED& m) {
  if (getRosBoolean(node_, "publish.nav.velned")) {
    nav_vel_ned_pub_->publish(m);
  }

  // Example geometry message
  if (m.i_tow == last_nav_pos_.i_tow) {
    velocity_.header.stamp = fix_.header.stamp; // same time as last navposllh
  } else {
    velocity_.header.stamp = node_->now(); // create a new timestamp
  }
  velocity_.header.frame_id = frame_id_;

  //  convert to XYZ linear velocity
  velocity_.twist.twist.linear.x = m.vel_e / 100.0;
  velocity_.twist.twist.linear.y = m.vel_n / 100.0;
  velocity_.twist.twist.linear.z = -m.vel_d / 100.0;

  const double var_speed = pow(m.s_acc / 100.0, 2);

  const int cols = 6;
  velocity_.twist.covariance[cols * 0 + 0] = var_speed;
  velocity_.twist.covariance[cols * 1 + 1] = var_speed;
  velocity_.twist.covariance[cols * 2 + 2] = var_speed;
  velocity_.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  vel_pub_->publish(velocity_);
  last_nav_vel_ = m;
}

void UbloxFirmware6::callbackNavSol(const ublox_msgs::msg::NavSOL& m) {
  if (getRosBoolean(node_, "publish.nav.sol")) {
    nav_sol_pub_->publish(m);
  }
  last_nav_sol_ = m;
}

}  // namespace ublox_node
