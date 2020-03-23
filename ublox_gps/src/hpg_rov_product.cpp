#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_dgnss.hpp>
#include <ublox_msgs/msg/nav_relposned.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/hpg_rov_product.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// U-Blox High Precision GNSS Rover
//
HpgRovProduct::HpgRovProduct(uint16_t nav_rate, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node)
  : dgnss_mode_(ublox_msgs::msg::CfgDGNSS::DGNSS_MODE_RTK_FIXED), nav_rate_(nav_rate), updater_(updater), node_(node)
{
  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_rel_pos_ned_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavRELPOSNED>("navrelposned", 1);
  }
}

void HpgRovProduct::getRosParams() {
  // default to fixed, see CfgDGNSS message for details
  getRosUint(node_, "dgnss_mode", dgnss_mode_,
             ublox_msgs::msg::CfgDGNSS::DGNSS_MODE_RTK_FIXED);
}

bool HpgRovProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  // Configure the DGNSS
  if (!gps->setDgnss(dgnss_mode_)) {
    throw std::runtime_error(std::string("Failed to Configure DGNSS"));
  }
  return true;
}

void HpgRovProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps->subscribe<ublox_msgs::msg::NavRELPOSNED>(std::bind(
     &HpgRovProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);
}

void HpgRovProduct::initializeRosDiagnostics() {
  freq_rtcm_ = std::make_unique<UbloxTopicDiagnostic>(std::string("rxmrtcm"),
                                                      kRtcmFreqMin, kRtcmFreqMax,
                                                      kRtcmFreqTol, kRtcmFreqWindow, updater_);
  updater_->add("Carrier Phase Solution", this,
                &HpgRovProduct::carrierPhaseDiagnostics);
  updater_->force_update();
}

void HpgRovProduct::carrierPhaseDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  uint32_t carr_soln = last_rel_pos_.flags & ublox_msgs::msg::NavRELPOSNED::FLAGS_CARR_SOLN_MASK;
  stat.add("iTow", last_rel_pos_.i_tow);
  if (carr_soln & ublox_msgs::msg::NavRELPOSNED::FLAGS_CARR_SOLN_NONE ||
      !(last_rel_pos_.flags & ublox_msgs::msg::NavRELPOSNED::FLAGS_DIFF_SOLN &&
        last_rel_pos_.flags & ublox_msgs::msg::NavRELPOSNED::FLAGS_REL_POS_VALID)) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.message = "None";
  } else {
    if (carr_soln & ublox_msgs::msg::NavRELPOSNED::FLAGS_CARR_SOLN_FLOAT) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "Float";
    } else if (carr_soln & ublox_msgs::msg::NavRELPOSNED::FLAGS_CARR_SOLN_FIXED) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Fixed";
    }
    stat.add("Ref Station ID", last_rel_pos_.ref_station_id);

    double rel_pos_n = (last_rel_pos_.rel_pos_n
                       + (last_rel_pos_.rel_pos_hpn * 1e-2)) * 1e-2;
    double rel_pos_e = (last_rel_pos_.rel_pos_e
                       + (last_rel_pos_.rel_pos_hpe * 1e-2)) * 1e-2;
    double rel_pos_d = (last_rel_pos_.rel_pos_d
                       + (last_rel_pos_.rel_pos_hpd * 1e-2)) * 1e-2;
    stat.add("Relative Position N [m]", rel_pos_n);
    stat.add("Relative Accuracy N [m]", last_rel_pos_.acc_n * 1e-4);
    stat.add("Relative Position E [m]", rel_pos_e);
    stat.add("Relative Accuracy E [m]", last_rel_pos_.acc_e * 1e-4);
    stat.add("Relative Position D [m]", rel_pos_d);
    stat.add("Relative Accuracy D [m]", last_rel_pos_.acc_d * 1e-4);
  }
}

void HpgRovProduct::callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED &m) {
  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_rel_pos_ned_pub_->publish(m);
  }

  last_rel_pos_ = m;
  updater_->force_update();
}

}  // namespace ublox_node
