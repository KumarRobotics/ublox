#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_tmode3.hpp>
#include <ublox_msgs/msg/nav_svin.hpp>

#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/rtcm.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// u-blox High Precision GNSS Reference Station
//

HpgRefProduct::HpgRefProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
  : tmode3_(0), lla_flag_(false), fixed_pos_acc_(0.0), svin_reset_(false), sv_in_min_dur_(0), sv_in_acc_lim_(0.0), nav_rate_(nav_rate), meas_rate_(meas_rate), updater_(updater), rtcms_(rtcms), node_(node)
{
  if (getRosBoolean(node_, "publish.nav.svin")) {
    navsvin_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavSVIN>("navsvin", 1);
  }
}

/**
 * @brief Get a int (size 8) vector from the parameter server.
 * @throws std::runtime_error if the parameter is out of bounds.
 * @return true if found, false if not found.
 */
bool getRosInt(rclcpp::Node* node, const std::string& key, std::vector<int8_t> &i) {
  std::vector<int64_t> param;
  if (!node->get_parameter(key, param)) {
    return false;
  }

  // Check the bounds
  int8_t min = std::numeric_limits<int8_t>::lowest();
  int8_t max = std::numeric_limits<int8_t>::max();
  checkRange(param, min, max, key);

  // set the output
  i.insert(i.begin(), param.begin(), param.end());
  return true;
}

void HpgRefProduct::getRosParams() {
  if (getRosBoolean(node_, "config_on_startup")) {
    if (nav_rate_ * meas_rate_ != 1000) {
      RCLCPP_WARN(node_->get_logger(), "For HPG Ref devices, nav_rate should be exactly 1 Hz.");
    }

    if (!getRosUint(node_, "tmode3", tmode3_)) {
      throw std::runtime_error("Invalid settings: TMODE3 must be set");
    }

    if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED) {
      if (!node_->get_parameter("arp.position", arp_position_)) {
        throw std::runtime_error(std::string("Invalid settings: arp.position ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!getRosInt(node_, "arp.position_hp", arp_position_hp_)) {
        throw std::runtime_error(std::string("Invalid settings: arp.position_hp ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!node_->get_parameter("arp.acc", fixed_pos_acc_)) {
        throw std::runtime_error(std::string("Invalid settings: arp.acc ")
                                + "must be set if TMODE3 is fixed");
      }
      if (!node_->get_parameter("arp.lla_flag", lla_flag_)) {
        RCLCPP_WARN(node_->get_logger(), "arp/lla_flag param not set, assuming ARP coordinates are %s",
                "in ECEF");
        lla_flag_ = false;
      }
    } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
      svin_reset_ = getRosBoolean(node_, "sv_in.reset");
      if (!getRosUint(node_, "sv_in.min_dur", sv_in_min_dur_)) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/min_dur ")
                                + "must be set if TMODE3 is survey-in");
      }
      if (!node_->get_parameter("sv_in.acc_lim", sv_in_acc_lim_)) {
        throw std::runtime_error(std::string("Invalid settings: sv_in/acc_lim ")
                                + "must be set if TMODE3 is survey-in");
      }
    } else if (tmode3_ != ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED) {
      throw std::runtime_error(std::string("tmode3 param invalid. See CfgTMODE3")
                              + " flag constants for possible values.");
    }
  }
}

bool HpgRefProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  // Configure TMODE3
  if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if (!gps->disableTmode3()) {
      throw std::runtime_error("Failed to disable TMODE3.");
    }
    mode_ = DISABLED;
  } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED) {
    if (!gps->configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_,
                               fixed_pos_acc_)) {
      throw std::runtime_error("Failed to set TMODE3 to fixed.");
    }
    if (!gps->configRtcm(rtcms_)) {
      throw std::runtime_error("Failed to set RTCM rates");
    }
    mode_ = FIXED;
  } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    if (!svin_reset_) {
      ublox_msgs::msg::NavSVIN nav_svin;
      if (!gps->poll(nav_svin)) {
        throw std::runtime_error(std::string("Failed to poll NavSVIN while") +
                                 " configuring survey-in");
      }
      // Don't reset survey-in if it's already active
      if (nav_svin.active) {
        mode_ = SURVEY_IN;
        return true;
      }
      // Don't reset survey-in if it already has a valid value
      if (nav_svin.valid) {
        setTimeMode(gps);
        return true;
      }
      ublox_msgs::msg::NavPVT nav_pvt;
      if (!gps->poll(nav_pvt, std::vector<uint8_t>(), std::chrono::milliseconds(15000))) {
        throw std::runtime_error(std::string("Failed to poll NavPVT while") +
                                 " configuring survey-in");
      }
      // Don't reset survey in if in time mode with a good fix
      if (nav_pvt.fix_type == ublox_msgs::msg::NavPVT::FIX_TYPE_TIME_ONLY
          && nav_pvt.flags & ublox_msgs::msg::NavPVT::FLAGS_GNSS_FIX_OK) {
        setTimeMode(gps);
        return true;
      }
    }
    // Reset the Survey In
    // For Survey in, meas rate must be at least 1 Hz
    uint16_t meas_rate_temp = meas_rate_ < 1000 ? meas_rate_ : 1000; // [ms]
    // If measurement period isn't a factor of 1000, set to default
    if (1000 % meas_rate_temp != 0) {
      meas_rate_temp = kDefaultMeasPeriod;
    }
    // Set nav rate to 1 Hz during survey in
    if (!gps->configRate(meas_rate_temp, 1000 / meas_rate_temp)) {
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") +
                               "before setting TMODE3 to survey-in.");
    }
    // As recommended in the documentation, first disable, then set to survey in
    if (!gps->disableTmode3()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to disable TMODE3 before setting to survey-in.");
    } else {
      mode_ = DISABLED;
    }
    // Set to Survey in mode
    if (!gps->configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_)) {
      throw std::runtime_error("Failed to set TMODE3 to survey-in.");
    }
    mode_ = SURVEY_IN;
  }
  return true;
}

void HpgRefProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav Survey-In
  // Save off the gps pointer so we can use it in the callback later.
  gps_ = gps;
  gps->subscribe<ublox_msgs::msg::NavSVIN>(std::bind(
      &HpgRefProduct::callbackNavSvIn, this, std::placeholders::_1), 1);
}

void HpgRefProduct::callbackNavSvIn(const ublox_msgs::msg::NavSVIN& m) {
  if (getRosBoolean(node_, "publish.nav.svin")) {
    navsvin_pub_->publish(m);
  }

  last_nav_svin_ = m;

  if (!m.active && m.valid && mode_ == SURVEY_IN) {
    setTimeMode(gps_);
  }

  updater_->force_update();
}

bool HpgRefProduct::setTimeMode(std::shared_ptr<ublox_gps::Gps> gps) {
  RCLCPP_INFO(node_->get_logger(), "Setting mode (internal state) to Time Mode");
  mode_ = TIME;

  // Set the Measurement & nav rate to user config
  // (survey-in sets nav_rate to 1 Hz regardless of user setting)
  if (!gps->configRate(meas_rate_, nav_rate_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set measurement rate to %d ms navigation rate to %d cycles",
                 meas_rate_, nav_rate_);
  }
  // Enable the RTCM out messages
  if (!gps->configRtcm(rtcms_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure RTCM IDs");
    return false;
  }
  return true;
}

void HpgRefProduct::initializeRosDiagnostics() {
  updater_->add("TMODE3", this, &HpgRefProduct::tmode3Diagnostics);
  updater_->force_update();
}

void HpgRefProduct::tmode3Diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (mode_ == INIT) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "Not configured";
  } else if (mode_ == DISABLED){
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Disabled";
  } else if (mode_ == SURVEY_IN) {
    if (!last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.message = "Survey-In inactive and invalid";
    } else if (last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.message = "Survey-In active but invalid";
    } else if (!last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Survey-In complete";
    } else if (last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.message = "Survey-In active and valid";
    }

    stat.add("iTOW [ms]", last_nav_svin_.i_tow);
    stat.add("Duration [s]", last_nav_svin_.dur);
    stat.add("# observations", last_nav_svin_.obs);
    stat.add("Mean X [m]", last_nav_svin_.mean_x * 1e-2);
    stat.add("Mean Y [m]", last_nav_svin_.mean_y * 1e-2);
    stat.add("Mean Z [m]", last_nav_svin_.mean_z * 1e-2);
    stat.add("Mean X HP [m]", last_nav_svin_.mean_xhp * 1e-4);
    stat.add("Mean Y HP [m]", last_nav_svin_.mean_yhp * 1e-4);
    stat.add("Mean Z HP [m]", last_nav_svin_.mean_zhp * 1e-4);
    stat.add("Mean Accuracy [m]", last_nav_svin_.mean_acc * 1e-4);
  } else if (mode_ == FIXED) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Fixed Position";
  } else if (mode_ == TIME) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.message = "Time";
  }
}

}  // namespace ublox_node
