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
#include <ublox_msgs/msg/cfg_rate.hpp>
#include <ublox_msgs/msg/nav_svin.hpp>

#include <ublox_gps/hpg_ref_product_firmware9.hpp>
#include <ublox_gps/rtcm.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// u-blox High Precision GNSS Reference Station
//

HpgRefProductFirmware9::HpgRefProductFirmware9(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
: HpgRefProduct(nav_rate, meas_rate, updater, rtcms, node)
{
  if (getRosBoolean(node_, "publish.nav.svin")) {
    navsvin_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavSVIN>("navsvin", 1);
  }
}

bool HpgRefProductFirmware9::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  gps_ = gps;

  // Configure TMODE3
  if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if (!configTMODE3(ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED)) {
      throw std::runtime_error("Failed to disable TMODE.");
    }
    mode_ = DISABLED;
  } else if (tmode3_ == ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED) {
    if (!configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_,
                               fixed_pos_acc_)) {
      throw std::runtime_error("Failed to set TMODE to fixed.");
    }
    if (!configRTCM()) {
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
    if (!configNavSolutionRate(meas_rate_temp, 1000 / meas_rate_temp)) {
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") +
                               "before setting TMODE to survey-in.");
    }
    // As recommended in the documentation, first disable, then set to survey in
    if (!configTMODE3(ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to disable TMODE before setting to survey-in.");
    } else {
      mode_ = DISABLED;
    }
    // Set to Survey in mode
    if (!configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_)) {
      throw std::runtime_error("Failed to set TMODE to survey-in.");
    }
    mode_ = SURVEY_IN;
  }
  return true;
}

bool HpgRefProductFirmware9::configTmode3Fixed(bool lla_flag,
                            std::vector<double> arp_position,
                            std::vector<int8_t> arp_position_hp,
                            float fixed_pos_acc) {
  if (arp_position.size() != 3 || arp_position_hp.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(), "Configuring TMODE to Fixed: size of position %s",
                 "& arp_position_hp args must be 3");
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Configuring TMODE to Fixed");

  ublox_msgs::msg::CfgVALSET cfgTMODE;
  cfgTMODE.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;
  cfgTMODE.cfgdata.resize(9);

  // Mode
  cfgTMODE.cfgdata[0].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_MODE;
  cfgTMODE.cfgdata[0].data.resize(1);
  cfgTMODE.cfgdata[0].data[0] = ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED;

  // Position Type
  cfgTMODE.cfgdata[1].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_POS_TYPE;
  cfgTMODE.cfgdata[1].data.resize(1);
  cfgTMODE.cfgdata[1].data[0] = lla_flag;

  // Fixed Position Accuracy
  cfgTMODE.cfgdata[2].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_FIXED_POS_ACC;
  cfgTMODE.cfgdata[2].data.resize(4);
  uint32_t fixed_pos_acc_01mm = round(fixed_pos_acc * 1e4);
  std::memcpy(&(cfgTMODE.cfgdata[2].data[0]), &(fixed_pos_acc_01mm), sizeof(uint32_t));

  // Set position
  if (lla_flag) {
    // Lat/Lon (Coarse values are converted from [deg] to [deg * 1e-7])
    cfgTMODE.cfgdata[3].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_LAT;
    cfgTMODE.cfgdata[3].data.resize(4);
    int32_t lat_1e7deg = round(arp_position[0] * 1e7);
    std::memcpy(&(cfgTMODE.cfgdata[3].data[0]), &(lat_1e7deg), sizeof(int32_t));

    cfgTMODE.cfgdata[4].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_LAT_HP;
    cfgTMODE.cfgdata[4].data.resize(1);
    cfgTMODE.cfgdata[4].data[0] = arp_position_hp[0];

    cfgTMODE.cfgdata[5].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_LON;
    cfgTMODE.cfgdata[5].data.resize(4);
    int32_t lon_1e7deg = round(arp_position[1] * 1e7);
    std::memcpy(&(cfgTMODE.cfgdata[5].data[0]), &(lon_1e7deg), sizeof(int32_t));

    cfgTMODE.cfgdata[6].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_LON_HP;
    cfgTMODE.cfgdata[6].data.resize(1);
    cfgTMODE.cfgdata[6].data[0] = arp_position_hp[1];

    cfgTMODE.cfgdata[7].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_HEIGHT;
    cfgTMODE.cfgdata[7].data.resize(4);
    int32_t height_1e7deg = round(arp_position[2] * 1e7);
    std::memcpy(&(cfgTMODE.cfgdata[7].data[0]), &(height_1e7deg), sizeof(int32_t));

    cfgTMODE.cfgdata[8].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_HEIGHT_HP;
    cfgTMODE.cfgdata[8].data.resize(1);
    cfgTMODE.cfgdata[8].data[0] = arp_position_hp[2];

  } else {
    // ECEF (Coarse values are converted from m to cm)
    cfgTMODE.cfgdata[3].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_X;
    cfgTMODE.cfgdata[3].data.resize(4);
    int32_t ecef_x_1e2deg = round(arp_position[0] * 1e2);
    std::memcpy(&(cfgTMODE.cfgdata[3].data[0]), &(ecef_x_1e2deg), sizeof(int32_t));

    cfgTMODE.cfgdata[4].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_X_HP;
    cfgTMODE.cfgdata[4].data.resize(1);
    cfgTMODE.cfgdata[4].data[0] = arp_position_hp[0];

    cfgTMODE.cfgdata[5].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_Y;
    cfgTMODE.cfgdata[5].data.resize(4);
    int32_t ecef_y_1e2deg = round(arp_position[1] * 1e2);
    std::memcpy(&(cfgTMODE.cfgdata[5].data[0]), &(ecef_y_1e2deg), sizeof(int32_t));

    cfgTMODE.cfgdata[6].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_Y_HP;
    cfgTMODE.cfgdata[6].data.resize(1);
    cfgTMODE.cfgdata[6].data[0] = arp_position_hp[1];

    cfgTMODE.cfgdata[7].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_Z;
    cfgTMODE.cfgdata[7].data.resize(4);
    int32_t ecef_z_1e2deg = round(arp_position[2] * 1e2);
    std::memcpy(&(cfgTMODE.cfgdata[7].data[0]), &(ecef_z_1e2deg), sizeof(int32_t));

    cfgTMODE.cfgdata[8].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_ECEF_Z_HP;
    cfgTMODE.cfgdata[8].data.resize(1);
    cfgTMODE.cfgdata[8].data[0] = arp_position_hp[2];
  }

  return gps_->configure(cfgTMODE);
}

bool HpgRefProductFirmware9::setTimeMode(std::shared_ptr<ublox_gps::Gps> gps) {
  (void)gps;
  RCLCPP_INFO(node_->get_logger(), "Setting mode (internal state) to Time Mode");
  mode_ = TIME;

  // Set the Measurement & nav rate to user config
  // (survey-in sets nav_rate to 1 Hz regardless of user setting)
  if (!configNavSolutionRate(meas_rate_, nav_rate_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set measurement rate to %d ms navigation rate to %d cycles",
                 meas_rate_, nav_rate_);
  }
  // Enable the RTCM out messages
  if (!configRTCM()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure RTCM IDs");
    return false;
  }
  return true;
}

bool HpgRefProductFirmware9::configNavSolutionRate(uint16_t meas_rate, uint16_t nav_rate){
  RCLCPP_DEBUG(node_->get_logger(), "Configuring measurement rate to %u ms and nav rate to %u cycles",
              meas_rate, nav_rate);

  ublox_msgs::msg::CfgVALSET cfgRate;
  cfgRate.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;
  cfgRate.cfgdata.resize(3);

  // Config GNSS Measurement Rate
  cfgRate.cfgdata[0].key = ublox_msgs::msg::CfgVALSETCfgdata::RATE_MEAS;
  cfgRate.cfgdata[0].data.resize(2);
  std::memcpy(&(cfgRate.cfgdata[0].data[0]), &meas_rate, sizeof(meas_rate));

  // Config Navigation Solution Rate
  cfgRate.cfgdata[1].key = ublox_msgs::msg::CfgVALSETCfgdata::RATE_NAV;
  cfgRate.cfgdata[1].data.resize(2);
  std::memcpy(&(cfgRate.cfgdata[1].data[0]), &nav_rate, sizeof(nav_rate));

  // Config Time Reference
  cfgRate.cfgdata[2].key = ublox_msgs::msg::CfgVALSETCfgdata::RATE_TIMEREF;
  cfgRate.cfgdata[2].data.resize(1);
  cfgRate.cfgdata[2].data[0] = ublox_msgs::msg::CfgRATE::TIME_REF_GPS;

  return gps_->configure(cfgRate, false);
}

bool HpgRefProductFirmware9::configRTCM(){
  RCLCPP_DEBUG(node_->get_logger(), "Configuring RTCM output messages");

  if(rtcms_.empty()) return false;

  ublox_msgs::msg::CfgVALSET cfgRTCM;
  cfgRTCM.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;
  cfgRTCM.cfgdata.resize(rtcms_.size());

  for(size_t i=0; i<rtcms_.size(); ++i){
    cfgRTCM.cfgdata[i].key = rtcmID2Key(rtcms_[i].id);
    cfgRTCM.cfgdata[i].data.resize(1);
    cfgRTCM.cfgdata[i].data[0] = rtcms_[i].rate;
  }
  return gps_->configure(cfgRTCM, false);
}

bool HpgRefProductFirmware9::configTMODE3(uint8_t mode){
  RCLCPP_DEBUG(node_->get_logger(), "Configuring TMODE Mode");
  if(mode >2){
    RCLCPP_ERROR(node_->get_logger(), "Requested TMODE is invalid: %u", mode);
    return false;
  }
  
  ublox_msgs::msg::CfgVALSET cfgTMODE3;
  cfgTMODE3.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;
  cfgTMODE3.cfgdata.resize(1);

  cfgTMODE3.cfgdata[0].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_MODE;
  cfgTMODE3.cfgdata[0].data.resize(1);
  cfgTMODE3.cfgdata[0].data[0] = mode;

  return gps_->configure(cfgTMODE3);
}

bool HpgRefProductFirmware9::configTmode3SurveyIn(uint32_t sv_in_min_dur_, uint32_t sv_in_acc_lim_){
  RCLCPP_DEBUG(node_->get_logger(), "Configuring Survey-In Mode");
  ublox_msgs::msg::CfgVALSET cfgSurveyIn;
  cfgSurveyIn.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;
  cfgSurveyIn.cfgdata.resize(3);

  // Set TMODE
  cfgSurveyIn.cfgdata[0].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_MODE;
  cfgSurveyIn.cfgdata[0].data.resize(1);
  cfgSurveyIn.cfgdata[0].data[0] = ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN;

  // Set Survey-in Minimum Duration
  cfgSurveyIn.cfgdata[1].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_SVIN_MIN_DUR;
  cfgSurveyIn.cfgdata[1].data.resize(4);
  std::memcpy(&(cfgSurveyIn.cfgdata[1].data[0]), &sv_in_min_dur_, 4);

  // Set Survey-in Accuracy Limit
  cfgSurveyIn.cfgdata[2].key = ublox_msgs::msg::CfgVALSETCfgdata::TMODE_SVIN_ACC_LIM;
  cfgSurveyIn.cfgdata[2].data.resize(4);
  uint32_t accuracy_tenth_of_mm = sv_in_acc_lim_ * 1e4;
  std::memcpy(&(cfgSurveyIn.cfgdata[2].data[0]), &(accuracy_tenth_of_mm), 4);

  return gps_->configure(cfgSurveyIn);
}

uint32_t HpgRefProductFirmware9::rtcmID2Key(uint8_t msgID){
  using key = ublox_msgs::msg::CfgVALSETCfgdata;
  switch(msgID){
    case 5:
      return key::MSGOUT_RTCM_3X_TYPE1005_UART1;
    case 74:
      return key::MSGOUT_RTCM_3X_TYPE1074_UART1;
    case 77:
      return key::MSGOUT_RTCM_3X_TYPE1077_UART1;
    case 84:
      return key::MSGOUT_RTCM_3X_TYPE1084_UART1;
    case 87:
      return key::MSGOUT_RTCM_3X_TYPE1087_UART1;
    case 94:
      return key::MSGOUT_RTCM_3X_TYPE1094_UART1;
    case 97:
      return key::MSGOUT_RTCM_3X_TYPE1097_UART1;
    case 124:
      return key::MSGOUT_RTCM_3X_TYPE1124_UART1;
    case 127:
      return key::MSGOUT_RTCM_3X_TYPE1127_UART1;
    case 230:
      return key::MSGOUT_RTCM_3X_TYPE1230_UART1;
    case 254:
      return key::MSGOUT_RTCM_3X_TYPE4072_0_UART1;
    case 253:
      return key::MSGOUT_RTCM_3X_TYPE4072_1_UART1;
    default:
      return false;
  }
}

}  // namespace ublox_node
