#include <memory>
#include <stdexcept>

#include <sensor_msgs/NavSatStatus.h>

#include <ublox_msgs/CfgGNSSBlock.h>
#include <ublox_msgs/CfgGNSS.h>
#include <ublox_msgs/MonHW.h>
#include <ublox_msgs/NavPVT7.h>
#include <ublox_msgs/NavSVINFO.h>

#include <ublox_gps/ublox_firmware7.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// Ublox Firmware Version 7
//
void UbloxFirmware7::getRosParams() {
  //
  // GNSS configuration
  //
  // GNSS enable/disable
  enable_gps_ = getRosBoolean(node_, "gnss/gps");
  enable_glonass_ = getRosBoolean(node_, "gnss/glonass");
  enable_qzss_ = getRosBoolean(node_, "gnss/qzss");

  getRosUint(node_, "gnss/qzss_sig_cfg", qzss_sig_cfg_,
              ublox_msgs::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !gnss_->isSupported("GPS")) {
    ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by this device");
  }
  if (enable_glonass_ && !gnss_->isSupported("GLO")) {
    ROS_WARN("gnss/glonass is true, but GLONASS is not %s",
             "supported by this device");
  }
  if (enable_qzss_ && !gnss_->isSupported("QZSS")) {
    ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (getRosBoolean(node_, "gnss/sbas") && !gnss_->isSupported("SBAS")) {
    ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");
  }

  if (getRosBoolean(node_, "gnss/galileo")) {
    ROS_WARN("ublox_version < 8, ignoring Galileo GNSS Settings");
  }
  if (getRosBoolean(node_, "gnss/beidou")) {
    ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
  }
  if (getRosBoolean(node_, "gnss/imes")) {
    ROS_WARN("ublox_version < 8, ignoring IMES GNSS Settings");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS;

  //
  // NMEA Configuration
  //
  if (getRosBoolean(node_, "nmea/set")) {
    bool compat, consider;

    if (!getRosUint(node_, "nmea/version", cfg_nmea_.nmea_version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/version must be set");
    }
    if (!getRosUint(node_, "nmea/num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                "true, therefore nmea/num_sv must be set");
    }
    if (!getRosUint(node_, "nmea/sv_numbering", cfg_nmea_.sv_numbering)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/sv_numbering must be set");
    }
    if (!node_->getParam("nmea/compat", compat)) {
        throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/compat must be set");
    }
    if (!node_->getParam("nmea/consider", consider)) {
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
          "true, therefore nmea/consider must be set");
    }

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
    // set filter
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/pos") ? cfg_nmea_.FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/msk_pos") ? cfg_nmea_.FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/time") ? cfg_nmea_.FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/date") ? cfg_nmea_.FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/gps_only") ? cfg_nmea_.FILTER_GPS_ONLY : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea/filter/track") ? cfg_nmea_.FILTER_TRACK : 0;
    // set gnssToFilter
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea/gnssToFilter/gps") ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea/gnssToFilter/sbas") ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea/gnssToFilter/qzss") ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea/gnssToFilter/glonass") ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;

    getRosUint(node_, "nmea/main_talker_id", cfg_nmea_.main_talker_id);
    getRosUint(node_, "nmea/gsv_talker_id", cfg_nmea_.gsv_talker_id);
  }
}

bool UbloxFirmware7::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  /** Configure the GNSS **/
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps->poll(cfgGNSSRead)) {
    ROS_DEBUG("Read GNSS config.");
    ROS_DEBUG("Num. tracking channels in hardware: %i", cfgGNSSRead.num_trk_ch_hw);
    ROS_DEBUG("Num. tracking channels to use: %i", cfgGNSSRead.num_trk_ch_use);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.num_config_blocks = 1;  // do services one by one
  cfgGNSSWrite.num_trk_ch_hw = cfgGNSSRead.num_trk_ch_hw;
  cfgGNSSWrite.num_trk_ch_use = cfgGNSSRead.num_trk_ch_use;
  cfgGNSSWrite.msg_ver = 0;

  // configure GLONASS
  if (gnss_->isSupported("GLO")) {
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_GLONASS;
    block.res_trk_ch = block.RES_TRK_CH_GLONASS;
    block.max_trk_ch = block.MAX_TRK_CH_GLONASS;
    block.flags = enable_glonass_ ? block.SIG_CFG_GLONASS_L1OF : 0;
    cfgGNSSWrite.blocks.push_back(block);
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " GLONASS.");
    }
  }

  if (gnss_->isSupported("QZSS")) {
    // configure QZSS
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_QZSS;
    block.res_trk_ch = block.RES_TRK_CH_QZSS;
    block.max_trk_ch = block.MAX_TRK_CH_QZSS;
    block.flags = enable_qzss_ ? qzss_sig_cfg_ : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " QZSS.");
    }
  }

  if (gnss_->isSupported("SBAS")) {
    // configure SBAS
    ublox_msgs::CfgGNSSBlock block;
    block.gnss_id = block.GNSS_ID_SBAS;
    block.res_trk_ch = block.RES_TRK_CH_SBAS;
    block.max_trk_ch = block.MAX_TRK_CH_SBAS;
    block.flags = getRosBoolean(node_, "gnss/sbas") ? block.SIG_CFG_SBAS_L1CA : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               (getRosBoolean(node_, "gnss/sbas") ? "enable" : "disable") +
                               " SBAS.");
    }
  }

  if (getRosBoolean(node_, "nmea/set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware7::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav PVT (always does so since fix information is published
  // from this)
  gps->subscribe<ublox_msgs::NavPVT7>(std::bind(
        &UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1),
        1);

  // Subscribe to Nav SVINFO
  if (getRosBoolean(node_, "publish/nav/svinfo")) {
    gps->subscribe<ublox_msgs::NavSVINFO>([this](const ublox_msgs::NavSVINFO &m) { nav_svinfo_pub_.publish(m); },
                                          kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean(node_, "publish/mon/hw")) {
    gps->subscribe<ublox_msgs::MonHW>([this](const ublox_msgs::MonHW &m) { mon_hw_pub_.publish(m); },
                                      1);
  }
}

}  // namespace ublox_node
