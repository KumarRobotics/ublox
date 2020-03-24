#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/cfg_gnss_block.hpp>
#include <ublox_msgs/msg/cfg_gnss.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_pvt7.hpp>
#include <ublox_msgs/msg/nav_svinfo.hpp>

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
  enable_gps_ = getRosBoolean(node_, "gnss.gps");
  enable_glonass_ = getRosBoolean(node_, "gnss.glonass");
  enable_qzss_ = getRosBoolean(node_, "gnss.qzss");

  getRosUint(node_, "gnss.qzss_sig_cfg", qzss_sig_cfg_,
              ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !gnss_->isSupported("GPS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss/gps is true, but GPS GNSS is not supported by this device");
  }
  if (enable_glonass_ && !gnss_->isSupported("GLO")) {
    RCLCPP_WARN(node_->get_logger(), "gnss/glonass is true, but GLONASS is not %s",
             "supported by this device");
  }
  if (enable_qzss_ && !gnss_->isSupported("QZSS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss/qzss is true, but QZSS is not supported by this device");
  }
  if (getRosBoolean(node_, "gnss.sbas") && !gnss_->isSupported("SBAS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss/sbas is true, but SBAS is not supported by this device");
  }

  if (getRosBoolean(node_, "gnss.galileo")) {
    RCLCPP_WARN(node_->get_logger(), "ublox_version < 8, ignoring Galileo GNSS Settings");
  }
  if (getRosBoolean(node_, "gnss.beidou")) {
    RCLCPP_WARN(node_->get_logger(), "ublox_version < 8, ignoring BeiDou Settings");
  }
  if (getRosBoolean(node_, "gnss.imes")) {
    RCLCPP_WARN(node_->get_logger(), "ublox_version < 8, ignoring IMES GNSS Settings");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

  //
  // NMEA Configuration
  //
  if (getRosBoolean(node_, "nmea.set")) {
    bool compat, consider;

    if (!getRosUint(node_, "nmea.version", cfg_nmea_.nmea_version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.version must be set");
    }
    if (!getRosUint(node_, "nmea.num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
                "true, therefore nmea.num_sv must be set");
    }
    if (!getRosUint(node_, "nmea.sv_numbering", cfg_nmea_.sv_numbering)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.sv_numbering must be set");
    }
    if (!node_->get_parameter("nmea.compat", compat)) {
        throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.compat must be set");
    }
    if (!node_->get_parameter("nmea.consider", consider)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.consider must be set");
    }

    // set flags
    cfg_nmea_.flags = compat ? ublox_msgs::msg::CfgNMEA7::FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? ublox_msgs::msg::CfgNMEA7::FLAGS_CONSIDER : 0;
    // set filter
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.pos") ? ublox_msgs::msg::CfgNMEA7::FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.msk_pos") ? ublox_msgs::msg::CfgNMEA7::FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.time") ? ublox_msgs::msg::CfgNMEA7::FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.date") ? ublox_msgs::msg::CfgNMEA7::FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.gps_only") ? ublox_msgs::msg::CfgNMEA7::FILTER_GPS_ONLY : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.track") ? ublox_msgs::msg::CfgNMEA7::FILTER_TRACK : 0;
    // set gnssToFilter
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.gps") ? ublox_msgs::msg::CfgNMEA7::GNSS_TO_FILTER_GPS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.sbas") ? ublox_msgs::msg::CfgNMEA7::GNSS_TO_FILTER_SBAS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.qzss") ? ublox_msgs::msg::CfgNMEA7::GNSS_TO_FILTER_QZSS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.glonass") ? ublox_msgs::msg::CfgNMEA7::GNSS_TO_FILTER_GLONASS : 0;

    getRosUint(node_, "nmea.main_talker_id", cfg_nmea_.main_talker_id);
    getRosUint(node_, "nmea.gsv_talker_id", cfg_nmea_.gsv_talker_id);
  }
}

bool UbloxFirmware7::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  /** Configure the GNSS **/
  ublox_msgs::msg::CfgGNSS cfgGNSSRead;
  if (gps->poll(cfgGNSSRead)) {
    RCLCPP_DEBUG(node_->get_logger(), "Read GNSS config.");
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels in hardware: %i", cfgGNSSRead.num_trk_ch_hw);
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels to use: %i", cfgGNSSRead.num_trk_ch_use);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::msg::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.num_config_blocks = 1;  // do services one by one
  cfgGNSSWrite.num_trk_ch_hw = cfgGNSSRead.num_trk_ch_hw;
  cfgGNSSWrite.num_trk_ch_use = cfgGNSSRead.num_trk_ch_use;
  cfgGNSSWrite.msg_ver = 0;

  // configure GLONASS
  if (gnss_->isSupported("GLO")) {
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_GLONASS;
    block.res_trk_ch = ublox_msgs::msg::CfgGNSSBlock::RES_TRK_CH_GLONASS;
    block.max_trk_ch = ublox_msgs::msg::CfgGNSSBlock::MAX_TRK_CH_GLONASS;
    block.flags = enable_glonass_ ? ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_GLONASS_L1OF : 0;
    cfgGNSSWrite.blocks.push_back(block);
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " GLONASS.");
    }
  }

  if (gnss_->isSupported("QZSS")) {
    // configure QZSS
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_QZSS;
    block.res_trk_ch = ublox_msgs::msg::CfgGNSSBlock::RES_TRK_CH_QZSS;
    block.max_trk_ch = ublox_msgs::msg::CfgGNSSBlock::MAX_TRK_CH_QZSS;
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
    ublox_msgs::msg::CfgGNSSBlock block;
    block.gnss_id = ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_SBAS;
    block.res_trk_ch = ublox_msgs::msg::CfgGNSSBlock::RES_TRK_CH_SBAS;
    block.max_trk_ch = ublox_msgs::msg::CfgGNSSBlock::MAX_TRK_CH_SBAS;
    block.flags = getRosBoolean(node_, "gnss.sbas") ? ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_SBAS_L1CA : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps->configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               (getRosBoolean(node_, "gnss.sbas") ? "enable" : "disable") +
                               " SBAS.");
    }
  }

  if (getRosBoolean(node_, "nmea.set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware7::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav PVT (always does so since fix information is published
  // from this)
  gps->subscribe<ublox_msgs::msg::NavPVT7>(std::bind(
        &UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1),
        1);

  // Subscribe to Nav SVINFO
  if (getRosBoolean(node_, "publish.nav.svinfo")) {
    gps->subscribe<ublox_msgs::msg::NavSVINFO>([this](const ublox_msgs::msg::NavSVINFO &m) { nav_svinfo_pub_->publish(m); },
                                          kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean(node_, "publish.mon.hw")) {
    gps->subscribe<ublox_msgs::msg::MonHW>([this](const ublox_msgs::msg::MonHW &m) { mon_hw_pub_->publish(m); },
                                      1);
  }
}

}  // namespace ublox_node
