//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_NODE_HPP
#define UBLOX_GPS_NODE_HPP

// STL
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
// ROS messages
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
// Other U-Blox package includes
#include <ublox_msgs/ublox_msgs.hpp>
// Ublox GPS includes
#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/rtcm.hpp>
#include <ublox_gps/utils.hpp>
#include <ublox_gps/raw_data_pa.hpp>
#include <ublox_gps/ublox_firmware.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

// This file also declares UbloxNode which implements ComponentInterface and is
// the main class and ros node. it implements functionality which applies to
// any u-blox device, regardless of the firmware version or product type.
// The class is designed in compositional style; it contains ComponentInterfaces
// which implement features specific to the device based on its firmware version
// and product category. UbloxNode calls the public methods of each component.
//
// This file declares UbloxFirmware is an abstract class which implements
// ComponentInterface and functions generic to all firmware (such as the
// initializing the fix diagnostics). Subclasses of UbloxFirmware for firmware
// versions 6-8 are also declared in this file.
//
// Lastly, this file declares classes for each product category which also
// implement u-blox interface, currently only the class for High Precision
// GNSS devices has been fully implemented and tested.

/**
 * @namespace ublox_node
 * This namespace is for the ROS u-blox node and handles anything regarding
 * ROS parameters, message passing, diagnostics, etc.
 */
namespace ublox_node {

/**
 * @brief This class represents u-blox ROS node for *all* firmware and product
 * versions.
 *
 * @details It loads the user parameters, configures the u-blox
 * device, subscribes to u-blox messages, and configures the device hardware.
 * Functionality specific to a given product or firmware version, etc. should
 * NOT be implemented in this class. Instead, the user should add the
 * functionality to the appropriate implementation of ComponentInterface.
 * If necessary, the user should create a class which implements u-blox
 * interface, then add a pointer to an instance of the class to the
 * components vector.
 * The UbloxNode calls the public methods of ComponentInterface for each
 * element in the components vector.
 */
class UbloxNode final {
 public:
  //! How long to wait during I/O reset [s]
  constexpr static int kResetWait = 10;
  //! how often (in seconds) to call poll messages
  constexpr static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  //! [s] 5Hz diagnostic period
  constexpr static float kDiagnosticPeriod = 0.2;
  //! Tolerance for Fix topic frequency as percentage of target frequency
  double kFixFreqTol = 0.15;
  //! Window [num messages] for Fix Frequency Diagnostic
  double kFixFreqWindow = 10;
  //! Minimum Time Stamp Status for fix frequency diagnostic
  double kTimeStampStatusMin = 0;

  /**
   * @brief Initialize and run the u-blox node.
   */
  UbloxNode();

  /**
   * @brief Get the node parameters from the ROS Parameter Server.
   */
  void getRosParams();

  /**
   * @brief Configure the device based on ROS parameters.
   * @return true if configured successfully
   */
  bool configureUblox();

  /**
   * @brief Subscribe to all requested u-blox messages.
   */
  void subscribe();

  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initializeRosDiagnostics();

  /**
   * @brief Print an INF message to the ROS console.
   */
  void printInf(const ublox_msgs::Inf &m, uint8_t id);

 private:

  /**
   * @brief Initialize the I/O handling.
   */
  void initializeIo();

  /**
   * @brief Initialize the U-Blox node. Configure the U-Blox and subscribe to
   * messages.
   */
  void initialize();

  /**
   * @brief Shutdown the node. Closes the serial port.
   */
  void shutdown();

  /**
   * @brief Send a reset message the u-blox device & re-initialize the I/O.
   * @return true if reset was successful, false otherwise.
   */
  bool resetDevice();

  /**
   * @brief Process the MonVer message and add firmware and product components.
   *
   * @details Determines the protocol version, product type and supported GNSS.
   */
  void processMonVer();

  /**
   * @brief Add the interface for firmware specific configuration, subscribers,
   * & diagnostics. This assumes the protocol_version_ has been set.
   */
  void addFirmwareInterface();

  /**
   * @brief Add the interface which is used for product category
   * configuration, subscribers, & diagnostics.
   * @param the product category, i.e. SPG, HPG, ADR, UDR, TIM, or FTS.
   * @param for HPG/TIM products, this value is either REF or ROV, for other
   * products this string is empty
   */
  void addProductInterface(const std::string & product_category,
                           const std::string & ref_rov = "");

  /**
   * @brief Poll messages from the U-Blox device.
   * @param event a timer indicating how often to poll the messages
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Configure INF messages, call after subscribe.
   */
  void configureInf();

  //! The u-blox node components
  /*!
   * The node will call the functions in these interfaces for each object
   * in the vector.
   */
  std::vector<std::shared_ptr<ComponentInterface> > components_;

  //! Determined From Mon VER
  float protocol_version_ = 0;
  // Variables set from parameter server
  //! Device port
  std::string device_;
  //! dynamic model type
  std::string dynamic_model_;
  //! Fix mode type
  std::string fix_mode_;
  //! Set from dynamic model string
  uint8_t dmodel_;
  //! Set from fix mode string
  uint8_t fmode_;
  //! UART1 baudrate
  uint32_t baudrate_;
  //! UART in protocol (see CfgPRT message for constants)
  uint16_t uart_in_;
  //! UART out protocol (see CfgPRT message for constants)
  uint16_t uart_out_;
  //! USB TX Ready Pin configuration (see CfgPRT message for constants)
  uint16_t usb_tx_;
  //! Whether to configure the USB port
  /*! Set to true if usb_in & usb_out parameters are set */
  bool set_usb_;
  //! USB in protocol (see CfgPRT message for constants)
  uint16_t usb_in_;
  //! USB out protocol (see CfgPRT message for constants)
  uint16_t usb_out_ ;
  //! The measurement rate in Hz
  double rate_;
  //! User-defined Datum
  ublox_msgs::CfgDAT cfg_dat_;
  //! SBAS Usage parameter (see CfgSBAS message)
  uint8_t sbas_usage_;
  //! Max SBAS parameter (see CfgSBAS message)
  uint8_t max_sbas_;
  //! Dead reckoning limit parameter
  uint8_t dr_limit_;
  //! Parameters to load from non-volatile memory during configuration
  ublox_msgs::CfgCFG load_;
  //! Parameters to save to non-volatile memory after configuration
  ublox_msgs::CfgCFG save_;
  //! rate for TIM-TM2
  uint8_t tim_rate_;

  //! raw data stream logging
  RawDataStreamPa rawDataStreamPa_;

  ros::Publisher nav_status_pub_;
  ros::Publisher nav_posecef_pub_;
  ros::Publisher nav_clock_pub_;
  ros::Publisher aid_alm_pub_;
  ros::Publisher aid_eph_pub_;
  ros::Publisher aid_hui_pub_;

  //! Navigation rate in measurement cycles, see CfgRate.msg
  uint16_t nav_rate_;

  //! The measurement [ms], see CfgRate.msg
  uint16_t meas_rate_;

  //! The ROS frame ID of this device
  std::string frame_id_;

  //! ROS diagnostic updater
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  //! fix frequency diagnostic updater
  std::shared_ptr<FixDiagnostic> freq_diag_;

  std::vector<ublox_gps::Rtcm> rtcms_;

  //! Which GNSS are supported by the device
  std::shared_ptr<Gnss> gnss_;

  //! Handles communication with the U-Blox Device
  std::shared_ptr<ublox_gps::Gps> gps_;

  //! Node Handle for GPS node
  std::shared_ptr<ros::NodeHandle> nh_;

};

/**
 * @brief Implements functions for firmware version 7.
 */
class UbloxFirmware7 final : public UbloxFirmware7Plus<ublox_msgs::NavPVT7> {
 public:
  explicit UbloxFirmware7(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node)
    : UbloxFirmware7Plus<ublox_msgs::NavPVT7>(frame_id, updater, freq_diag, gnss, node) {
    nav_svinfo_pub_ = node->advertise<ublox_msgs::NavSVINFO>("navsvinfo", 1);
    mon_hw_pub_ = node->advertise<ublox_msgs::MonHW>("monhw", 1);
  }

  /**
   * @brief Get the parameters specific to firmware version 7.
   *
   * @details Get the GNSS and NMEA settings.
   */
  void getRosParams() override;

  /**
   * @brief Configure GNSS individually. Only configures GLONASS.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Subscribe to messages which are not generic to all firmware.
   *
   * @details Subscribe to NavPVT7 messages, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  //! Used to configure NMEA (if set_nmea_)
  /*!
   * Filled from ROS parameters
   */
  ublox_msgs::CfgNMEA7 cfg_nmea_;

  ros::Publisher nav_svinfo_pub_;
  ros::Publisher mon_hw_pub_;
};

/**
 *  @brief Implements functions for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus<ublox_msgs::NavPVT> {
 public:
  explicit UbloxFirmware8(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node)
    : UbloxFirmware7Plus<ublox_msgs::NavPVT>(frame_id, updater, freq_diag, gnss, node) {
    nav_sat_pub_ = node->advertise<ublox_msgs::NavSAT>("navstate", 1);
    mon_hw_pub_ = node->advertise<ublox_msgs::MonHW>("monhw", 1);
    rxm_rtcm_pub_ = node->advertise<ublox_msgs::RxmRTCM>("rxmrtcm", 1);
  }

  /**
   * @brief Get the ROS parameters specific to firmware version 8.
   *
   * @details Get the GNSS, NMEA, and UPD settings.
   */
  void getRosParams() override;

  /**
   * @brief Configure settings specific to firmware 8 based on ROS parameters.
   *
   * @details Configure GNSS, if it is different from current settings.
   * Configure the NMEA if desired by the user. It also may clear the
   * flash memory based on the ROS parameters.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Subscribe to u-blox messages which are not generic to all firmware
   * versions.
   *
   * @details Subscribe to NavPVT, NavSAT, MonHW, and RxmRTCM messages based
   * on user settings.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  // Set from ROS parameters
  //! Whether or not to enable the Galileo GNSS
  bool enable_galileo_;
  //! Whether or not to enable the BeiDuo GNSS
  bool enable_beidou_;
  //! Whether or not to enable the IMES GNSS
  bool enable_imes_;
  //! Desired NMEA configuration.
  ublox_msgs::CfgNMEA cfg_nmea_;
  //! Whether to clear the flash memory during configuration
  bool clear_bbr_;
  bool save_on_shutdown_;

  ros::Publisher nav_sat_pub_;
  ros::Publisher mon_hw_pub_;
  ros::Publisher rxm_rtcm_pub_;
};

/**
 *  @brief Implements functions for firmware version 9.
 *  For now it simply re-uses the firmware version 8 class
 *  but allows for future expansion of functionality
 */
class UbloxFirmware9 final : public UbloxFirmware8 {
public:
  explicit UbloxFirmware9(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node);
};

/**
 * @brief Implements functions for High Precision GNSS Reference station
 * devices.
 */
class HpgRefProduct: public virtual ComponentInterface {
 public:
  //! Default measurement period for HPG devices
  constexpr static uint16_t kDefaultMeasPeriod = 250;

  explicit HpgRefProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, ros::NodeHandle* node);

  /**
   * @brief Get the ROS parameters specific to the Reference Station
   * configuration.
   *
   * @details Get the TMODE3 settings, the parameters it gets depends on the
   * tmode3 parameter. For example, it will get survey-in parameters if the
   * tmode3 parameter is set to survey in or it will get the fixed parameters if
   * it is set to fixed.
   */
  void getRosParams() override;

  /**
   * @brief Configure the u-blox Reference Station settings.
   *
   * @details Configure the TMODE3 settings and sets the internal state based
   * on the TMODE3 status. If the TMODE3 is set to fixed, it will configure
   * the RTCM messages.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Add diagnostic updaters for the TMODE3 status.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to u-blox Reference Station messages.
   *
   * @details Subscribe to NavSVIN messages based on user parameters.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Update the last received NavSVIN message and call diagnostic updater
   *
   * @details When the survey in finishes, it changes the measurement &
   * navigation rate to the user configured values and enables the user
   * configured RTCM messages. Publish received Nav SVIN messages if enabled.
   * @param m the message to process
   */
  void callbackNavSvIn(const ublox_msgs::NavSVIN& m);

 protected:
  /**
   * @brief Update the TMODE3 diagnostics.
   *
   * @details Updates the status of the survey-in if in  survey-in mode or the
   * RTCM messages if in time mode.
   */
  void tmode3Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Set the device mode to time mode (internal state variable).
   *
   * @details Configure the RTCM messages and measurement and navigation rate.
   */
  bool setTimeMode(std::shared_ptr<ublox_gps::Gps> gps);

  //! The last received Nav SVIN message
  ublox_msgs::NavSVIN last_nav_svin_;

  //! TMODE3 to set, such as disabled, survey-in, fixed
  uint8_t tmode3_;

  // TMODE3 = Fixed mode settings
  //! True if coordinates are in LLA, false if ECEF
  /*! Used only for fixed mode */
  bool lla_flag_;
  //! Antenna Reference Point Position [m] or [deg]
  /*! Used only for fixed mode */
  std::vector<float> arp_position_;
  //! Antenna Reference Point Position High Precision [0.1 mm] or [deg * 1e-9]
  /*! Used only for fixed mode */
  std::vector<int8_t> arp_position_hp_;
  //! Fixed Position Accuracy [m]
  /*! Used only for fixed mode */
  float fixed_pos_acc_;

  // Settings for TMODE3 = Survey-in
  //! Whether to always reset the survey-in during configuration.
  /*!
   * If false, it only resets survey-in if there's no fix and TMODE3 is
   * disabled before configuration.
   * This variable is used only if TMODE3 is set to survey-in.
   */
  bool svin_reset_;
  //! Measurement period used during Survey-In [s]
  /*! This variable is used only if TMODE3 is set to survey-in. */
  uint32_t sv_in_min_dur_;
  //! Survey in accuracy limit [m]
  /*! This variable is used only if TMODE3 is set to survey-in. */
  float sv_in_acc_lim_;

  //! Status of device time mode
  enum {
    INIT, //!< Initialization mode (before configuration)
    FIXED, //!< Fixed mode (should switch to time mode almost immediately)
    DISABLED, //!< Time mode disabled
    SURVEY_IN, //!< Survey-In mode
    TIME //!< Time mode, after survey-in or after configuring fixed mode
  } mode_;

  ros::Publisher navsvin_pub_;

  uint16_t nav_rate_;
  uint16_t meas_rate_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  std::vector<ublox_gps::Rtcm> rtcms_;
  std::shared_ptr<ublox_gps::Gps> gps_;
  ros::NodeHandle* node_;
};

class HpPosRecProduct final : public virtual HpgRefProduct {
 public:
  explicit HpPosRecProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, ros::NodeHandle* node);

  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:

  /**
   * @brief Set the last received message and call rover diagnostic updater
   *
   * @details Publish received NavRELPOSNED messages if enabled
   */
  void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m);

  sensor_msgs::Imu imu_;

  //! Last relative position (used for diagnostic updater)
  ublox_msgs::NavRELPOSNED9 last_rel_pos_;

  ros::Publisher nav_relposned_pub_;
  ros::Publisher imu_pub_;

  std::string frame_id_;
};

}

#endif  // UBLOX_GPS_NODE_HPP
