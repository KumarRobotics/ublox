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
#include <memory>
#include <string>
#include <vector>
// ROS includes
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
// U-Blox msgs nicludes
#include <ublox_msgs/msg/cfg_cfg.hpp>
#include <ublox_msgs/msg/cfg_dat.hpp>
#include <ublox_msgs/msg/inf.h>
// Ublox GPS includes
#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/rtcm.hpp>
#include <ublox_gps/raw_data_pa.hpp>

// This file also declares UbloxNode which is the main class and ros node. It
// implements functionality which applies to any u-blox device, regardless of
// the firmware version or product type.  The class is designed in compositional
// style; it contains ComponentInterfaces which implement features specific to
// the device based on its firmware version and product category. UbloxNode
// calls the public methods of each component.

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
class UbloxNode final : public rclcpp::Node {
 public:
  //! How long to wait during I/O reset [s]
  constexpr static int kResetWait = 10;
  //! How often (in seconds) to send keep-alive message
  constexpr static double kKeepAlivePeriod = 10.0;
  //! How often (in seconds) to call poll messages
  constexpr static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  //! [s] 5Hz diagnostic period
  const float kDiagnosticPeriod = 0.2;
  //! Tolerance for Fix topic frequency as percentage of target frequency
  const double kFixFreqTol = 0.15;
  //! Window [num messages] for Fix Frequency Diagnostic
  const double kFixFreqWindow = 10;
  //! Minimum Time Stamp Status for fix frequency diagnostic
  const double kTimeStampStatusMin = 0;

  /**
   * @brief Initialize and run the u-blox node.
   */
  explicit UbloxNode(const rclcpp::NodeOptions & options);

  ~UbloxNode() override;

  UbloxNode(UbloxNode &&c) = delete;
  UbloxNode &operator=(UbloxNode &&c) = delete;
  UbloxNode(const UbloxNode &c) = delete;
  UbloxNode &operator=(const UbloxNode &c) = delete;

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
  void printInf(const ublox_msgs::msg::Inf &m, uint8_t id);

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
   * @brief Poll version message from the U-Blox device to keep socket active.
   */
  void keepAlive();

  /**
   * @brief Poll messages from the U-Blox device.
   */
  void pollMessages();

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
  float protocol_version_ = 0.0;
  // Variables set from parameter server
  //! Device port
  std::string device_;
  //! dynamic model type
  std::string dynamic_model_;
  //! Fix mode type
  std::string fix_mode_;
  //! Set from dynamic model string
  uint8_t dmodel_{0};
  //! Set from fix mode string
  uint8_t fmode_{0};
  //! UART1 baudrate
  uint32_t baudrate_{0};
  //! UART in protocol (see CfgPRT message for constants)
  uint16_t uart_in_{0};
  //! UART out protocol (see CfgPRT message for constants)
  uint16_t uart_out_{0};
  //! USB TX Ready Pin configuration (see CfgPRT message for constants)
  uint16_t usb_tx_{0};
  //! Whether to configure the USB port
  /*! Set to true if usb_in & usb_out parameters are set */
  bool set_usb_{false};
  //! USB in protocol (see CfgPRT message for constants)
  uint16_t usb_in_{0};
  //! USB out protocol (see CfgPRT message for constants)
  uint16_t usb_out_{0};
  //! The measurement rate in Hz
  double rate_{0.0};
  //! User-defined Datum
  ublox_msgs::msg::CfgDAT cfg_dat_;
  //! SBAS Usage parameter (see CfgSBAS message)
  uint8_t sbas_usage_{0};
  //! Max SBAS parameter (see CfgSBAS message)
  uint8_t max_sbas_{0};
  //! Dead reckoning limit parameter
  uint8_t dr_limit_{0};
  //! Parameters to load from non-volatile memory during configuration
  ublox_msgs::msg::CfgCFG load_;
  //! Parameters to save to non-volatile memory after configuration
  ublox_msgs::msg::CfgCFG save_;
  //! rate for TIM-TM2
  uint8_t tim_rate_{0};

  //! raw data stream logging
  std::shared_ptr<RawDataStreamPa> raw_data_stream_pa_;

  rclcpp::Publisher<ublox_msgs::msg::NavSTATUS>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavPOSECEF>::SharedPtr nav_posecef_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavCLOCK>::SharedPtr nav_clock_pub_;
  rclcpp::Publisher<ublox_msgs::msg::AidALM>::SharedPtr aid_alm_pub_;
  rclcpp::Publisher<ublox_msgs::msg::AidEPH>::SharedPtr aid_eph_pub_;
  rclcpp::Publisher<ublox_msgs::msg::AidHUI>::SharedPtr aid_hui_pub_;

  //! Navigation rate in measurement cycles, see CfgRate.msg
  uint16_t nav_rate_{0};

  //! The measurement [ms], see CfgRate.msg
  uint16_t meas_rate_{0};

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

  rclcpp::TimerBase::SharedPtr keep_alive_;
  rclcpp::TimerBase::SharedPtr poller_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_NODE_HPP
