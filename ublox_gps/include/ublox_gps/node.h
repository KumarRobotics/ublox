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

#ifndef UBLOX_GPS_NODE_H
#define UBLOX_GPS_NODE_H

// STL
#include <vector>
#include <set>
// Boost
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
// ROS includes
#include <ros/ros.h>
#include <ros/serialization.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
// ROS messages
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
// Other U-Blox package includes
#include <ublox_msgs/ublox_msgs.h>
// Ublox GPS includes
#include <ublox_gps/gps.h>
#include <ublox_gps/utils.h>

namespace ublox_node {
/**
 * Abstract class representing a U-blox node.
 */
class UbloxNode {
 public:
  // Queue size for ROS publishers
  const static uint32_t kROSQueueSize = 1;
  // Subscribe Rate for U-Blox SV Info messages
  const static uint32_t kNavSvInfoSubscribeRate = 20; // [Hz]
  // how often (in seconds) to call poll messages
  const static double kPollDuration = 1.0;
  // Subscribe Rate of U-Blox msgs
  const static uint32_t kSubscribeRate = 1; // [Hz]
  // Constants used for diagnostic frequency updater
  const static float kDiagnosticPeriod = 0.2; // [s] 5Hz diagnostic period
  const static double kTolerance = 0.05;
  const static double kWindow = 10;
  const static double kTimeStampStatusMin = 0;
  // Default measurement period used during Survey-In [s]
  const static uint16_t kDefaultMeasPeriod = 250;

  /**
   * @brief Set the node handle parameter.
   */ 
  void setNh(boost::shared_ptr<ros::NodeHandle> nh) {
    nh_ = nh;
  }

  /**
   * @brief Set the protocol version from the device information.
   */
  void setProtocolVersion(float protocol_version) {
    protocol_version_ = protocol_version;
  }

  /**
   * @brief Get the protocol version of the device.
   */
  float getProtocolVersion() {
    return protocol_version_;
  }

  /**
   * @brief Set product category (e.g. SPG, HPG, ADR, FTS, etc.) from the 
   * device information.
   */
  void setProductCategory(std::string product_category) {
    product_category_ = product_category;
  }

  /**
   * Whether or not this device is a High Precision GNSS.
   */
  bool isHighPrecision() {
    return product_category_.compare("HPG") == 0;
  }

  /**
   * @gnss The string representing the GNSS. Refer MonVER message protocol.
   * i.e. GPS, GLO, GAL, BDS, QZSS, SBAS, IMES
   * @return true if the device supports the given GNSS
   */
  bool supportsGnss(std::string gnss) {
    return supported_.count(gnss) > 0;
  }

  /**
   * @brief Publish a ROS message of type MessageT. Should be used to publish
   * all messages which are simply read from U-blox and published.
   * @param m the message to publish
   * @param topic the topic to publish the message on
   */
  template <typename MessageT>
  void publish(const MessageT& m, const std::string& topic);

 protected:
  // Current mode of U-Blox
  enum {INIT, FIXED, SURVEY_IN, TIME} mode_;
  // ROS objects
  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<diagnostic_updater::Updater> updater_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag_;

  ublox_gps::Gps gps_;

  /* Variables set from parameter server */
  // Whether the message subscribers are enabled
  std::map<std::string, bool> enabled_;
  std::string frame_id_, device_, dynamic_model_, fix_mode_;
  // Set from dynamic model & fix mode strings
  uint8_t dmodel_, fmode_;
  // UART baudrate and in/out protocol (see CfgPRT message for constants)
  int baudrate_, uart_in_, uart_out_; 
  int rate_, meas_rate_, nav_rate_;
  // Settings for High Precision GNSS
  int tmode3_, dgnss_mode_;
  
  // Fixed mode settings
  bool lla_flag_;
  // Antenna Reference Point Position [m]
  std::vector<float> arp_position_;
  // Antenna Reference Point Position - High Precision [0.1 mm]
  std::vector<float> arp_position_hp_;
  // Fixed Position Accuracy [m]
  float fixed_pos_acc_;
  
  // Survey in settings
  // Survey in minimum duration [s]
  int sv_in_min_dur_;
  // Survey in accuracy limit [m]
  float sv_in_acc_lim_;
  // IDs of RTCM out messages to configure
  std::vector<int> rtcm_ids_;
  // Rate of RTCM out messages [Hz]
  int rtcm_rate_;
  // If true: enable the GNSS
  bool enable_gps_, enable_sbas_, enable_galileo_, enable_beidou_, enable_imes_; 
  bool enable_qzss_, enable_glonass_, enable_ppp_;
  int qzss_sig_cfg_, sbas_usage_, max_sbas_, dr_limit_;
  int fix_status_service_;

  /** Determined From Mon VER */
  float protocol_version_;
  // Product Category, e.g. SPG, HPG, see documentation for types
  std::string product_category_;
  // Which GNSS are supported by the device
  std::set<std::string> supported_;

  /**
   * @brief Initialize the U-blox node. Configure the U-blox and subscribe to 
   * messages.
   */
  void initialize();

  /**
   * Process the MonVer message. Find the protocol version, hardware type and 
   * supported GNSS.
   */
  void processMonVer(ublox_msgs::MonVER monVer);

  /**
   * @brief Handle to send fix status to ROS diagnostics.
   */
  virtual void fixDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& stat) = 0;

  /**
   * @brief Configure the U-Blox GNSS settings.
   */
  virtual void configureGnss() = 0;

  /* 
   * @brief Subscribe to ublox messages available on this version. 
   * (e.g. NavPVT, RxmRAWX)
   */
  virtual void subscribeVersion() = 0; 

 private:
  // Asynchronous IO objects
  boost::asio::io_service io_service_;
  boost::shared_ptr<boost::asio::serial_port> serial_handle_;
  boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_handle_;
  
  /**
   * @brief Set class parameters from the ROS node parameters.
   * @return 0 if successful
   */
  int setParams();

  /**
   * @brief Poll messages from the U-Blox device.
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Initialize the Serial / TCP IO.
   */
  void initializeIo();
  
  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initDiagnostics();

  /**
   * @brief Configure Ublox device based on requested settings.
   * @return true if configured succesfully
   */
  bool configureUblox();

  /*
   * @brief Subscribe to all requested U-Blox messages. Call subscribe version.
   */
  void subscribeAll();
};

/**
 * Represents a U-Blox node for firmware version 6.
 */
class UbloxNode6 : public UbloxNode {
 public:
  UbloxNode6(boost::shared_ptr<ros::NodeHandle> _nh);

  /*
   * @brief Publish a NavPOSLLh message & update the fix diagnostics & 
   * last known position.
   */
  void publishNavPosLlh(const ublox_msgs::NavPOSLLH& m);
  
  /*
   * @brief Publish a NavVELNED message & update the last known velocity.
   */
  void publishNavVelNed(const ublox_msgs::NavVELNED& m);

  /*
   * @brief Publish a NavSOL message and update the number of SVs used for the 
   * fix.
   */
  void publishNavSol(const ublox_msgs::NavSOL& m);
 protected:
  /**
   * @brief Updates fix diagnostic from NavPOSLLH, NavVELNED, and NavSOL 
   * messages.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Prints a warning, GNSS configuration not available in this version.
   */
  void configureGnss();
    /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribeVersion();

 private:
  // The last received navigation position
  ublox_msgs::NavPOSLLH last_nav_pos_;
  // The last received navigation velocity
  ublox_msgs::NavVELNED last_nav_vel_;
  // The last received num svs used
  int num_svs_used_;
  // The last Nav Sat fix based on last_nav_pos_
  sensor_msgs::NavSatFix fix;
  // The last Twist based on last_nav_vel_
  geometry_msgs::TwistWithCovarianceStamped velocity;
  // The last received nav status
  ublox_msgs::NavSTATUS status;
};

/**
 * Abstract class defining functions applicable to Firmware versions >=7.
 */
class UbloxNode7Plus : public UbloxNode {
 public:
  /**
   * Publish a NavPVT message. Also publishes Fix and Twist messages and
   * updates the fix diagnostics.
   */
  void publishNavPvt(const ublox_msgs::NavPVT& m);
  
 
 protected:
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /* Implement these in version specific classes */
  virtual void configureGnss() = 0;
  virtual void subscribeVersion() = 0; 
  ublox_msgs::NavPVT last_nav_pvt_; 
};

/**
 * Represents a U-Blox node for firmware version 7.
 */
class UbloxNode7 : public UbloxNode7Plus {
 public:
  UbloxNode7(boost::shared_ptr<ros::NodeHandle> _nh);

 protected:
  /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribeVersion();
  /**
   * @brief Configures GNSS individually. Only configures GLONASS.
   */
  void configureGnss();
};

/**
 * Represents a U-Blox node for firmware version 8.
 */
class UbloxNode8 : public UbloxNode7Plus {
 public:
  UbloxNode8(boost::shared_ptr<ros::NodeHandle> _nh);

 protected:
  /**
   * @brief Subscribes to NavPVT, RxmRAWX, and RxmSFRBX messages. 
   */
  void subscribeVersion();
  /**
   * @brief Configures all GNSS systems in 1 message based on ROS params and
   * configure DGNSS.
   */
  void configureGnss();
  /**
   * @brief Only for High Precision GNSS Devices. Publishes received Nav SVIN
   * messages. When the survey in finishes,  it changes the measurement & 
   * navigation rate to the user configured values and enables the user
   * configured RTCM messages.
   */
  void publishNavSvIn(ublox_msgs::NavSVIN msg);
};

}

#endif