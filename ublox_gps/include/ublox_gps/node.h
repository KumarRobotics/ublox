#ifndef UBLOX_GPS_NODE_H
#define UBLOX_GPS_NODE_H

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <ublox_gps/gps.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/CfgGNSS_Block.h>
#include <ublox_msgs/CfgPRT.h>

namespace ublox_node {
/**
 * Abstract class representing a U-blox node.
 */
class UbloxNode {
 public:
  const static uint32_t kROSQueueSize = 1;
  // Constants used for diagnostic frequency updater
  const static double kTolerance = 0.05;
  const static double kWindow = 10;
  const static double kTimeStampStatusMin = 0;

  /**
   * @brief Set the node handle parameter.
   */ 
  void setNh(boost::shared_ptr<ros::NodeHandle> _nh) {
    nh = _nh;
  }

  /**
   * @brief Set class parameters from the ROS node parameters.
   * @return 0 if successful
   */
  int setParams();
  
  /**
   * @brief Publish a ROS message of type MessageT.
   * @param m the message to publish
   * @param topic the topic to publish the message on
   */
  template <typename MessageT>
  void publish(const MessageT& m, const std::string& topic);

  /**
   * @brief Poll messages from the U-Blox device.
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Initialize the U-blox node. Configure the U-blox and subscribe to 
   * messages.
   * @return 0 if successful
   */
  int initialize();

  /**
   * @brief Initialize the Serial / TCP IO.
   */
  int initializeIo();
  
  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initDiagnostics();

  /**
   * @brief Configure Ublox device based on requested settings.
   */
  void configureUblox();

  /*
   * @brief Subscribe to U-blox messages available on all firmware versions.
   */
  void subscribe();

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

  // Asynchronous IO objects
  boost::asio::io_service io_service;
  boost::shared_ptr<boost::asio::serial_port> serial_handle;
  boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_handle;

  // ROS objects
  boost::shared_ptr<ros::NodeHandle> nh;
  boost::shared_ptr<diagnostic_updater::Updater> updater;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;

  ublox_gps::Gps gps;

  // Variables set from parameter server
  std::map<std::string, bool> enabled;
  std::string frame_id_, device_, dynamic_model_, fix_mode_;
  // Set from dynamic model & fix mode strings
  uint8_t dmodel, fmode;
  // UART in out protocol
  int baudrate_, uart_in_, uart_out_; 
  int rate_, meas_rate_, nav_rate_, rtcm_rate_;
  std::vector<int> rtcm_ids_;
  bool enable_gps_, enable_sbas_, enable_galileo_, enable_beidou_, enable_imes_; 
  bool enable_qzss_, enable_glonass_, enable_ppp_;
  int dr_limit_, qzss_sig_cfg_;
  int fix_status_service_;
  int sbas_usage_, max_sbas_;
  // Used during initialization
  bool setup_ok_;
};

/**
 * Represents a U-Blox node for firmware version 6.
 */
class UbloxNode6 : public UbloxNode {
 public:
  UbloxNode6(boost::shared_ptr<ros::NodeHandle> _nh);

  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void configureGnss();
  void subscribeVersion();

  /*
   * @brief Publish a NavPOSLLh message & update the fix diagnostics & 
   * last known position.
   */
  void publishNavPOSLLH(const ublox_msgs::NavPOSLLH& m);
  
  /*
   * @brief Publish a NavVELNED message & update the last known velocity.
   */
  void publishNavVELNED(const ublox_msgs::NavVELNED& m);

  /*
   * @brief Publish a NavSOL message and update the number of SVs used for the 
   * fix.
   */
  void publishNavSOL(const ublox_msgs::NavSOL& m);
 
 private:
  ublox_msgs::NavPOSLLH last_nav_pos_;
  ublox_msgs::NavVELNED last_nav_vel_;
  int num_svs_used_;
  sensor_msgs::NavSatFix fix;
  geometry_msgs::TwistWithCovarianceStamped velocity;
  ublox_msgs::NavSTATUS status;
};

/**
 * Abstract class defining functions applicable to Firmware versions >=7.
 */
class UbloxNode7Plus : public UbloxNode {
 public:
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * Publish a NavPVT message and update the fix diagnostics.
   */
  void publishNavPVT(const ublox_msgs::NavPVT& m);
  
  /* Implement these in version specific classes */
  virtual void configureGnss() = 0;
  virtual void subscribeVersion() = 0; 
 
 protected:
  ublox_msgs::NavPVT last_nav_pvt_; 
};

/**
 * Represents a U-Blox node for firmware version 7.
 */
class UbloxNode7 : public UbloxNode7Plus {
 public:
  UbloxNode7(boost::shared_ptr<ros::NodeHandle> _nh);

  void subscribeVersion();
  void configureGnss();
};

/**
 * Represents a U-Blox node for firmware version 8.
 */
class UbloxNode8 : public UbloxNode7Plus {
 public:
  UbloxNode8(boost::shared_ptr<ros::NodeHandle> _nh);
  
  void subscribeVersion();
  void configureGnss();
};

}

#endif