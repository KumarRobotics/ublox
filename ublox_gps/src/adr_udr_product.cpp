#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/hnr_pvt.hpp>
#include <ublox_msgs/msg/nav_att.hpp>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// Extract U-Blox 24-bit signed integers from a 32-bit data blob
// and cast to int32_t
//
static inline std::int32_t extract_int24(std::uint32_t bitfield) {
    std::int32_t temp = static_cast<std::int32_t>(bitfield << 8);
    return temp >> 8;
}

//
// U-Blox Automotive or Untethered Dead Reckoning
// High Precision GNSS products have have firmware version >= 8
//
AdrUdrProduct::AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, const bool use_highprecision, rclcpp::Node* node)
  : protocol_version_(protocol_version), use_adr_(false), nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater), use_highprecision_(use_highprecision), node_(node)
{
  if (getRosBoolean(node_, "publish.esf.meas")) {
    imu_pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("imu_meas", 1);
    esf_meas_pub_ = node_->create_publisher<ublox_msgs::msg::EsfMEAS>("esfmeas", 1);
  }
  if (getRosBoolean(node_, "publish.nav.att")) {
    nav_att_pub_ = node_->create_publisher<ublox_msgs::msg::NavATT>("navatt", 1);
    imu_att_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu_att", 1);
  }
  if (getRosBoolean(node_, "publish.nav.pvt")) {
    nav_pvt_pub_ = node_->create_publisher<ublox_msgs::msg::NavPVT>("navpvt", 1);
  }
  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_ = node_->create_publisher<ublox_msgs::msg::EsfINS>("esfins", 1);
    esf_ins_ros_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/kinematics", 1);
  }
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_ = node_->create_publisher<ublox_msgs::msg::EsfRAW>("esfraw", 1);
    imu_raw_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu_raw", 1);
  }
  if (getRosBoolean(node_, "publish.esf.status")) {
    esf_status_pub_ = node_->create_publisher<ublox_msgs::msg::EsfSTATUS>("esfstatus", 1);
  }
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    hnr_pvt_pub_ = node_->create_publisher<ublox_msgs::msg::HnrPVT>("hnrpvt", 1);
  }
  if (getRosBoolean(node_, "publish.nav.hpposllh")) {
    if (use_highprecision_) {
      nav_hpposllh_pub_ = node_->create_publisher<ublox_msgs::msg::NavHPPOSLLH>("navhpposllh", 1);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Parameter 'publish.nav.hpposllh' is enabled, but this device is not recognized as a high-precision product.");
    }
  }
  if (getRosBoolean(node_, "publish.nav.hpposecef")) {
    if (use_highprecision_) {
      nav_hpposecef_pub_ = node_->create_publisher<ublox_msgs::msg::NavHPPOSECEF>("navhpposecef", 1);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Parameter 'publish.nav.hpposecef' is enabled, but this device is not recognized as a high-precision product.");
    }
  }
  if (use_highprecision_) {
    fix_hp_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("~/fix_highprecision", 1);
  }
  if (getRosBoolean(node_, "publish.esf.meas") || getRosBoolean(node_, "publish.esf.raw")) {
    imu_temp_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>("~/imu_temperature", 1);
  }
  nav_diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/fusion_status", 1);

  // Perform any message metadata value setting we can do only once, including default values
  // This only improves performance a little, but removes duplcate code
  imu_.header.frame_id = frame_id_;
  imu_.orientation_covariance[0] = -1.0;
  imu_.orientation_covariance[4] = -1.0;
  imu_.orientation_covariance[8] = -1.0;
  imu_.linear_acceleration_covariance[0] = -1.0;
  imu_.linear_acceleration_covariance[4] = -1.0;
  imu_.linear_acceleration_covariance[8] = -1.0;
  imu_.angular_velocity_covariance[0] = -1.0;
  imu_.angular_velocity_covariance[4] = -1.0;
  imu_.angular_velocity_covariance[8] = -1.0;

  imu_raw_ = imu_;  // Initialize using the same values as bove
  imu_att_ = imu_;  // Initialize using the same values as above
  esf_ins_ros_ = imu_;  // Initialize using the same values as above

  fix_hp_.header.frame_id = frame_id_;
  fix_hp_.position_covariance[0] = -1.0;
  fix_hp_.position_covariance[4] = -1.0;
  fix_hp_.position_covariance[8] = -1.0;
  fix_hp_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  nav_diag_.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  nav_diag_.name = "NavigationDiagnostics";
  nav_diag_.message = "EsfSTATUS";
  nav_diag_.hardware_id = frame_id_;
}

void AdrUdrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {

  // Subscribe to ADR/UDR Navigation Attitude messages
  if (getRosBoolean(node_, "publish.nav.att")) {
    gps->subscribe<ublox_msgs::msg::NavATT>(std::bind(
      &AdrUdrProduct::callbackNavATT, this, std::placeholders::_1), 1);
  }

  // Subscribe to ADR/UDR Inertial Navigation System kinematics messages
  if (getRosBoolean(node_, "publish.esf.ins")) {
    gps->subscribe<ublox_msgs::msg::EsfINS>(std::bind(
      &AdrUdrProduct::callbackEsfIns, this, std::placeholders::_1), 1);
  }

  // Subscribe to ADR/UDR Post-Processed IMU messages
  if (getRosBoolean(node_, "publish.esf.meas")) {
    gps->subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
      &AdrUdrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);
  }

  // Subscribe to ADR/UDR Raw IMU messages
  if (getRosBoolean(node_, "publish.esf.raw")) {
    gps->subscribe<ublox_msgs::msg::EsfRAW>(std::bind(
      &AdrUdrProduct::callbackEsfRAW, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Status messages
  if (getRosBoolean(node_, "publish.esf.status")) {
    gps->subscribe<ublox_msgs::msg::EsfSTATUS>(std::bind(
      &AdrUdrProduct::callbackEsfStatus, this, std::placeholders::_1), 1);
  }

  // Subscribe to High-Navigation rate PVT messages
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    gps->subscribe<ublox_msgs::msg::HnrPVT>(
      [this](const ublox_msgs::msg::HnrPVT &m) { hnr_pvt_pub_->publish(m); },
      1);
  }

  // Subscribe to High-Precision Lat-Lon-Height messages; only in firmware >= 8
  if (use_highprecision_) {
    gps->subscribe<ublox_msgs::msg::NavHPPOSLLH>(std::bind(
      &AdrUdrProduct::callbackNavHpPosLlh, this, std::placeholders::_1), 1);
    gps->subscribe<ublox_msgs::msg::NavHPPOSECEF>(std::bind(
      &AdrUdrProduct::callbackNavHpPosEcef, this, std::placeholders::_1), 1);
  }
  // Subscribe to the Position-Velocity-Time solution messages.
  gps->subscribe<ublox_msgs::msg::NavPVT>(std::bind(
    &AdrUdrProduct::callbackNavPVT, this, std::placeholders::_1), 1);

}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m) {
  const rclcpp::Time callback_time{node_->now()};
  // This is time-critical data, so if something is expecting it, republish early
  if (getRosBoolean(node_, "publish.esf.meas")) {
    esf_meas_pub_->publish(m);
  }

  for (const std::uint32_t datapoint : m.data) {
    //grab the last six bits of data as the data type description field
    const std::uint8_t data_type = datapoint >> 24;
    // Interpret the first 24 bits as a signed integer
    const std::int32_t data_value = extract_int24(datapoint);
    switch (data_type) {
      case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
        imu_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
        imu_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
        imu_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
        imu_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
        imu_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
       imu_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
        last_imu_temperature_ = static_cast<double>(data_value) * kConvertDegCelsius;
        break;
      // The following are not currently used
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_LEFT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_LEFT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_RIGHT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SINGLE_TICK:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SPEED:
        break;
      default:
          RCLCPP_INFO(node_->get_logger(), "Unknown IMU measurement, data_type: %u , data_value: %d", data_type, data_value);
    }
    imu_.header.stamp = callback_time;
    imu_pub_->publish(imu_);
  }
}

void AdrUdrProduct::getRosParams() {
  use_adr_ = getRosBoolean(node_, "use_adr");
  // Check the nav rate
  float nav_rate_hz = 1000.0 / (meas_rate_ * nav_rate_);
  if (nav_rate_hz != 1) {
    RCLCPP_WARN(node_->get_logger(), "Nav Rate recommended to be 1 Hz");
  }
}

bool AdrUdrProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (!gps->setUseAdr(use_adr_, protocol_version_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::callbackNavATT(const ublox_msgs::msg::NavATT &m) {
  imu_att_.header.stamp = node_->now();
  if (getRosBoolean(node_, "publish.nav.att")) {
    nav_att_pub_->publish(m);
  }
  last_nav_att_ = m;

  constexpr double kNavAttScaleAndRadianConversion{1e-5 * M_PI / 180.0};
  // Transform U-Blox Euler angles to a Quaternion and populate covariances
  const double roll = M_PI_2 - (static_cast<double>(m.roll) * kNavAttScaleAndRadianConversion);
  const double pitch = M_PI_2 - (static_cast<double>(m.pitch) * kNavAttScaleAndRadianConversion);
  const double heading = M_PI_2 - (static_cast<double>(m.heading) * kNavAttScaleAndRadianConversion);
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, heading);

  imu_att_.orientation.x = orientation[0];
  imu_att_.orientation.y = orientation[1];
  imu_att_.orientation.z = orientation[2];
  imu_att_.orientation.w = orientation[3];

  imu_att_.orientation_covariance[0] = std::pow(static_cast<double>(m.acc_roll) * kNavAttScaleAndRadianConversion, 2.0);
  imu_att_.orientation_covariance[4] = std::pow(static_cast<double>(m.acc_pitch) * kNavAttScaleAndRadianConversion, 2.0);
  imu_att_.orientation_covariance[8] = std::pow(static_cast<double>(m.acc_heading) * kNavAttScaleAndRadianConversion, 2.0);

  imu_att_pub_->publish(imu_att_);
}

//
// Publish a sensor_msgs/msg/Imu message for the intertial navigation solution
//
void AdrUdrProduct::callbackEsfIns(const ublox_msgs::msg::EsfINS &m) {
  esf_ins_ros_.header.stamp = node_->now();

  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_->publish(m);
  }

  constexpr double kScaleNewtons{1e-6};
  constexpr double kScaleAndRadianConversion{1e-3 * M_PI / 180.0};

  esf_ins_ros_.angular_velocity.x = static_cast<double>(m.x_ang_rate) * kScaleAndRadianConversion;
  esf_ins_ros_.angular_velocity.y = static_cast<double>(m.y_ang_rate) * kScaleAndRadianConversion;
  esf_ins_ros_.angular_velocity.z = static_cast<double>(m.z_ang_rate) * kScaleAndRadianConversion;

  esf_ins_ros_.linear_acceleration.x = static_cast<double>(m.x_accel) * kScaleNewtons;
  esf_ins_ros_.linear_acceleration.y = static_cast<double>(m.y_accel) * kScaleNewtons;
  esf_ins_ros_.linear_acceleration.z = static_cast<double>(m.z_accel) * kScaleNewtons;

  // EsfINS does not contain all the data we want, so we use
  // the last NavATT message for orientation if its iTOW frame matches this one
  const ublox_msgs::msg::NavATT nav_att = last_nav_att_;
  if (nav_att.i_tow == m.i_tow) {
    constexpr double kEsfInsScaleAndRadianConversion{1e-5 * M_PI / 180.0};

    const double roll = M_PI_2 - (static_cast<double>(nav_att.roll) * kEsfInsScaleAndRadianConversion);
    const double pitch = M_PI_2 - (static_cast<double>(nav_att.pitch) * kEsfInsScaleAndRadianConversion);
    const double heading = M_PI_2 - (static_cast<double>(nav_att.heading) * kEsfInsScaleAndRadianConversion);
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, heading);  // Translate from Euler angles to a Quaternion
    esf_ins_ros_.orientation.x = orientation[0];
    esf_ins_ros_.orientation.y = orientation[1];
    esf_ins_ros_.orientation.z = orientation[2];
    esf_ins_ros_.orientation.w = orientation[3];

    esf_ins_ros_.orientation_covariance[0] =
        std::pow(static_cast<double>(nav_att.acc_roll) * kEsfInsScaleAndRadianConversion, 2);
    esf_ins_ros_.orientation_covariance[4] =
        std::pow(static_cast<double>(nav_att.acc_pitch) * kEsfInsScaleAndRadianConversion, 2);
    esf_ins_ros_.orientation_covariance[8] =
        std::pow(static_cast<double>(nav_att.acc_heading) * kEsfInsScaleAndRadianConversion, 2);
  } else {  // No data available for this data frame, mark as invalid
    esf_ins_ros_.orientation_covariance[0] = -1.0;
    esf_ins_ros_.orientation_covariance[4] = -1.0;
    esf_ins_ros_.orientation_covariance[8] = -1.0;
  }
  esf_ins_ros_pub_->publish(esf_ins_ros_);
}

const char* wt_status_str(std::uint8_t bitfield) {
  std::uint8_t wt_status = bitfield & 0xb00000011;
  if (wt_status == 2) return "calibrated";
  if (wt_status == 1) return "initializing";
  if (wt_status == 0) return "off";
  return "deserialization error";
}

const char* imu_alg_str(std::uint8_t bitfield) {
  std::uint8_t alg_status = bitfield & 0xb00011100;
  if (alg_status == 2) return "calibrated";
  if (alg_status == 1) return "initializing";
  if (alg_status == 0) return "off";
  return "deserialization error";
}

const char* ins_init_str(std::uint8_t bitfield) {
  std::uint8_t ins_status = bitfield & 0xb01100000;
  if (ins_status == 2) return "calibrated";
  if (ins_status == 1) return "initializing";
  if (ins_status == 0) return "off";
  return "deserialization error";
}

const char* imu_init_str(std::uint8_t bitfield) {
  std::uint8_t imu_status = bitfield & 0xb00000011;
  if (imu_status == 2) return "calibrated";
  if (imu_status == 1) return "initializing";
  if (imu_status == 0) return "off";
  return "deserialization error";
}

//
// Parse the sensor fusion status information
//
void AdrUdrProduct::callbackEsfStatus(const ublox_msgs::msg::EsfSTATUS &m) {
  const rclcpp::Time callback_time{node_->now()};
  diagnostic_msgs::msg::KeyValue wt_status;
  wt_status.key = "wheel_tick_status";
  wt_status.value = wt_status_str(m.reserved1[0]);
  diagnostic_msgs::msg::KeyValue imu_alg;
  imu_alg.key = "IMU_alignment_status";
  imu_alg.value = imu_alg_str(m.reserved1[0]);
  diagnostic_msgs::msg::KeyValue ins_ini;
  ins_ini.key = "INS_init_status";
  ins_ini.value = ins_init_str(m.reserved1[0]);
  diagnostic_msgs::msg::KeyValue imu_ini;
  imu_ini.key = "IMU_init_status";
  imu_ini.value = imu_init_str(m.reserved1[1]);
  diagnostic_msgs::msg::KeyValue fusion_mode;
  fusion_mode.key = "fusion_mode";
  fusion_mode.value = m.fusion_mode == 3 ? "disabled_fault" : (m.fusion_mode == 2 ? "suspended" : (m.fusion_mode == 1 ? "online" : "initializing"));
  diagnostic_msgs::msg::KeyValue num_sens;
  num_sens.key = "num_sensors";
  num_sens.value = std::to_string(m.num_sens);

  nav_diag_.values.push_back(imu_alg);
  nav_diag_.values.push_back(imu_ini);
  nav_diag_.values.push_back(wt_status);
  nav_diag_.values.push_back(ins_ini);
  nav_diag_.values.push_back(fusion_mode);
  nav_diag_.values.push_back(num_sens);

  nav_diag_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  nav_diag_pub_->publish(nav_diag_);

  if (getRosBoolean(node_, "publish.esf.meas") || getRosBoolean(node_, "publish.esf.raw")) {
    sensor_msgs::msg::Temperature temp;
    temp.header.stamp = callback_time;
    temp.temperature = last_imu_temperature_;
    temp.variance = 0.0;
    imu_temp_pub_->publish(temp);
  }
}


//
// Decode the Raw IMU measurement output
//
void AdrUdrProduct::callbackEsfRAW(const ublox_msgs::msg::EsfRAW &m) {
  const rclcpp::Time callback_time{node_->now()};
  // This is time-critical data, so if something is expecting it, republish early
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_->publish(m);
  }

  for (const ublox_msgs::msg::EsfRAWBlock &imu_data_entry : m.blocks) {
    const std::uint32_t datapoint = imu_data_entry.data;
    // Grab the last six bits of data as the data type description field
    const std::uint8_t data_type = datapoint >> 24;
    // Interpret the first 24 bits as a signed integer, and cast to double
    const std::int32_t data_value = extract_int24(datapoint);

    switch (data_type) {
      case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
        imu_raw_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
        imu_raw_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
        imu_raw_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
        imu_raw_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
        imu_raw_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
        imu_raw_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMps2;
        break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
	    last_imu_temperature_ = static_cast<double>(data_value) * kConvertDegCelsius;
        break;
      // The following are not currently used; they relate to sensor fusion with wheel encoder data
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_LEFT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_LEFT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_RIGHT:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SINGLE_TICK:
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SPEED:
        break;
      default:
        RCLCPP_INFO(node_->get_logger(), "Unknown IMU measurement, data_type: %u , data_value: %d", data_type, data_value);
    }
    imu_raw_.header.stamp = callback_time;
    imu_raw_pub_->publish(imu_raw_);
  }
}

//
// Republish the High-Precision ECEF Position message (HPPOSECEF)
//
void AdrUdrProduct::callbackNavHpPosEcef(const ublox_msgs::msg::NavHPPOSECEF& m) {
  if (getRosBoolean(node_, "publish.nav.hpposecef")) {
    nav_hpposecef_pub_->publish(m);
  }
}

//
// Decode the High-Precision Geodetic Position message (HPPOSLLH) into NavSatFix
//
void AdrUdrProduct::callbackNavHpPosLlh(const ublox_msgs::msg::NavHPPOSLLH& m) {
  fix_hp_.header.stamp = node_->now();  // Ideally, we should get a timestamp from the device
  // Do not publish invalid HPPOSLLH data
  if (m.flags != 0) {
    return;
  }

  if (getRosBoolean(node_, "publish.nav.hpposllh")) {
    nav_hpposllh_pub_->publish(m);
  }

  if (last_nav_pvt_.fix_type >= ublox_msgs::msg::NavPVT::FIX_TYPE_2D) {
    fix_hp_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    fix_hp_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  // Calculate the high-precision lat, lon, alt values
  fix_hp_.latitude = 1e-7 *  (static_cast<double>(m.lat) + (static_cast<double>(m.lat_hp) * 1e-9));
  fix_hp_.longitude = 1e-7 * (static_cast<double>(m.lon) + (static_cast<double>(m.lon_hp) * 1e-9));
  fix_hp_.altitude = 1e-3 * (static_cast<double>(m.height) + (static_cast<double>(m.height_hp) * 1e-1));

  // Populate the covariance data
  const double var_h = std::pow(static_cast<double>(m.h_acc) * 1e-3, 2.0);
  const double var_v = std::pow(static_cast<double>(m.v_acc) * 1e-3, 2.0);
  fix_hp_.position_covariance[0] = var_h;
  fix_hp_.position_covariance[4] = var_h;
  fix_hp_.position_covariance[8] = var_v;
  fix_hp_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_hp_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix_hp_pub_->publish(fix_hp_);
}

//
// Use NavPVT for some message fusion diagnostics data
//
void AdrUdrProduct::callbackNavPVT(const ublox_msgs::msg::NavPVT& m) {
  if (getRosBoolean(node_, "publish.nav.pvt")) {
    nav_pvt_pub_->publish(m);
  }
  last_nav_pvt_ = m;

  // update the iTow timestamp
  uint8_t valid_time = m.VALID_DATE | m.VALID_TIME | m.VALID_FULLY_RESOLVED;
  if (((m.valid & valid_time) == valid_time) &&
      (m.flags2 & m.FLAGS2_CONFIRMED_AVAILABLE)) {
    // Use the NavPVT timestamp since it reflects the device computation time
    // The nanosecond time from the NavPVT message can be between -1e9 and 1e9
    // rclcpp::Time uses only unsigned values, so a negative nanosecond value
    // must be converted to a positive value
    last_itow_time_.first = m.i_tow;
    if (m.nano < 0) {
      last_itow_time_.second.sec = ublox_node::toUtcSeconds(m) - 1;
      last_itow_time_.second.nanosec = static_cast<uint32_t>(m.nano + 1e9);
    }
    else {
      last_itow_time_.second.sec = ublox_node::toUtcSeconds(m);
      last_itow_time_.second.nanosec = static_cast<uint32_t>(m.nano);
    }
  }
}

}  // namespace ublox_node
