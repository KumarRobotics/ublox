#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/hnr_pvt.hpp>
#include <ublox_msgs/msg/nav_att.hpp>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

// Scaling factors for floating point representations of data
constexpr float kConvertRadPerSec{std::pow(2, -12) * M_PI / 180.0F};
constexpr float kConvertMps2{std::pow(2, -10)};
constexpr float kConvertDegCelsius{0.01F};

//
// Extract U-Blox 24-bit signed integers from a 32-bit data blob
// and cast into int32_t
//
static inline std::int32_t extract_int24(std::uint32_t bitfield) {
    std::int32_t temp = static_cast<std::int32_t>(bitfield << 8);
    return temp >> 8;
}

//
// u-blox ADR devices, partially implemented
//
AdrUdrProduct::AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node)
  : use_adr_(false), nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater), node_(node)
{
  if (getRosBoolean(node_, "publish.esf.meas")) {
    imu_pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("imu_meas", 1);
    time_ref_pub_ =
      node_->create_publisher<sensor_msgs::msg::TimeReference>("interrupt_time", 1);
    esf_meas_pub_ = node_->create_publisher<ublox_msgs::msg::EsfMEAS>("esfmeas", 1);
  }
  if (getRosBoolean(node_, "publish.nav.att")) {
    nav_att_pub_ = node_->create_publisher<ublox_msgs::msg::NavATT>("navatt", 1);
  }
  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_ = node_->create_publisher<ublox_msgs::msg::EsfINS>("esfins", 1);
  }
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_ = node_->create_publisher<ublox_msgs::msg::EsfRAW>("esfraw", 1);
    imu_raw_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1);
  }
  if (getRosBoolean(node_, "publish.esf.status")) {
    esf_status_pub_ = node_->create_publisher<ublox_msgs::msg::EsfSTATUS>("esfstatus", 1);
  }
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    hnr_pvt_pub_ = node_->create_publisher<ublox_msgs::msg::HnrPVT>("hnrpvt", 1);
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
  if (!gps->setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to NAV ATT messages
  if (getRosBoolean(node_, "publish.nav.att")) {
    gps->subscribe<ublox_msgs::msg::NavATT>([this](const ublox_msgs::msg::NavATT &m) { nav_att_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF INS messages
  if (getRosBoolean(node_, "publish.esf.ins")) {
    gps->subscribe<ublox_msgs::msg::EsfINS>([this](const ublox_msgs::msg::EsfINS &m) { esf_ins_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF Meas messages
  if (getRosBoolean(node_, "publish.esf.meas")) {
    gps->subscribe<ublox_msgs::msg::EsfMEAS>([this](const ublox_msgs::msg::EsfMEAS &m) { esf_meas_pub_->publish(m); },
                                        1);
    // also publish sensor_msgs::Imu
    gps->subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
      &AdrUdrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Raw messages
  if (getRosBoolean(node_, "publish.esf.raw")) {
    gps->subscribe<ublox_msgs::msg::EsfRAW>([this](const ublox_msgs::msg::EsfRAW &m) { esf_raw_pub_->publish(m); },
                                       1);

    // also publish sensor_msgs::Imu for the raw data
    gps->subscribe<ublox_msgs::msg::EsfRAW>(std::bind(
      &AdrUdrProduct::callbackEsfRAW, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Status messages
  if (getRosBoolean(node_, "publish.esf.status")) {
    gps->subscribe<ublox_msgs::msg::EsfSTATUS>([this](const ublox_msgs::msg::EsfSTATUS &m) { esf_status_pub_->publish(m); },
                                          1);
  }

  // Subscribe to HNR PVT messages
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    gps->subscribe<ublox_msgs::msg::HnrPVT>([this](const ublox_msgs::msg::HnrPVT &m) { hnr_pvt_pub_->publish(m); },
                                       1);
  }
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m) {
  if (getRosBoolean(node_, "publish.esf.meas")) {
    imu_.header.stamp = node_->now();
    imu_.header.frame_id = frame_id_;
    imu_.orientation_covariance[0] = -1;
    imu_.linear_acceleration_covariance[0] = -1;
    imu_.angular_velocity_covariance[0] = -1;

    const std::vector<unsigned int> imu_data = m.data;
    for (const std::uint32_t datapoint : imu_data) {
      unsigned int data_type = datapoint >> 24; //grab the last six bits of data
      double data_sign = (datapoint & (1 << 23)); //grab the sign (+/-) of the rest of the data
      unsigned int data_value = datapoint & 0x7FFFFF; //grab the rest of the data...should be 23 bits

      //grab the last six bits of data as the data type description field
      const std::uint8_t data_type = datapoint >> 24;
      // Interpret the first 24 bits as a signed integer
      const std::int32_t data_value = extract_int24(datapoint);

      switch (data_type) {
        case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
          imu_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
          imu_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMps2;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
          imu_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
          imu_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMps2;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
          break;
          imu_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
          imu_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMps2;
          break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
          imu_temp_.temperature = static_cast<double>(data_value) * kConvertDegCelsius);
          break;  // Reserved for future use
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

      // create time ref message and put in the data
      t_ref_.header.seq = m.risingEdgeCount;
      t_ref_.header.stamp = node_->now();
      t_ref_.header.frame_id = frame_id_;

      // Consider removing the below
      t_ref_.time_ref = rclcpp::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

      time_ref_pub_->publish(t_ref_);
      imu_pub_->publish(imu_);

      // Consider removing the below
      //std::ostringstream src;
      //src << "TIM" << int(m.ch);
      //t_ref_.source = src.str();
    }
  }

}

void AdrUdrProduct::callbackEsfRAW(const ublox_msgs::msg::EsfRAW &m) {
  if (getRosBoolean(node_, "publish.esf.raw")) {
    imu_raw_.header.stamp = node_->now();
    imu_raw_.header.frame_id = frame_id_;
    imu_raw_.orientation_covariance[0] = -1;
    imu_raw_.linear_acceleration_covariance[0] = -1;
    imu_raw_.angular_velocity_covariance[0] = -1;

    const std::vector<ublox_msgs::msg::EsfRAWBlock> imu_data_blocks = m.blocks;
    for (const ublox_msgs::msg::EsfRAWBlock &imu_data_entry : imu_data_blocks) {
      const std::uint32_t datapoint = imu_data_entry.data;

      //grab the last six bits of data as the data type description field
      const std::uint8_t data_type = datapoint >> 24;
      // Interpret the first 24 bits as a signed integer, and cast to double
      const std::int32_t data_value = extract_int24(datapoint);

      switch (data_type) {
        case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
          imu_raw_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
          imu_raw_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMps2; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
          imu_raw_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
          imu_raw_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMps2; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
          imu_raw_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
          imu_raw_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMps2; break;
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_LEFT:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_LEFT:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_RIGHT:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SINGLE_TICK:
          break;  // Reserved for future use
        case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SPEED:
          break;  // Reserved for future use
        default:
          RCLCPP_INFO(node_->get_logger(), "Unknown IMU measurement, data_type: %u , data_value: %d", data_type, data_value);
      }
    }
    imu_raw_pub_->publish(imu_raw_);
  }
}

>>>>>>> 0cb2c06 (Add full product support for ZED-F9R)
}  // namespace ublox_node
