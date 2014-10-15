//=================================================================================================
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

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include <boost/asio/io_service.hpp>
#include <vector>
#include <map>

#include <ublox/serialization/ublox_msgs.h>
#include <ublox_gps/async_worker.h>
#include <ublox_gps/callback.h>

namespace ublox_gps {

/**
 * @brief Dynamic model - consult documentation of CFG-NAV5 for details.
 * @note These numeric values must match the official protocol spec.
 */
enum DynamicModel {
  DYN_MODEL_PORTABLE = 0,
  DYN_MODEL_STATIONARY = 2,
  DYN_MODEL_PEDESTRIAN = 3,
  DYN_MODEL_AUTOMOTIVE = 4,
  DYN_MODEL_SEA = 5,
  DYN_MODEL_AIRBORNE_1G = 6,
  DYN_MODEL_AIRBORNE_2G = 7,
  DYN_MODEL_AIRBORNE_4G = 8,
};

/**
 * @brief Fix mode - 2D, 3D or both.
 * @note These numeric values must match the official protocol spec.
 */
enum FixMode {
  FIX_MODE_2D = 1,
  FIX_MODE_3D = 2,
  FIX_MODE_BOTH = 3,
};

enum 

/**
 * @brief Determine dynamic model from human-readable string.
 * @param model One of the following (case-insensitive):
 *  - portable
 *  - stationary
 *  - pedestrian
 *  - automotive
 *  - sea
 *  - airborne1
 *  - airborne2
 *  - airborne4
 * @return DynamicModel
 * @throws std::runtime_error on invalid argument.
 */
DynamicModel modelFromString(const std::string& model);

/**
 * @brief Determine fix mode from human-readable string.
 * @param mode One of the following (case-insensitive):
 *  - 2d
 *  - 3d
 *  - both
 * @return FixMode
 * @throws std::runtime_error on invalid argument.
 */
FixMode fixModeFromString(const std::string& mode);

class Gps
{
public:
  Gps();
  virtual ~Gps();

  template <typename StreamT> void initialize(StreamT& stream, boost::asio::io_service& io_service);
  void initialize(const boost::shared_ptr<Worker> &worker);
  void close();

  /**
   * @brief Set the device measurement rate.
   * @param measRate Period in milliseconds between subsequent measurements.
   * @return true on ACK, false on other conditions.
   */
  bool setMeasRate(uint16_t measRate);
  
  bool setBaudrate(unsigned int baudrate);
  bool setRate(uint8_t class_id, uint8_t message_id, unsigned int rate);
  
  /**
   * @brief Set the device dynamic model.
   * @param model Dynamic model to use. Consult ublox protocol spec for details.
   * @return true on ACK, false on other conditions.
   */
  bool setDynamicModel(DynamicModel model);
  
  /**
   * @brief Set the device fix mode.
   * @param mode 2D, 3D or both.
   * @return true on ACK, false on other conditions.
   */
  bool setFixMode(FixMode mode);
  
  /**
   * @brief Set the dead reckoning time limit
   * @param limit Time limit in seconds.
   * @return true on ACK, false on other conditions.
   */
  bool setDeadReckonLimit(uint8_t limit);
  
  /**
   * @brief Enable or disable PPP (precise-point-positioning).
   * @param enabled If true, PPP is enabled.
   * @return true on ACK, false on other conditions.
   * 
   * @note This is part of the expert settings. It is recommended you check
   * the ublox manual first.
   */
  bool setPPPEnabled(bool enabled);
  
  bool enableSBAS(bool enabled);

  template <typename T> Callbacks::iterator subscribe(typename CallbackHandler_<T>::Callback callback, unsigned int rate);
  template <typename T> Callbacks::iterator subscribe(typename CallbackHandler_<T>::Callback callback);
  template <typename T> bool read(T& message, const boost::posix_time::time_duration& timeout = default_timeout_);

  bool isInitialized() const { return worker_ != 0; }
  bool isConfigured() const { return isInitialized() && configured_; }
  bool isOpen() const { return worker_->isOpen(); }
  
  template <typename ConfigT> bool poll(ConfigT& message, const boost::posix_time::time_duration& timeout = default_timeout_);
  bool poll(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload = std::vector<uint8_t>());
  template <typename ConfigT> bool configure(const ConfigT& message, bool wait = true);
  void waitForAcknowledge(const boost::posix_time::time_duration& timeout);

private:
  void readCallback(unsigned char *data, std::size_t& size);

private:
  boost::shared_ptr<Worker> worker_;
  bool configured_;
  enum { WAIT, ACK, NACK } acknowledge_;
  unsigned int baudrate_;
  static boost::posix_time::time_duration default_timeout_;

  Callbacks callbacks_;
  boost::mutex callback_mutex_;
};

template <typename StreamT>
void Gps::initialize(StreamT& stream, boost::asio::io_service& io_service)
{
  if (worker_) return;
  initialize(boost::shared_ptr<Worker>(new AsyncWorker<StreamT>(stream, io_service)));
}

template <> void Gps::initialize(boost::asio::serial_port& serial_port, boost::asio::io_service& io_service);
extern template void Gps::initialize<boost::asio::ip::tcp::socket>(boost::asio::ip::tcp::socket& stream, boost::asio::io_service& io_service);
// extern template void Gps::initialize<boost::asio::ip::udp::socket>(boost::asio::ip::udp::socket& stream, boost::asio::io_service& io_service);

template <typename T>
Callbacks::iterator Gps::subscribe(typename CallbackHandler_<T>::Callback callback, unsigned int rate)
{
  if (!setRate(T::CLASS_ID, T::MESSAGE_ID, rate)) return Callbacks::iterator();
  return subscribe<T>(callback);
}

template <typename T>
Callbacks::iterator Gps::subscribe(typename CallbackHandler_<T>::Callback callback)
{
  boost::mutex::scoped_lock lock(callback_mutex_);
  CallbackHandler_<T> *handler = new CallbackHandler_<T>(callback);
  return callbacks_.insert(std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID), boost::shared_ptr<CallbackHandler>(handler)));
}

template <typename T>
void CallbackHandler_<T>::handle(ublox::Reader &reader) {
  boost::mutex::scoped_lock(mutex_);
  try {
    if (!reader.read<T>(message_)) {
      std::cout << "Decoder error for " << static_cast<unsigned int>(reader.classId()) << "/" << static_cast<unsigned int>(reader.messageId()) << " (" << reader.length() << " bytes)" << std::endl;
      return;
    }
  } catch(std::runtime_error& e) {
    std::cout << "Decoder error for " << static_cast<unsigned int>(reader.classId()) << "/" << static_cast<unsigned int>(reader.messageId()) << " (" << reader.length() << " bytes): " << std::string(e.what()) << std::endl;
    return;
  }

  if (func_) func_(message_);
  condition_.notify_all();
}

template <typename ConfigT> bool Gps::poll(ConfigT& message, const boost::posix_time::time_duration& timeout) {
  if (!poll(ConfigT::CLASS_ID, ConfigT::MESSAGE_ID)) return false;
  return read(message, timeout);
}

template <typename T> bool Gps::read(T& message, const boost::posix_time::time_duration& timeout) {
  bool result = false;
  if (!worker_) return false;

  callback_mutex_.lock();
  CallbackHandler_<T> *handler = new CallbackHandler_<T>();
  Callbacks::iterator callback = callbacks_.insert((std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID), boost::shared_ptr<CallbackHandler>(handler))));
  callback_mutex_.unlock();

  if (handler->wait(timeout)) {
    message = handler->get();
    result = true;
  }

  callback_mutex_.lock();
  callbacks_.erase(callback);
  callback_mutex_.unlock();
  return result;
}

template <typename ConfigT> bool Gps::configure(const ConfigT& message, bool wait) {
  if (!worker_) return false;

  acknowledge_ = WAIT;

  std::vector<unsigned char> out(1024);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(message)) return false;
  worker_->send(out.data(), writer.end() - out.data());

  if (!wait) return true;

  waitForAcknowledge(default_timeout_);
  return (acknowledge_ == ACK);
}

} // namespace ublox_gps

#endif // UBLOX_GPS_H
