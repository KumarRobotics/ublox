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
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_CALLBACK_H
#define UBLOX_GPS_CALLBACK_H

#include <ros/console.h>
#include <ublox/serialization/ublox_msgs.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace ublox_gps {

class CallbackHandler {
 public:
  virtual void handle(ublox::Reader& reader) = 0;
  bool wait(const boost::posix_time::time_duration& timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    return condition_.timed_wait(lock, timeout);
  }
  boost::mutex mutex_;
  boost::condition_variable condition_;
};

template <typename T>
class CallbackHandler_ : public CallbackHandler {
 public:
  typedef boost::function<void(const T&)> Callback;
  CallbackHandler_(const Callback& func = Callback()) : func_(func) {}
  virtual const T& get() { return message_; }

  /**
   * @brief Decode the U-Blox message & call the callback function if it exists.
   */
  void handle(ublox::Reader& reader) {
    boost::mutex::scoped_lock(mutex_);
    try {
      if (!reader.read<T>(message_)) {
        ROS_DEBUG_COND(debug >= 2, 
                       "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)", 
                       static_cast<unsigned int>(reader.classId()),
                       static_cast<unsigned int>(reader.messageId()),
                       reader.length());
        condition_.notify_all();
        return;
      }
    } catch (std::runtime_error& e) {
      ROS_DEBUG_COND(debug >= 2, 
                     "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)", 
                     static_cast<unsigned int>(reader.classId()),
                     static_cast<unsigned int>(reader.messageId()),
                     reader.length());
      condition_.notify_all();
      return;
    }

    if (func_) func_(message_);
    condition_.notify_all();
  }
  
 private:
  Callback func_;
  T message_;
};

class CallbackHandlers {
  typedef std::multimap<std::pair<uint8_t, uint8_t>,
                        boost::shared_ptr<CallbackHandler> > Callbacks;
 public:
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                     boost::shared_ptr<CallbackHandler>(handler)));
  }

  template <typename T>
  void insert(
      typename CallbackHandler_<T>::Callback callback, 
      unsigned int message_id) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, message_id),
                     boost::shared_ptr<CallbackHandler>(handler)));
  }

  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    boost::mutex::scoped_lock lock(callback_mutex_);
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback)
      callback->second->handle(reader);
  }

  template <typename T>
  bool read(T& message, const boost::posix_time::time_duration& timeout) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    CallbackHandler_<T>* handler = new CallbackHandler_<T>();
    Callbacks::iterator callback = callbacks_.insert(
      (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                      boost::shared_ptr<CallbackHandler>(handler))));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout)) {
      message = handler->get();
      result = true;
    }
    
    // Remove the callback handler
    callback_mutex_.lock();
    callbacks_.erase(callback);
    callback_mutex_.unlock();
    return result;
  }

 private:
  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  boost::mutex callback_mutex_;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_CALLBACK_H
