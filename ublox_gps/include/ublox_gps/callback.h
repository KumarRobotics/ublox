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

#ifndef UBLOX_GPS_CALLBACK_H
#define UBLOX_GPS_CALLBACK_H

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <ublox/serialization/ublox_msgs.h>

namespace ublox_gps {

class CallbackHandler {
public:
  virtual void handle(ublox::Reader &reader) = 0;
  virtual bool wait(const boost::posix_time::time_duration& timeout);
  boost::mutex mutex_;
  boost::condition_variable condition_;
};

template <typename T>
class CallbackHandler_ : public CallbackHandler {
public:
  typedef boost::function<void(const T&)> Callback;
  CallbackHandler_(const Callback& func = Callback()) : func_(func) {}
  virtual void handle(ublox::Reader &reader);
  virtual const T& get() { return message_; }

private:
  Callback func_;
  T message_;
};

typedef std::multimap<std::pair<uint8_t,uint8_t>, boost::shared_ptr<CallbackHandler> > Callbacks;

} // namespace ublox_gps

#endif // UBLOX_GPS_CALLBACK_H
