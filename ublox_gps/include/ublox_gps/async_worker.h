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

#ifndef UBLOX_GPS_ASYNC_WORKER_H
#define UBLOX_GPS_ASYNC_WORKER_H

#include <ublox_gps/gps.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "worker.h"

namespace ublox_gps {

static const int debug = 1;

template <typename StreamT>
class AsyncWorker : public Worker
{
public:
  AsyncWorker(StreamT& stream, boost::asio::io_service& io_service, std::size_t buffer_size = 8192);
  virtual ~AsyncWorker();

  void setCallback(const Callback& callback) { read_callback_ = callback; }

  bool send(const unsigned char *data, const unsigned int size);
  void wait(const boost::posix_time::time_duration& timeout);

  bool isOpen() const { return stream_.is_open(); }
  
protected:
  void doRead();
  void readEnd(const boost::system::error_code&, std::size_t);
  void doWrite();
  void doClose();

  StreamT& stream_;
  boost::asio::io_service& io_service_;

  boost::mutex read_mutex_;
  boost::condition read_condition_;
  std::vector<unsigned char> in_;
  std::size_t in_buffer_size_;

  boost::mutex write_mutex_;
  boost::condition write_condition_;
  std::vector<unsigned char> out_;

  boost::shared_ptr<boost::thread> background_thread_;
  Callback read_callback_;

  bool stopping_;
};

template <typename StreamT>
AsyncWorker<StreamT>::AsyncWorker(StreamT& stream, boost::asio::io_service& io_service, std::size_t buffer_size)
  : stream_(stream)
  , io_service_(io_service)
  , stopping_(false)
{
  in_.resize(buffer_size);
  in_buffer_size_ = 0;

  out_.reserve(buffer_size);

  io_service_.post(boost::bind(&AsyncWorker<StreamT>::doRead, this));
  background_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_)));
}

template <typename StreamT>
AsyncWorker<StreamT>::~AsyncWorker()
{
  io_service_.post(boost::bind(&AsyncWorker<StreamT>::doClose, this));
  background_thread_->join();
  io_service_.reset();
}

template <typename StreamT>
bool AsyncWorker<StreamT>::send(const unsigned char *data, const unsigned int size) {
  boost::mutex::scoped_lock lock(write_mutex_);

  if (out_.capacity() - out_.size() < size) return false;
  out_.insert(out_.end(), data, data + size);

  io_service_.post(boost::bind(&AsyncWorker<StreamT>::doWrite, this));
  return true;
}

template <typename StreamT>
void AsyncWorker<StreamT>::doRead()
{
  // read_mutex_.lock();
  stream_.async_read_some(boost::asio::buffer(in_.data() + in_buffer_size_, in_.size() - in_buffer_size_), boost::bind(&AsyncWorker<StreamT>::readEnd, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

template <typename StreamT>
void AsyncWorker<StreamT>::readEnd(const boost::system::error_code& error, std::size_t bytes_transfered)
{
  if (error) {
    // do something

  } else if (bytes_transfered > 0) {
    in_buffer_size_ += bytes_transfered;

    if (debug >= 4) {
      std::cout << "received " << bytes_transfered << " bytes" << std::endl;
      for(std::vector<unsigned char>::iterator it = in_.begin() + in_buffer_size_ - bytes_transfered; it != in_.begin() + in_buffer_size_; ++it) std::cout << std::hex << static_cast<unsigned int>(*it) << " ";
      std::cout << std::dec << std::endl;
    }

    if (read_callback_) read_callback_(in_.data(), in_buffer_size_);

    read_condition_.notify_all();
  }

  // read_mutex_.unlock();
  if (!stopping_) io_service_.post(boost::bind(&AsyncWorker<StreamT>::doRead, this));
}

template <typename StreamT>
void AsyncWorker<StreamT>::doWrite()
{
  boost::mutex::scoped_lock lock(write_mutex_);
  if (out_.size() == 0) return;

  boost::asio::write(stream_, boost::asio::buffer(out_.data(), out_.size()));

  if (debug >= 2) {
    std::cout << "sent " << out_.size() << " bytes:" << std::endl;
    for(std::vector<unsigned char>::iterator it = out_.begin(); it != out_.end(); ++it) std::cout << std::hex << static_cast<unsigned int>(*it) << " ";
    std::cout << std::dec << std::endl;
  }
  out_.clear();
  write_condition_.notify_all();
}

template <typename StreamT>
void AsyncWorker<StreamT>::doClose()
{
  stopping_ = true;
  boost::system::error_code error;
  stream_.cancel(error);
}

template <typename StreamT>
void AsyncWorker<StreamT>::wait(const boost::posix_time::time_duration &timeout)
{
  boost::mutex::scoped_lock lock(read_mutex_);
  read_condition_.timed_wait(lock, timeout);
}

} // namespace ublox_gps

#endif // UBLOX_GPS_ASYNC_WORKER_H
