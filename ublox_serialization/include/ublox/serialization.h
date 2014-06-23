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

#ifndef UBLOX_SERIALIZATION_H
#define UBLOX_SERIALIZATION_H

#include <stdint.h>
#include <boost/call_traits.hpp>
#include <vector>
#include <algorithm>

#include "checksum.h"

namespace ublox {

static const uint8_t DEFAULT_SYNC_A = 0xB5;
static const uint8_t DEFAULT_SYNC_B = 0x62;

template <typename T>
struct Serializer {
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<T>::reference message);
  static uint32_t serializedLength(typename boost::call_traits<T>::param_type message);
  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<T>::param_type message);
};

template <typename T>
class Message {
public:
  static bool canDecode(uint8_t class_id, uint8_t message_id) {
    return std::find(keys_.begin(), keys_.end(), std::make_pair(class_id, message_id)) != keys_.end();
  }

  static void addKey(uint8_t class_id, uint8_t message_id) {
    keys_.push_back(std::make_pair(class_id, message_id));
  }

  struct StaticKeyInitializer
  {
    StaticKeyInitializer(uint8_t class_id, uint8_t message_id) { Message<T>::addKey(class_id, message_id); }
  };

private:
  static std::vector<std::pair<uint8_t,uint8_t> > keys_;
};

struct Options
{
  Options() : sync_a(DEFAULT_SYNC_A), sync_b(DEFAULT_SYNC_B) {}
  uint8_t sync_a, sync_b;
};

class Reader {
public:
  Reader(const uint8_t *data, uint32_t count, const Options &options = Options()) : data_(data), count_(count), found_(false), options_(options) {}

  typedef const uint8_t *iterator;

  iterator search()
  {
    if (found_) next();

    for( ; count_ > 0; --count_, ++data_) {
      if (data_[0] == options_.sync_a && (count_ == 1 || data_[1] == options_.sync_b)) break;
    }

    return data_;
  }

  bool found()
  {
    if (found_) return true;
    if (count_ < 6) return false;
    if (data_[0] != options_.sync_a || data_[1] != options_.sync_b) return false;
    if (count_ < length() + 8) return false;

    found_ = true;
    return true;
  }

  iterator next() {
    if (found()) {
      uint32_t size = length() + 8;
      data_ += size; count_ -= size;
    }
    found_ = false;
    return data_;
  }

  iterator pos() {
    return data_;
  }

  iterator end() {
    return data_ + count_;
  }

  uint8_t classId() { return data_[2]; }
  uint8_t messageId() { return data_[3]; }
  uint32_t length() { return (data_[5] << 8) + data_[4]; }
  const uint8_t *data() { return data_ + 6; }
  uint16_t checksum() { return *reinterpret_cast<const uint16_t *>(data_ + 6 + length()); }

  template <typename T>
  bool read(typename boost::call_traits<T>::reference message, bool search = false)
  {
    if (search) this->search();
    if (!found()) return false;
    if (!Message<T>::canDecode(classId(), messageId())) return false;

    uint16_t chk;
    if (calculateChecksum(data_ + 2, length() + 4, chk) != this->checksum()) {
      // checksum error
      return false;
    }

    Serializer<T>::read(data_ + 6, length(), message);
    return true;
  }

  template <typename T> bool hasType()
  {
    if (!found()) return false;
    return Message<T>::canDecode(classId(), messageId());
  }

  bool isMessage(uint8_t class_id, uint8_t message_id)
  {
    if (!found()) return false;
    return (data_[2] == class_id && data_[3] == message_id);
  }

private:
  const uint8_t *data_;
  uint32_t count_;
  bool found_;
  Options options_;
};

class Writer
{
public:
  Writer(uint8_t *data, uint32_t size, const Options &options = Options()) : data_(data), size_(size), options_(options) {}

  typedef uint8_t *iterator;

  template <typename T> bool write(const T& message, uint8_t class_id = T::CLASS_ID, uint8_t message_id = T::MESSAGE_ID) {
    uint32_t length = Serializer<T>::serializedLength(message);
    if (size_ < length + 8) return false;
    Serializer<T>::write(data_ + 6, size_ - 6, message);
    return write(0, length, class_id, message_id);
  }

  bool write(const uint8_t* message, uint32_t length, uint8_t class_id, uint8_t message_id) {
    if (size_ < length + 8) return false;
    uint8_t *start = data_;

    // write header
    *data_++ = options_.sync_a;
    *data_++ = options_.sync_b;
    *data_++ = class_id;
    *data_++ = message_id;
    *data_++ = length & 0xFF;
    *data_++ = (length >> 8) & 0xFF;
    size_ -= 6;

    // write message
    if (message) std::copy(message, message + length, data_);
    data_ += length;
    size_ -= length;

    // write checksum
    uint8_t ck_a, ck_b;
    calculateChecksum(start + 2, length + 4, ck_a, ck_b);
    *data_++ = ck_a;
    *data_++ = ck_b;
    size_ -= 2;

    return true;
  }

  iterator end() {
    return data_;
  }

private:
  uint8_t *data_;
  uint32_t size_;
  Options options_;
};

} // namespace ublox

#define DECLARE_UBLOX_MESSAGE(class_id, message_id, package, message) \
  template class ublox::Serializer<package::message>; \
  template class ublox::Message<package::message>; \
  namespace package { namespace { \
    static const ublox::Message<message>::StaticKeyInitializer static_key_initializer_##message(class_id, message_id); \
  } } \

// use implementation of class Serializer in "serialization_ros.h"
#include "serialization_ros.h"

#endif // UBLOX_SERIALIZATION_H
