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

#ifndef UBLOX_SERIALIZATION_SERIALIZATION_HPP
#define UBLOX_SERIALIZATION_SERIALIZATION_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "checksum.hpp"

///
/// This file defines the UbloxSerializer template class which encodes and decodes
/// specific message types.
/// The Reader class decodes messages and from a buffer and the Writer class
/// encodes messages and writes them to a buffer.
/// It also declares macros for declaring Messages. The Message class
/// maps ROS messages types to class and message ID(s).
///


/**
 * @namespace ublox
 * This namespace is for u-blox message serialization.
 */
namespace ublox {

//! u-blox message Sync A char
static const uint8_t DEFAULT_SYNC_A = 0xB5;
//! u-blox message Sync B char
static const uint8_t DEFAULT_SYNC_B = 0x62;
//! Number of bytes in a message header (Sync chars + class ID + message ID)
static const uint8_t kHeaderLength = 6;
//! Number of checksum bytes in the u-blox message
static const uint8_t kChecksumLength = 2;

/**
 * \brief Templated serialization class.  Default implementation provides backwards compatibility with
 * old message types.
 *
 * Specializing the UbloxSerializer class is the only thing you need to do to get the ROS serialization system
 * to work with a type.
 */
template<typename T, typename Enabled = void>
struct UbloxSerializer
{
  /**
   * \brief Write an object to the stream.  Normally the stream passed in here will be a UbloxOStream
   */
  template<typename Stream>
  inline static void write(Stream& stream, const T & t);

  /**
   * \brief Read an object from the stream.  Normally the stream passed in here will be a UbloxIStream
   */
  template<typename Stream>
  inline static void read(Stream& stream, T & t);

  /**
   * \brief Determine the serialized length of an object.
   */
  inline static uint32_t serializedLength(const T & t);
};

template <typename T>
struct UbloxSerializer<T, typename std::enable_if<std::is_same<T, uint8_t>::value ||
                                                  std::is_same<T, uint16_t>::value ||
                                                  std::is_same<T, uint32_t>::value ||
                                                  std::is_same<T, uint64_t>::value ||
                                                  std::is_same<T, int8_t>::value ||
                                                  std::is_same<T, int16_t>::value ||
                                                  std::is_same<T, int32_t>::value ||
                                                  std::is_same<T, int64_t>::value ||
                                                  std::is_same<T, float>::value ||
                                                  std::is_same<T, double>::value>::type>
{
  template<typename Stream>
  inline static void write(Stream& stream, const T v) {
    *reinterpret_cast<T*>(stream.advance(sizeof(v))) = v;
  }

  template<typename Stream>
  inline static void read(Stream& stream, T& v) {
    v = *reinterpret_cast<T*>(stream.advance(sizeof(v)));
  }

  inline static uint32_t serializedLength(const T& v) {
    (void)v;
    return sizeof(T);
  }
};

/**
 * \brief Serialize an object.  Stream here should normally be a UbloxOStream
 */
template<typename T, typename Stream>
inline void serialize(Stream& stream, const T& t) {
  UbloxSerializer<T>::write(stream, t);
}

/**
 * \brief Deserialize an object.  Stream here should normally be a UbloxIStream
 */
template<typename T, typename Stream>
inline void deserialize(Stream& stream, T& t) {
  UbloxSerializer<T>::read(stream, t);
}

/**
 * \brief Determine the serialized length of an object
 */
template<typename T>
inline uint32_t serializationLength(const T& t) {
  return UbloxSerializer<T>::serializedLength(t);
}

/**
 * \brief Array serializer, default implementation does nothing
 */
template<typename T, size_t N, class Enabled = void>
struct StdArrayUbloxSerializer
{};

/**
 * \brief Array serializer, specialized for fixed-size, simple types
 */
template<typename T, size_t N>
struct StdArrayUbloxSerializer<T, N, typename std::enable_if<std::is_same<T, uint8_t>::value ||
                                                             std::is_same<T, uint16_t>::value ||
                                                             std::is_same<T, uint32_t>::value ||
                                                             std::is_same<T, uint64_t>::value ||
                                                             std::is_same<T, int8_t>::value ||
                                                             std::is_same<T, int16_t>::value ||
                                                             std::is_same<T, int32_t>::value ||
                                                             std::is_same<T, int64_t>::value ||
                                                             std::is_same<T, float>::value ||
                                                             std::is_same<T, double>::value>::type>
{
  template<typename Stream>
  inline static void write(Stream& stream, const std::array<T, N>& v) {
    const uint32_t data_len = N * sizeof(T);
    std::memcpy(stream.advance(data_len), &v.front(), data_len);
  }

  template<typename Stream>
  inline static void read(Stream& stream, std::array<T, N>& v) {
    const uint32_t data_len = N * sizeof(T);
    std::memcpy(&v.front(), stream.advance(data_len), data_len);
  }

  inline static uint32_t serializedLength(const std::array<T, N>& v) {
    (void)v;
    return N * sizeof(T);
  }
};

/**
 * \brief serialize version for std::array
 */
template<typename T, size_t N, typename Stream>
inline void serialize(Stream& stream, const std::array<T, N>& t)
{
  StdArrayUbloxSerializer<T, N>::write(stream, t);
}

/**
 * \brief deserialize version for std::array
 */
template<typename T, size_t N, typename Stream>
inline void deserialize(Stream& stream, std::array<T, N>& t) {
  StdArrayUbloxSerializer<T, N>::read(stream, t);
}

/**
 * \brief serializationLength version for std::array
 */
template<typename T, size_t N>
inline uint32_t serializationLength(const std::array<T, N>& t)
{
  return StdArrayUbloxSerializer<T, N>::serializedLength(t);
}

/**
 * \brief UbloxStream base-class, provides common functionality for UbloxIStream and UbloxOStream
 */
struct UbloxStream
{
  /*
   * \brief Returns a pointer to the current position of the stream
   */
  inline uint8_t* getData() {
    return data_;
  }
  /**
   * \brief Advances the stream, checking bounds, and returns a pointer to the position before it
   * was advanced.
   * \throws StreamOverrunException if len would take this stream past the end of its buffer
   */
  uint8_t* advance(uint32_t len) {
    uint8_t* old_data = data_;
    data_ += len;
    if (data_ > end_) {
      // Throwing directly here causes a significant speed hit due to the extra code generated
      // for the throw statement
      //throwStreamOverrun();
    }
    return old_data;
  }

  /**
   * \brief Returns the amount of space left in the stream
   */
  inline uint32_t getLength() { return static_cast<uint32_t>(end_ - data_); }

protected:
  UbloxStream(uint8_t* _data, uint32_t _count)
  : data_(_data)
  , end_(_data + _count)
  {}

private:
  uint8_t* data_;
  uint8_t* end_;
};

/**
 * \brief Input stream
 */
struct UbloxIStream : public UbloxStream
{
  UbloxIStream(uint8_t* data, uint32_t count)
  : UbloxStream(data, count)
  {}

  /**
   * \brief Deserialize an item from this input stream
   */
  template<typename T>
  void next(T& t) {
    deserialize(*this, t);
  }
};

/**
 * \brief Output stream
 */
struct UbloxOStream : public UbloxStream
{
  UbloxOStream(uint8_t* data, uint32_t count)
  : UbloxStream(data, count)
  {}

  /**
   * \brief Serialize an item to this output stream
   */
  template<typename T>
  void next(const T& t) {
    serialize(*this, t);
  }
};

/**
 * @brief Keeps track of which class and message IDs can be decoded by a given
 * message type.
 */
template <typename T>
class Message {
 public:
  /**
   * @brief Can this message type decode a u-blox message with the given ID?
   * @param class_id the class ID of the u-blox message
   * @param message_id the message ID of the u-blox message
   * @return whether or not this message type decode the u-blox message
   */
  static bool canDecode(uint8_t class_id, uint8_t message_id) {
    return std::find(keys_.begin(), keys_.end(),
                     std::make_pair(class_id, message_id)) != keys_.end();
  }

  /**
   * @brief Indicate that this message type can decode u-blox messages with the
   * given ID
   * @param class_id the class ID of the u-blox message
   * @param message_id the message ID of the u-blox message
   */
  static void addKey(uint8_t class_id, uint8_t message_id) {
    keys_.emplace_back(std::make_pair(class_id, message_id));
  }

  struct StaticKeyInitializer
  {
    StaticKeyInitializer(uint8_t class_id, uint8_t message_id) {
      Message<T>::addKey(class_id, message_id);
    }
  };

 private:
  static std::vector<std::pair<uint8_t,uint8_t> > keys_;
};

/**
 * @brief Options for the Reader and Writer for encoding and decoding messages.
 */
struct Options {
  /**
   * The default options for a u-blox message.
   */
  Options() : sync_a(DEFAULT_SYNC_A), sync_b(DEFAULT_SYNC_B),
              header_length(kHeaderLength), checksum_length(kChecksumLength) {}
  //! The sync_a byte value identifying the start of a message
  uint8_t sync_a;
  //! The sync_b byte value identifying the start of a message
  uint8_t sync_b;
  //! The length of the message header in bytes (everything before the payload)
  uint8_t header_length;
  //! The length of the checksum in bytes
  uint8_t checksum_length;

  /**
   * @brief Get the number of bytes in the header and footer.
   * @return the number of bytes in the header and footer
   */
  uint32_t wrapper_length() {
    return header_length + checksum_length;
  }
};

/**
 * @brief Decodes byte messages into u-blox ROS messages.
 */
class Reader {
 public:
  /**
   * @param data a buffer containing u-blox messages
   * @param count the size of the buffer
   * @param options A struct containing the parameters sync_a and sync_b
   * which represent the sync bytes indicating the beginning of the message
   */
  Reader(const uint8_t *data, uint32_t count,
         const Options &options = Options()) :
      data_(data), count_(count), found_(false), options_(options)
  {
    extra_data_.reserve(1024);
  }

  using iterator = const uint8_t *;

  /**
   * @brief Search the buffer for the beginning of the next u-blox message
   * @return a pointer to the start of the next u-blox message
   */
  iterator search()
  {
    if (found_) {
      next();
    }

    // Search for a message header
    for( ; count_ > 0; --count_, ++data_) {
      if (data_[0] == options_.sync_a &&
          (count_ == 1 || data_[1] == options_.sync_b)) {
        break;
      } else {
        extra_data_.push_back(data_[0]);
      }
    }

    return data_;
  }

  /**
   * @brief Has a u-blox message been found in the buffer?
   * @returns true if A message with the correct header & length has been found
   */
  bool found()
  {
    if (found_) {
      return true;
    }
    // Verify message is long enough to have sync chars, id, length & checksum
    if (count_ < options_.wrapper_length()) {
      return false;
    }
    // Verify the header bits
    if (data_[0] != options_.sync_a || data_[1] != options_.sync_b) {
      return false;
    }
    // Verify that the buffer length is long enough based on the received
    // message length
    if (count_ < length() + options_.wrapper_length()) {
      return false;
    }

    found_ = true;
    return true;
  }

  /**
   * @brief Go to the start of the next message based on the received message
   * length.
   *
   * @details Warning: Does not go to the correct byte location if the received
   * message length is incorrect. If this is the case, search must be called.
   */
  iterator next() {
    if (found()) {
      uint32_t size = length() + options_.wrapper_length();
      data_ += size; count_ -= size;
    }
    found_ = false;
    return data_;
  }

  /**
   * @brief Get the current position in the read buffer.
   * @return the current position of the read buffer
   */
  iterator pos() {
    return data_;
  }

  iterator end() {
    return data_ + count_;
  }

  uint8_t classId() { return data_[2]; }
  uint8_t messageId() { return data_[3]; }

  /**
   * @brief Get the length of the u-blox message payload.
   *
   * @details Payload length does not include the header or checksum length.
   * Determines the length from the header of the u-blox message.
   * @return the length of the message payload
   */
  uint32_t length() { return (data_[5] << 8) + data_[4]; }
  const uint8_t *data() { return data_ + options_.header_length; }

  /**
   * @brief Get the checksum of the u-blox message.
   *
   * @return the checksum of the u-blox message
   */
  uint16_t checksum() {
    return *reinterpret_cast<const uint16_t *>(data_ + options_.header_length +
                                               length());
  }

  /**
   * @brief Decode the given message.
   * @param message the output message
   * @param search whether or not to skip to the next message in the buffer
   */
  template <typename T>
  bool read(T &message,
            bool search = false) {
    if (search) {
      this->search();
    }
    if (!found()) {
      return false;
    }
    if (!Message<T>::canDecode(classId(), messageId())) {
      return false;
    }

    uint16_t chk{0};
    if (calculateChecksum(data_ + 2, length() + 4, chk) != this->checksum()) {
      // checksum error
      // Note that it is possible (and even likely) that we get here without
      // having the entire packet available.  This happens when there are both
      // NMEA and UBlox messages configured on the serial wire, and the packet is
      // laid out like:
      //
      // <NMEA_message><UBlox_message_missing_two_bytes>
      //
      // Therefore, we do not print errors in this case and instead just don't
      // do any additional work.
      return false;
    }

    UbloxSerializer<T>::read(data_ + options_.header_length, length(), message);
    return true;
  }

  /**
   * @brief Can the given message type decode the current message in the buffer?
   * @return whether the given message type can decode the current message in
   * the buffer
   */
  template <typename T>
  bool hasType() {
    if (!found()) {
      return false;
    }
    return Message<T>::canDecode(classId(), messageId());
  }

  /**
   * @brief Does the u-blox message have the given class and message ID?
   * @return Whether or not the u-blox message has the given class and message
   * ID
   */
  bool isMessage(uint8_t class_id, uint8_t message_id) {
    if (!found()) {
      return false;
    }
    return (classId() == class_id && messageId() == message_id);
  }

  const std::string &getExtraData() const {
    return extra_data_;
  }

private:
  //! The buffer of message bytes
  const uint8_t *data_;
  //! Unused data from the read buffer, contains nmea messages.
  std::string extra_data_;
  //! the number of bytes in the buffer, //! decrement as the buffer is read
  uint32_t count_;
  //! Whether or not a message has been found
  bool found_;
  //! Options representing the sync char values, etc.
  Options options_;
};

/**
 * @brief Encodes a u-blox ROS message as a byte array.
 */
class Writer {
 public:
  using iterator = uint8_t *;

  /**
   * @brief Construct a Writer with the given buffer.
   * @param data a buffer for messages
   * @param size the size of the buffer
   * @param options options representing the message sync chars, etc.
   */
  Writer(uint8_t *data, uint32_t size, const Options &options = Options()) :
      data_(data), size_(size), options_(options) {}

  /**
   * @brief Encode the u-blox message.
   * @param message the message to encode
   * @param class_id the u-blox class ID, defaults to the message CLASS_ID
   * @param message_id the u-blox message ID, defaults to the message MESSAGE_ID
   * @return true if the message was encoded correctly, false otherwise
   */
  template <typename T> bool write(const T& message,
                                   uint8_t class_id = T::CLASS_ID,
                                   uint8_t message_id = T::MESSAGE_ID) {
    // Check for buffer overflow
    uint32_t length = UbloxSerializer<T>::serializedLength(message);
    if (size_ < length + options_.wrapper_length()) {
      // ROS_ERROR("u-blox write buffer overflow. Message %u / %u not written",
      //           class_id, message_id);
      return false;
    }
    // Encode the message and add it to the buffer
    UbloxSerializer<T>::write(data_ + options_.header_length,
                         size_ - options_.header_length, message);
    return write(nullptr, length, class_id, message_id);
  }

  /**
   * @brief Wrap the encoded message payload with a header and checksum and
   * add it to the buffer.
   * @param message the encoded message payload (no header or checksum)
   * @param length the length of the message payload
   * @param class_id the u-blox class ID
   * @param message_id the u-blox message ID
   * @return true if the message was encoded correctly, false otherwise
   */
  bool write(const uint8_t* message, uint32_t length, uint8_t class_id,
             uint8_t message_id) {
    if (size_ < length + options_.wrapper_length()) {
      // ROS_ERROR("u-blox write buffer overflow. Message %u / %u not written",
      //           class_id, message_id);
      return false;
    }
    iterator start = data_;

    // write header
    *data_++ = options_.sync_a;
    *data_++ = options_.sync_b;
    *data_++ = class_id;
    *data_++ = message_id;
    *data_++ = length & 0xFF;
    *data_++ = (length >> 8) & 0xFF;
    size_ -= options_.header_length;

    // write message
    if (message) {
      std::copy(message, message + length, data_);
    }
    data_ += length;
    size_ -= length;

    // write checksum
    uint8_t ck_a, ck_b;
    calculateChecksum(start + 2, length + 4, ck_a, ck_b);
    *data_++ = ck_a;
    *data_++ = ck_b;
    size_ -= options_.checksum_length;

    return true;
  }

  iterator end() {
    return data_;
  }

 private:
  //! The buffer of message bytes
  iterator data_;
  //! The number of remaining bytes in the buffer
  /*! Decrements as messages are written to the buffer */
  uint32_t size_;
  //! Options representing the sync char values, etc.
  Options options_;
};

}  // namespace ublox

// Use to declare u-blox messages and message serializers
#define DECLARE_UBLOX_MESSAGE(class_id, message_id, package, message) \
  template class ublox::UbloxSerializer<package::msg::message>; \
  template class ublox::Message<package::msg::message>; \
  namespace package { namespace { \
    static const ublox::Message<package::msg::message>::StaticKeyInitializer static_key_initializer_##message(class_id, message_id); \
  } } \

// Use for messages which have the same structure but different IDs, e.g. INF
// Call DECLARE_UBLOX_MESSAGE for the first message and DECLARE_UBLOX_MESSAGE_ID
// for following declarations
#define DECLARE_UBLOX_MESSAGE_ID(class_id, message_id, package, message, name) \
  namespace package { namespace { \
    static const ublox::Message<package::msg::message>::StaticKeyInitializer static_key_initializer_##name(class_id, message_id); \
  } } \

#endif  // UBLOX_SERIALIZATION_SERIALIZATION_HPP
