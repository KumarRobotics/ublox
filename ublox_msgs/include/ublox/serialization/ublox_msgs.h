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

#ifndef UBLOX_SERIALIZATION_UBLOX_MSGS_H
#define UBLOX_SERIALIZATION_UBLOX_MSGS_H

#include <ublox/serialization.h>
#include <ublox_msgs/ublox_msgs.h>

namespace ublox {

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavDGPS_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::NavDGPS_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.iTOW);
    stream.next(m.age);
    stream.next(m.baseId);
    stream.next(m.baseHealth);
    stream.next(m.numCh);
    stream.next(m.status);
    stream.next(m.reserved1);
    m.sv.resize(m.numCh);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::NavDGPS_<ContainerAllocator> >::param_type m)
  {
    return 16 + 12 * m.numCh;
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::NavDGPS_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.age);
    stream.next(m.baseId);
    stream.next(m.baseHealth);
    stream.next(static_cast<typename ublox_msgs::NavDGPS_<ContainerAllocator>::_numCh_type>(m.sv.size()));
    stream.next(m.status);
    stream.next(m.reserved1);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSBAS_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::NavSBAS_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.iTOW);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(m.cnt);
    stream.next(m.reserved0);
    m.sv.resize(m.cnt);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::NavSBAS_<ContainerAllocator> >::param_type m)
  {
    return 12 + 12 * m.cnt;
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::NavSBAS_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(static_cast<typename ublox_msgs::NavSBAS_<ContainerAllocator>::_cnt_type>(m.sv.size()));
    stream.next(m.reserved0);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSVINFO_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::NavSVINFO_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.iTOW);
    stream.next(m.numCh);
    stream.next(m.globalFlags);
    stream.next(m.reserved2);
    m.sv.resize(m.numCh);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::NavSVINFO_<ContainerAllocator> >::param_type m)
  {
    return 8 + 12 * m.numCh;
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::NavSVINFO_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(static_cast<typename ublox_msgs::NavSVINFO_<ContainerAllocator>::_numCh_type>(m.sv.size()));
    stream.next(m.globalFlags);
    stream.next(m.reserved2);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmRAW_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::RxmRAW_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(m.numSV);
    stream.next(m.reserved1);
    m.sv.resize(m.numSV);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::RxmRAW_<ContainerAllocator> >::param_type m)
  {
    return 8 + 24 * m.numSV;
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::RxmRAW_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(static_cast<typename ublox_msgs::RxmRAW_<ContainerAllocator>::_numSV_type>(m.sv.size()));
    stream.next(m.reserved1);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmSVSI_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::RxmSVSI_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(m.numVis);
    stream.next(m.numSV);
    m.sv.resize(m.numSV);
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::RxmSVSI_<ContainerAllocator> >::param_type m)
  {
    return 8 + 6 * m.numSV;
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::RxmSVSI_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(m.numVis);
    stream.next(static_cast<typename ublox_msgs::RxmSVSI_<ContainerAllocator>::_numSV_type>(m.sv.size()));
    for(std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmALM_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::RxmALM_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    try {
      typename ublox_msgs::RxmALM_<ContainerAllocator>::_dwrd_type::value_type temp;

      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    } catch(ros::serialization::StreamOverrunException& e) {
    }
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::RxmALM_<ContainerAllocator> >::param_type m)
  {
    return 8 + (4 * m.dwrd.size());
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::RxmALM_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for(std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::serialize(stream, m.dwrd[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmEPH_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::RxmEPH_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    try {
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;

      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    } catch(ros::serialization::StreamOverrunException& e) {
    }
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::RxmEPH_<ContainerAllocator> >::param_type m)
  {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::RxmEPH_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for(std::size_t i = 0; i < m.sf1d.size(); ++i) ros::serialization::serialize(stream, m.sf1d[i]);
    for(std::size_t i = 0; i < m.sf2d.size(); ++i) ros::serialization::serialize(stream, m.sf2d[i]);
    for(std::size_t i = 0; i < m.sf3d.size(); ++i) ros::serialization::serialize(stream, m.sf3d[i]);
  }
};

/////////////////////////////////////////////////

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidALM_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::AidALM_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    try {
      typename ublox_msgs::AidALM_<ContainerAllocator>::_dwrd_type::value_type temp;

      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    } catch(ros::serialization::StreamOverrunException& e) {
    }
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::AidALM_<ContainerAllocator> >::param_type m)
  {
    return 8 + (4 * m.dwrd.size());
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::AidALM_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for(std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::serialize(stream, m.dwrd[i]);
  }
};

template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidEPH_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count, typename boost::call_traits<ublox_msgs::AidEPH_<ContainerAllocator> >::reference m)
  {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    try {
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;

      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      for(std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    } catch(ros::serialization::StreamOverrunException& e) {
    }
  }

  static uint32_t serializedLength (typename boost::call_traits<ublox_msgs::AidEPH_<ContainerAllocator> >::param_type m)
  {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t *data, uint32_t size, typename boost::call_traits<ublox_msgs::AidEPH_<ContainerAllocator> >::param_type m)
  {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for(std::size_t i = 0; i < m.sf1d.size(); ++i) ros::serialization::serialize(stream, m.sf1d[i]);
    for(std::size_t i = 0; i < m.sf2d.size(); ++i) ros::serialization::serialize(stream, m.sf2d[i]);
    for(std::size_t i = 0; i < m.sf3d.size(); ++i) ros::serialization::serialize(stream, m.sf3d[i]);
  }
};

} // namespace ublox

#endif // UBLOX_SERIALIZATION_UBLOX_MSGS_H
