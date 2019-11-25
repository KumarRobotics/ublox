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


#ifndef UBLOX_SERIALIZATION_UBLOX_MSGS_HPP
#define UBLOX_SERIALIZATION_UBLOX_MSGS_HPP

#include <ros/console.h>
#include <ublox/serialization.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

///
/// This file declares custom serializers for u-blox messages with dynamic
/// lengths and messages where the get/set messages have different sizes, but
/// share the same parameters, such as CfgDAT.
///

namespace ublox {

///
/// @brief Serializes the CfgDAT message which has a different length for
/// get/set.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgDAT_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::CfgDAT_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.datum_num);
    stream.next(m.datum_name);
    stream.next(m.maj_a);
    stream.next(m.flat);
    stream.next(m.d_x);
    stream.next(m.d_y);
    stream.next(m.d_z);
    stream.next(m.rot_x);
    stream.next(m.rot_y);
    stream.next(m.rot_z);
    stream.next(m.scale);
  }

  static uint32_t serializedLength(const ublox_msgs::CfgDAT_<ContainerAllocator> &m) {
    // this is the size of CfgDAT set messages
    // serializedLength is only used for writes so this is ok
    return 44;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::CfgDAT_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    // ignores datumNum & datumName
    stream.next(m.maj_a);
    stream.next(m.flat);
    stream.next(m.d_x);
    stream.next(m.d_y);
    stream.next(m.d_z);
    stream.next(m.rot_x);
    stream.next(m.rot_y);
    stream.next(m.rot_z);
    stream.next(m.scale);
  }
};

///
/// @brief Serializes the CfgGNSS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgGNSS_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::CfgGNSS_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.msg_ver);
    stream.next(m.num_trk_ch_hw);
    stream.next(m.num_trk_ch_use);
    stream.next(m.num_config_blocks);
    m.blocks.resize(m.num_config_blocks);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      ros::serialization::deserialize(stream, m.blocks[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::CfgGNSS_<ContainerAllocator> &m) {
    return 4 + 8 * m.num_config_blocks;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::CfgGNSS_<ContainerAllocator> &m) {
    if (m.blocks.size() != m.num_config_blocks) {
      ROS_ERROR("CfgGNSS num_config_blocks must equal blocks size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.msg_ver);
    stream.next(m.num_trk_ch_hw);
    stream.next(m.num_trk_ch_use);
    stream.next(
        static_cast<typename ublox_msgs::CfgGNSS_<ContainerAllocator>::_num_config_blocks_type>(m.blocks.size()));
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      ros::serialization::serialize(stream, m.blocks[i]);
    }
  }
};

///
/// @brief Serializes the CfgInf message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgINF_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::CfgINF_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    int num_blocks = count / 10;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) {
      ros::serialization::deserialize(stream, m.blocks[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::CfgINF_<ContainerAllocator> &m) {
    return 10 * m.blocks.size();
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::CfgINF_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      ros::serialization::serialize(stream, m.blocks[i]);
    }
  }
};

///
/// @brief Serializes the Inf message which has a dynamic length string.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::Inf_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::Inf_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    m.str.resize(count);
    for (int i = 0; i < count; ++i) {
      ros::serialization::deserialize(stream, m.str[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::Inf_<ContainerAllocator> &m) {
    return m.str.size();
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::Inf_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    for (std::size_t i = 0; i < m.str.size(); ++i) {
      ros::serialization::serialize(stream, m.str[i]);
    }
  }
};

///
/// @brief Serializes the MonVER message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::MonVER_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::MonVER_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.sw_version);
    stream.next(m.hw_version);

    m.extension.clear();
    int N = (count - 40) / 30;
    m.extension.reserve(N);
    typename ublox_msgs::MonVER_<ContainerAllocator>::_extension_type::value_type ext;
    for (int i = 0; i < N; i++) {
      // Read each extension string
      stream.next(ext);
      m.extension.push_back(ext);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::MonVER_<ContainerAllocator> &m) {
    return 40 + (30 * m.extension.size());
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::MonVER_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.sw_version);
    stream.next(m.hw_version);
    for (std::size_t i = 0; i < m.extension.size(); ++i) {
      ros::serialization::serialize(stream, m.extension[i]);
    }
  }
};

///
/// @brief Serializes the NavDGPS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavDGPS_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::NavDGPS_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.age);
    stream.next(m.base_id);
    stream.next(m.base_health);
    stream.next(m.num_ch);
    stream.next(m.status);
    stream.next(m.reserved1);
    m.sv.resize(m.num_ch);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::NavDGPS_<ContainerAllocator> &m) {
    return 16 + 12 * m.num_ch;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::NavDGPS_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_ch) {
      ROS_ERROR("NavDGPS numCh must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.age);
    stream.next(m.base_id);
    stream.next(m.base_health);
    stream.next(static_cast<typename ublox_msgs::NavDGPS_<ContainerAllocator>::_num_ch_type>(m.sv.size()));
    stream.next(m.status);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};


///
/// @brief Serializes the NavSBAS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSBAS_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::NavSBAS_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(m.cnt);
    stream.next(m.reserved0);
    m.sv.resize(m.cnt);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::NavSBAS_<ContainerAllocator> &m) {
    return 12 + 12 * m.cnt;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::NavSBAS_<ContainerAllocator> &m) {
    if (m.sv.size() != m.cnt) {
      ROS_ERROR("NavSBAS cnt must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(static_cast<typename ublox_msgs::NavSBAS_<ContainerAllocator>::_cnt_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};

///
/// @brief Serializes the NavSAT message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSAT_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::NavSAT_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.num_svs);
    stream.next(m.reserved0);
    m.sv.resize(m.num_svs);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::NavSAT_<ContainerAllocator> &m) {
    return 8 + 12 * m.num_svs;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::NavSAT_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_svs) {
      ROS_ERROR("NavSAT num_svs must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(static_cast<typename ublox_msgs::NavSAT_<ContainerAllocator>::_num_svs_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};

///
/// @brief Serializes the NavDGPS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSVINFO_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::NavSVINFO_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.num_ch);
    stream.next(m.global_flags);
    stream.next(m.reserved2);
    m.sv.resize(m.num_ch);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::NavSVINFO_<ContainerAllocator> &m) {
    return 8 + 12 * m.num_ch;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::NavSVINFO_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_ch) {
      ROS_ERROR("NavSVINFO num_ch must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(static_cast<typename ublox_msgs::NavSVINFO_<ContainerAllocator>::_num_ch_type>(m.sv.size()));
    stream.next(m.global_flags);
    stream.next(m.reserved2);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};

///
/// @brief Serializes the RxmRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmRAW_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmRAW_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.num_sv);
    stream.next(m.reserved1);
    m.sv.resize(m.num_sv);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmRAW_<ContainerAllocator> &m) {
    return 8 + 24 * m.num_sv;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmRAW_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_sv) {
      ROS_ERROR("RxmRAW num_sv must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(static_cast<typename ublox_msgs::RxmRAW_<ContainerAllocator>::_num_sv_type>(m.sv.size()));
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};

///
/// @brief Serializes the RxmRAWX message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmRAWX_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmRAWX_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(m.num_meas);
    stream.next(m.rec_stat);
    stream.next(m.version);
    stream.next(m.reserved1);
    m.meas.resize(m.num_meas);
    for (std::size_t i = 0; i < m.meas.size(); ++i) {
      ros::serialization::deserialize(stream, m.meas[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmRAWX_<ContainerAllocator> &m) {
    return 16 + 32 * m.num_meas;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmRAWX_<ContainerAllocator> &m) {
    if (m.meas.size() != m.num_meas) {
      ROS_ERROR("RxmRAWX num_meas must equal meas size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(static_cast<typename ublox_msgs::RxmRAWX_<ContainerAllocator>::_num_meas_type>(m.meas.size()));
    stream.next(m.rec_stat);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.meas.size(); ++i) {
      ros::serialization::serialize(stream, m.meas[i]);
    }
  }
};

///
/// @brief Serializes the RxmSFRBX message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmSFRBX_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmSFRBX_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.reserved0);
    stream.next(m.freq_id);
    stream.next(m.num_words);
    stream.next(m.chn);
    stream.next(m.version);
    stream.next(m.reserved1);
    m.dwrd.resize(m.num_words);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      ros::serialization::deserialize(stream, m.dwrd[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmSFRBX_<ContainerAllocator> &m) {
    return 8 + 4 * m.num_words;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmSFRBX_<ContainerAllocator> &m) {
    if (m.dwrd.size() != m.num_words) {
      ROS_ERROR("RxmSFRBX num_words must equal dwrd size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.reserved0);
    stream.next(m.freq_id);
    stream.next(static_cast<typename ublox_msgs::RxmSFRBX_<ContainerAllocator>::_num_words_type>(m.dwrd.size()));
    stream.next(m.chn);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      ros::serialization::serialize(stream, m.dwrd[i]);
    }
  }
};

///
/// @brief Serializes the RxmSVSI message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmSVSI_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmSVSI_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.week);
    stream.next(m.num_vis);
    stream.next(m.num_sv);
    m.sv.resize(m.num_sv);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::deserialize(stream, m.sv[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmSVSI_<ContainerAllocator> &m) {
    return 8 + 6 * m.num_sv;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmSVSI_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_sv) {
      ROS_ERROR("RxmSVSI num_sv must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.week);
    stream.next(m.num_vis);
    stream.next(static_cast<typename ublox_msgs::RxmSVSI_<ContainerAllocator>::_num_sv_type>(m.sv.size()));
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      ros::serialization::serialize(stream, m.sv[i]);
    }
  }
};

///
/// @brief Serializes the RxmALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmALM_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmALM_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename ublox_msgs::RxmALM_<ContainerAllocator>::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmALM_<ContainerAllocator> &m) {
    return 8 + (4 * m.dwrd.size());
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmALM_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      ros::serialization::serialize(stream, m.dwrd[i]);
    }
  }
};

///
/// @brief Serializes the RxmEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmEPH_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::RxmEPH_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::RxmEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;

      m.sf1d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      m.sf2d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      m.sf3d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    }
  }

  static uint32_t serializedLength(const ublox_msgs::RxmEPH_<ContainerAllocator> &m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::RxmEPH_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for(std::size_t i = 0; i < m.sf1d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf1d[i]);
    }
    for(std::size_t i = 0; i < m.sf2d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf2d[i]);
    }
    for(std::size_t i = 0; i < m.sf3d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf3d[i]);
    }
  }
};

///
/// @brief Serializes the AidALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidALM_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::AidALM_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename ublox_msgs::AidALM_<ContainerAllocator>::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  static uint32_t serializedLength(const ublox_msgs::AidALM_<ContainerAllocator> &m) {
    return 8 + (4 * m.dwrd.size());
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::AidALM_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      ros::serialization::serialize(stream, m.dwrd[i]);
    }
  }
};

///
/// @brief Serializes the AidEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidEPH_<ContainerAllocator> >
{
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::AidEPH_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::AidEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;
      m.sf1d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      m.sf2d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      m.sf3d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    }
  }

  static uint32_t serializedLength(const ublox_msgs::AidEPH_<ContainerAllocator> &m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::AidEPH_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for (std::size_t i = 0; i < m.sf1d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf1d[i]);
    }
    for (std::size_t i = 0; i < m.sf2d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf2d[i]);
    }
    for (std::size_t i = 0; i < m.sf3d.size(); ++i) {
      ros::serialization::serialize(stream, m.sf3d[i]);
    }
  }
};

///
/// @brief Serializes the EsfMEAS message which has a repeated block and an
/// optional block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfMEAS_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::EsfMEAS_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.time_tag);
    stream.next(m.flags);
    stream.next(m.id);

    bool calib_valid = m.flags & m.FLAGS_CALIB_T_TAG_VALID;
    int data_size = (count - (calib_valid ? 12 : 8)) / 4;
    // Repeating block
    m.data.resize(data_size);
    for (std::size_t i = 0; i < data_size; ++i) {
      ros::serialization::deserialize(stream, m.data[i]);
    }
    // Optional block
    if (calib_valid) {
      m.calib_t_tag.resize(1);
      ros::serialization::deserialize(stream, m.calib_t_tag[0]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::EsfMEAS_<ContainerAllocator> &m) {
    return 4 + 8 * m.data.size() + 4 * m.calib_t_tag.size();
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::EsfMEAS_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.time_tag);
    stream.next(m.flags);
    stream.next(m.id);
    for (std::size_t i = 0; i < m.data.size(); ++i) {
      ros::serialization::serialize(stream, m.data[i]);
    }
    for (std::size_t i = 0; i < m.calib_t_tag.size(); ++i) {
      ros::serialization::serialize(stream, m.calib_t_tag[i]);
    }
  }
};

///
/// @brief Serializes the EsfRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfRAW_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::EsfRAW_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.reserved0);
    m.blocks.clear();
    int num_blocks = (count - 4) / 8;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) {
      ros::serialization::deserialize(stream, m.blocks[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::EsfRAW_<ContainerAllocator> &m) {
    return 4 + 8 * m.blocks.size();
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::EsfRAW_<ContainerAllocator> &m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      ros::serialization::serialize(stream, m.blocks[i]);
    }
  }
};

///
/// @brief Serializes the EsfSTATUS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfSTATUS_<ContainerAllocator> > {
  static void read(const uint8_t *data, uint32_t count,
                   ublox_msgs::EsfSTATUS_<ContainerAllocator> &m) {
    ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.fusion_mode);
    stream.next(m.reserved2);
    stream.next(m.num_sens);
    m.sens.resize(m.num_sens);
    for (std::size_t i = 0; i < m.sens.size(); ++i) {
      ros::serialization::deserialize(stream, m.sens[i]);
    }
  }

  static uint32_t serializedLength(const ublox_msgs::EsfSTATUS_<ContainerAllocator> &m) {
    return 16 + 4 * m.num_sens;
  }

  static void write(uint8_t *data, uint32_t size,
                    const ublox_msgs::EsfSTATUS_<ContainerAllocator> &m) {
    if (m.sens.size() != m.num_sens) {
      ROS_ERROR("Writing EsfSTATUS message: num_sens must equal size of sens");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.fusion_mode);
    stream.next(m.reserved2);
    stream.next(static_cast<typename ublox_msgs::EsfSTATUS_<ContainerAllocator>::_num_sens_type>(m.sens.size()));
    for (std::size_t i = 0; i < m.sens.size(); ++i) {
      ros::serialization::serialize(stream, m.sens[i]);
    }
  }
};


} // namespace ublox

#endif // UBLOX_SERIALIZATION_UBLOX_MSGS_HPP
