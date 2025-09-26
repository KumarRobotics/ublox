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


#ifndef UBLOX_MSGS_SERIALIZATION_HPP
#define UBLOX_MSGS_SERIALIZATION_HPP

#include <cstdint>

#include <ublox_serialization/serialization.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

///
/// This file declares custom serializers for u-blox messages with dynamic
/// lengths and messages where the get/set messages have different sizes, but
/// share the same parameters, such as CfgDAT.
///

namespace ublox {

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::Ack_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::Ack_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.cls_id);
    stream.next(m.msg_id);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::Ack_<ContainerAllocator> &m) {
    (void)m;
    return 2;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::Ack_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.cls_id);
    stream.next(m.msg_id);
  }
};

///
/// @brief Serializes the AidALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::AidALM_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::AidALM_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename ublox_msgs::msg::AidALM_<ContainerAllocator>::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::AidALM_<ContainerAllocator> &m) {
    return 8 + (4 * m.dwrd.size());
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::AidALM_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      serialize(stream, m.dwrd[i]);
    }
  }
};

///
/// @brief Serializes the AidEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::AidEPH_<ContainerAllocator> >
{
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::AidEPH_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename ublox_msgs::msg::AidEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::msg::AidEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::msg::AidEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;
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

  inline static uint32_t serializedLength(const ublox_msgs::msg::AidEPH_<ContainerAllocator> &m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::AidEPH_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for (std::size_t i = 0; i < m.sf1d.size(); ++i) {
      serialize(stream, m.sf1d[i]);
    }
    for (std::size_t i = 0; i < m.sf2d.size(); ++i) {
      serialize(stream, m.sf2d[i]);
    }
    for (std::size_t i = 0; i < m.sf3d.size(); ++i) {
      serialize(stream, m.sf3d[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::AidHUI_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::AidHUI_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.health);
    stream.next(m.utc_a0);
    stream.next(m.utc_a1);
    stream.next(m.utc_tow);
    stream.next(m.utc_wnt);
    stream.next(m.utc_dn);
    stream.next(m.utc_lsf);
    stream.next(m.utc_spare);
    stream.next(m.klob_a0);
    stream.next(m.klob_a1);
    stream.next(m.klob_a2);
    stream.next(m.klob_a3);
    stream.next(m.klob_b0);
    stream.next(m.klob_b1);
    stream.next(m.klob_b2);
    stream.next(m.klob_b3);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::AidHUI_<ContainerAllocator> &m) {
    (void)m;
    return 72;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::AidHUI_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.health);
    stream.next(m.utc_a0);
    stream.next(m.utc_a1);
    stream.next(m.utc_tow);
    stream.next(m.utc_wnt);
    stream.next(m.utc_dn);
    stream.next(m.utc_lsf);
    stream.next(m.utc_spare);
    stream.next(m.klob_a0);
    stream.next(m.klob_a1);
    stream.next(m.klob_a2);
    stream.next(m.klob_a3);
    stream.next(m.klob_b0);
    stream.next(m.klob_b1);
    stream.next(m.klob_b2);
    stream.next(m.klob_b3);
    stream.next(m.flags);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgCFG_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgCFG_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.clear_mask);
    stream.next(m.save_mask);
    stream.next(m.load_mask);
    stream.next(m.device_mask);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgCFG_<ContainerAllocator> &m) {
    (void)m;
    return 13;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgCFG_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.clear_mask);
    stream.next(m.save_mask);
    stream.next(m.load_mask);
    stream.next(m.device_mask);
  }
};

///
/// @brief Serializes the CfgDAT message which has a different length for
/// get/set.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgDAT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgDAT_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
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

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgDAT_<ContainerAllocator> &m) {
    // this is the size of CfgDAT set messages
    // serializedLength is only used for writes so this is ok
    (void)m;
    return 44;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgDAT_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
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

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgDGNSS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgDGNSS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.dgnss_mode);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgDGNSS_<ContainerAllocator> &m) {
    (void)m;
    return 4;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgDGNSS_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.dgnss_mode);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgGNSSBlock_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::CfgGNSSBlock_<ContainerAllocator> & m) {
    stream.next(m.gnss_id);
    stream.next(m.res_trk_ch);
    stream.next(m.max_trk_ch);
    stream.next(m.reserved1);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgGNSSBlock_<ContainerAllocator> & m) {
    (void)m;
    return 8;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::CfgGNSSBlock_<ContainerAllocator> & m) {
    stream.next(m.gnss_id);
    stream.next(m.res_trk_ch);
    stream.next(m.max_trk_ch);
    stream.next(m.reserved1);
    stream.next(m.flags);
  }
};

///
/// @brief Serializes the CfgGNSS message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgGNSS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgGNSS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.msg_ver);
    stream.next(m.num_trk_ch_hw);
    stream.next(m.num_trk_ch_use);
    stream.next(m.num_config_blocks);
    m.blocks.resize(m.num_config_blocks);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      deserialize(stream, m.blocks[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgGNSS_<ContainerAllocator> &m) {
    return 4 + 8 * m.num_config_blocks;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgGNSS_<ContainerAllocator> &m) {
    if (m.blocks.size() != m.num_config_blocks) {
      // ROS_ERROR("CfgGNSS num_config_blocks must equal blocks size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.msg_ver);
    stream.next(m.num_trk_ch_hw);
    stream.next(m.num_trk_ch_use);
    stream.next(
        static_cast<typename ublox_msgs::msg::CfgGNSS_<ContainerAllocator>::_num_config_blocks_type>(m.blocks.size()));
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      serialize(stream, m.blocks[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgINFBlock_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::CfgINFBlock_<ContainerAllocator> & m) {
    stream.next(m.protocol_id);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.inf_msg_mask[0]);
    stream.next(m.inf_msg_mask[1]);
    stream.next(m.inf_msg_mask[2]);
    stream.next(m.inf_msg_mask[3]);
    stream.next(m.inf_msg_mask[4]);
    stream.next(m.inf_msg_mask[5]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgINFBlock_<ContainerAllocator> & m) {
    (void)m;
    return 10;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::CfgINFBlock_<ContainerAllocator> & m) {
    stream.next(m.protocol_id);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.inf_msg_mask[0]);
    stream.next(m.inf_msg_mask[1]);
    stream.next(m.inf_msg_mask[2]);
    stream.next(m.inf_msg_mask[3]);
    stream.next(m.inf_msg_mask[4]);
    stream.next(m.inf_msg_mask[5]);
  }
};

///
/// @brief Serializes the CfgInf message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgINF_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgINF_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    uint32_t num_blocks = count / 10;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) {
      deserialize(stream, m.blocks[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgINF_<ContainerAllocator> &m) {
    return 10 * m.blocks.size();
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgINF_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      serialize(stream, m.blocks[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgMSG_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgMSG_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.msg_class);
    stream.next(m.msg_id);
    stream.next(m.rate);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgMSG_<ContainerAllocator> &m) {
    (void)m;
    return 3;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgMSG_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.msg_class);
    stream.next(m.msg_id);
    stream.next(m.rate);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgNAV5_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgNAV5_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.mask);
    stream.next(m.dyn_model);
    stream.next(m.fix_mode);
    stream.next(m.fixed_alt);
    stream.next(m.fixed_alt_var);
    stream.next(m.min_elev);
    stream.next(m.dr_limit);
    stream.next(m.p_dop);
    stream.next(m.t_dop);
    stream.next(m.p_acc);
    stream.next(m.t_acc);
    stream.next(m.static_hold_thresh);
    stream.next(m.dgnss_time_out);
    stream.next(m.cno_thresh_num_svs);
    stream.next(m.cno_thresh);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.static_hold_max_dist);
    stream.next(m.utc_standard);
    stream.next(m.reserved2[0]);
    stream.next(m.reserved2[1]);
    stream.next(m.reserved2[2]);
    stream.next(m.reserved2[3]);
    stream.next(m.reserved2[4]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgNAV5_<ContainerAllocator> &m) {
    (void)m;
    return 36;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgNAV5_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.mask);
    stream.next(m.dyn_model);
    stream.next(m.fix_mode);
    stream.next(m.fixed_alt);
    stream.next(m.fixed_alt_var);
    stream.next(m.min_elev);
    stream.next(m.dr_limit);
    stream.next(m.p_dop);
    stream.next(m.t_dop);
    stream.next(m.p_acc);
    stream.next(m.t_acc);
    stream.next(m.static_hold_thresh);
    stream.next(m.dgnss_time_out);
    stream.next(m.cno_thresh_num_svs);
    stream.next(m.cno_thresh);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.static_hold_max_dist);
    stream.next(m.utc_standard);
    stream.next(m.reserved2[0]);
    stream.next(m.reserved2[1]);
    stream.next(m.reserved2[2]);
    stream.next(m.reserved2[3]);
    stream.next(m.reserved2[4]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgNAVX5_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgNAVX5_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.mask1);
    stream.next(m.mask2);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.min_svs);
    stream.next(m.max_svs);
    stream.next(m.min_cno);
    stream.next(m.reserved2);
    stream.next(m.ini_fix3d);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.ack_aiding);
    stream.next(m.wkn_rollover);
    stream.next(m.sig_atten_comp_mode);
    stream.next(m.reserved4[0]);
    stream.next(m.reserved4[1]);
    stream.next(m.reserved4[2]);
    stream.next(m.reserved4[3]);
    stream.next(m.reserved4[4]);
    stream.next(m.use_ppp);
    stream.next(m.aop_cfg);
    stream.next(m.reserved5[0]);
    stream.next(m.reserved5[1]);
    stream.next(m.aop_orb_max_err);
    stream.next(m.reserved6[0]);
    stream.next(m.reserved6[1]);
    stream.next(m.reserved6[2]);
    stream.next(m.reserved6[3]);
    stream.next(m.reserved6[4]);
    stream.next(m.reserved6[5]);
    stream.next(m.reserved6[6]);
    stream.next(m.use_adr);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgNAVX5_<ContainerAllocator> &m) {
    (void)m;
    return 40;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgNAVX5_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.mask1);
    stream.next(m.mask2);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.min_svs);
    stream.next(m.max_svs);
    stream.next(m.min_cno);
    stream.next(m.reserved2);
    stream.next(m.ini_fix3d);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.ack_aiding);
    stream.next(m.wkn_rollover);
    stream.next(m.sig_atten_comp_mode);
    stream.next(m.reserved4[0]);
    stream.next(m.reserved4[1]);
    stream.next(m.reserved4[2]);
    stream.next(m.reserved4[3]);
    stream.next(m.reserved4[4]);
    stream.next(m.use_ppp);
    stream.next(m.aop_cfg);
    stream.next(m.reserved5[0]);
    stream.next(m.reserved5[1]);
    stream.next(m.aop_orb_max_err);
    stream.next(m.reserved6[0]);
    stream.next(m.reserved6[1]);
    stream.next(m.reserved6[2]);
    stream.next(m.reserved6[3]);
    stream.next(m.reserved6[4]);
    stream.next(m.reserved6[5]);
    stream.next(m.reserved6[6]);
    stream.next(m.use_adr);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgNMEA_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgNMEA_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.filter);
    stream.next(m.nmea_version);
    stream.next(m.num_sv);
    stream.next(m.flags);
    stream.next(m.gnss_to_filter);
    stream.next(m.sv_numbering);
    stream.next(m.main_talker_id);
    stream.next(m.gsv_talker_id);
    stream.next(m.version);
    stream.next(m.bds_talker_id[0]);
    stream.next(m.bds_talker_id[1]);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgNMEA_<ContainerAllocator> &m) {
    (void)m;
    return 20;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgNMEA_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.filter);
    stream.next(m.nmea_version);
    stream.next(m.num_sv);
    stream.next(m.flags);
    stream.next(m.gnss_to_filter);
    stream.next(m.sv_numbering);
    stream.next(m.main_talker_id);
    stream.next(m.gsv_talker_id);
    stream.next(m.version);
    stream.next(m.bds_talker_id[0]);
    stream.next(m.bds_talker_id[1]);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgNMEA6_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgNMEA6_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.filter);
    stream.next(m.version);
    stream.next(m.num_sv);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgNMEA6_<ContainerAllocator> &m) {
    (void)m;
    return 4;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgNMEA6_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.filter);
    stream.next(m.version);
    stream.next(m.num_sv);
    stream.next(m.flags);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgNMEA7_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgNMEA7_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.filter);
    stream.next(m.nmea_version);
    stream.next(m.num_sv);
    stream.next(m.flags);
    stream.next(m.gnss_to_filter);
    stream.next(m.sv_numbering);
    stream.next(m.main_talker_id);
    stream.next(m.gsv_talker_id);
    stream.next(m.reserved);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgNMEA7_<ContainerAllocator> &m) {
    (void)m;
    return 12;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgNMEA7_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.filter);
    stream.next(m.nmea_version);
    stream.next(m.num_sv);
    stream.next(m.flags);
    stream.next(m.gnss_to_filter);
    stream.next(m.sv_numbering);
    stream.next(m.main_talker_id);
    stream.next(m.gsv_talker_id);
    stream.next(m.reserved);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgPRT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgPRT_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.port_id);
    stream.next(m.reserved0);
    stream.next(m.tx_ready);
    stream.next(m.mode);
    stream.next(m.baud_rate);
    stream.next(m.in_proto_mask);
    stream.next(m.out_proto_mask);
    stream.next(m.flags);
    stream.next(m.reserved1);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgPRT_<ContainerAllocator> &m) {
    (void)m;
    return 20;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgPRT_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.port_id);
    stream.next(m.reserved0);
    stream.next(m.tx_ready);
    stream.next(m.mode);
    stream.next(m.baud_rate);
    stream.next(m.in_proto_mask);
    stream.next(m.out_proto_mask);
    stream.next(m.flags);
    stream.next(m.reserved1);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgRATE_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgRATE_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.meas_rate);
    stream.next(m.nav_rate);
    stream.next(m.time_ref);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgRATE_<ContainerAllocator> &m) {
    (void)m;
    return 6;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgRATE_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.meas_rate);
    stream.next(m.nav_rate);
    stream.next(m.time_ref);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgRST_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgRST_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.nav_bbr_mask);
    stream.next(m.reset_mode);
    stream.next(m.reserved1);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgRST_<ContainerAllocator> &m) {
    (void)m;
    return 4;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgRST_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.nav_bbr_mask);
    stream.next(m.reset_mode);
    stream.next(m.reserved1);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgSBAS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgSBAS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.mode);
    stream.next(m.usage);
    stream.next(m.max_sbas);
    stream.next(m.scanmode2);
    stream.next(m.scanmode1);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgSBAS_<ContainerAllocator> &m) {
    (void)m;
    return 8;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgSBAS_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.mode);
    stream.next(m.usage);
    stream.next(m.max_sbas);
    stream.next(m.scanmode2);
    stream.next(m.scanmode1);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgTMODE3_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgTMODE3_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.flags);
    stream.next(m.ecef_x_or_lat);
    stream.next(m.ecef_y_or_lon);
    stream.next(m.ecef_z_or_alt);
    stream.next(m.ecef_x_or_lat_hp);
    stream.next(m.ecef_y_or_lon_hp);
    stream.next(m.ecef_z_or_alt_hp);
    stream.next(m.reserved2);
    stream.next(m.fixed_pos_acc);
    stream.next(m.svin_min_dur);
    stream.next(m.svin_acc_limit);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.reserved3[2]);
    stream.next(m.reserved3[3]);
    stream.next(m.reserved3[4]);
    stream.next(m.reserved3[5]);
    stream.next(m.reserved3[6]);
    stream.next(m.reserved3[7]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgTMODE3_<ContainerAllocator> &m) {
    (void)m;
    return 40;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::CfgTMODE3_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.flags);
    stream.next(m.ecef_x_or_lat);
    stream.next(m.ecef_y_or_lon);
    stream.next(m.ecef_z_or_alt);
    stream.next(m.ecef_x_or_lat_hp);
    stream.next(m.ecef_y_or_lon_hp);
    stream.next(m.ecef_z_or_alt_hp);
    stream.next(m.reserved2);
    stream.next(m.fixed_pos_acc);
    stream.next(m.svin_min_dur);
    stream.next(m.svin_acc_limit);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.reserved3[2]);
    stream.next(m.reserved3[3]);
    stream.next(m.reserved3[4]);
    stream.next(m.reserved3[5]);
    stream.next(m.reserved3[6]);
    stream.next(m.reserved3[7]);
  }
};

///
/// @brief Serializes the CfgVALDEL message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgVALDEL_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgVALDEL_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    for (std::size_t i = 0; i < m.keys.size(); ++i) {
      deserialize(stream, m.keys[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgVALDEL_<ContainerAllocator> &m) {
    return 4 + 4 * m.keys.size();
  }

  inline static void write(const uint8_t *data, uint32_t count,
                           const ublox_msgs::msg::CfgVALDEL_<ContainerAllocator> &m) {
    UbloxOStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    for (std::size_t i = 0; i < m.keys.size(); ++i) {
      serialize(stream, m.keys[i]);
    }
  }
};

///
/// @brief Serializes the CfgVALGET message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgVALGET_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgVALGET_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.position);
    uint8_t data_size = (count - 4) / 4;
    m.keys.resize(data_size);
    // the key and values are all together in the keys array
    for (std::size_t i = 0; i < data_size; ++i) {
      deserialize(stream, m.keys[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgVALGET_<ContainerAllocator> &m) {
    return 4 + 4 * m.keys.size();
  }

  inline static void write(const uint8_t *data, uint32_t count,
                           const ublox_msgs::msg::CfgVALGET_<ContainerAllocator> &m) {
    UbloxOStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.position);
    for (std::size_t i = 0; i < m.keys.size(); ++i) {
      serialize(stream, m.keys[i]);
    }
  }
};

///
/// @brief Serializes the CfgVALSET message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::CfgVALSET_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::CfgVALSET_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    for (std::size_t i = 0; i < m.cfgdata.size(); ++i) {
      deserialize(stream, m.cfgdata[i].key);
      for (std::size_t j = 0; j < m.cfgdata[i].data.size(); ++j) {
        deserialize(stream, m.cfgdata[i].data[j]);
      }
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::CfgVALSET_<ContainerAllocator> &m) {
    int data_size = 0;
    for (std::size_t i = 0; i < m.cfgdata.size(); ++i) {
      data_size += m.cfgdata[i].data.size();
      data_size += 4;
    }
    return 4 + data_size;
  }

  inline static void write(const uint8_t *data, uint32_t count,
                           const ublox_msgs::msg::CfgVALSET_<ContainerAllocator> &m) {
    UbloxOStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.layers);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    for (std::size_t i = 0; i < m.cfgdata.size(); ++i) {
      serialize(stream, m.cfgdata[i].key);
      for (std::size_t j = 0; j < m.cfgdata[i].data.size(); ++j) {
        serialize(stream, m.cfgdata[i].data[j]);
      }
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfINS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::EsfINS_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.bitfield0);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.i_tow);
    stream.next(m.x_ang_rate);
    stream.next(m.y_ang_rate);
    stream.next(m.z_ang_rate);
    stream.next(m.x_accel);
    stream.next(m.y_accel);
    stream.next(m.z_accel);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfINS_<ContainerAllocator> & m) {
    (void)m;
    return 36;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::EsfINS_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.bitfield0);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.i_tow);
    stream.next(m.x_ang_rate);
    stream.next(m.y_ang_rate);
    stream.next(m.z_ang_rate);
    stream.next(m.x_accel);
    stream.next(m.y_accel);
    stream.next(m.z_accel);
  }
};

///
/// @brief Serializes the EsfMEAS message which has a repeated block and an
/// optional block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfMEAS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::EsfMEAS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.time_tag);
    stream.next(m.flags);
    stream.next(m.id);

    bool calib_valid = m.flags & m.FLAGS_CALIB_T_TAG_VALID;
    uint32_t data_size = (count - (calib_valid ? 12 : 8)) / 4;
    // Repeating block
    m.data.resize(data_size);
    for (std::size_t i = 0; i < data_size; ++i) {
      deserialize(stream, m.data[i]);
    }
    // Optional block
    if (calib_valid) {
      m.calib_t_tag.resize(1);
      deserialize(stream, m.calib_t_tag[0]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfMEAS_<ContainerAllocator> &m) {
    return 4 + 8 * m.data.size() + 4 * m.calib_t_tag.size();
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::EsfMEAS_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.time_tag);
    stream.next(m.flags);
    stream.next(m.id);
    for (std::size_t i = 0; i < m.data.size(); ++i) {
      serialize(stream, m.data[i]);
    }
    for (std::size_t i = 0; i < m.calib_t_tag.size(); ++i) {
      serialize(stream, m.calib_t_tag[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfRAWBlock_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::EsfRAWBlock_<ContainerAllocator> & m) {
    stream.next(m.data);
    stream.next(m.s_t_tag);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfRAWBlock_<ContainerAllocator> & m) {
    (void)m;
    return 8;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::EsfRAWBlock_<ContainerAllocator> & m) {
    stream.next(m.data);
    stream.next(m.s_t_tag);
  }
};

///
/// @brief Serializes the EsfRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfRAW_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::EsfRAW_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.reserved0);
    m.blocks.clear();
    uint32_t num_blocks = (count - 4) / 8;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) {
      deserialize(stream, m.blocks[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfRAW_<ContainerAllocator> &m) {
    return 4 + 8 * m.blocks.size();
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::EsfRAW_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) {
      serialize(stream, m.blocks[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfSTATUSSens_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::EsfSTATUSSens_<ContainerAllocator> & m) {
    stream.next(m.sens_status1);
    stream.next(m.sens_status2);
    stream.next(m.freq);
    stream.next(m.faults);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfSTATUSSens_<ContainerAllocator> & m) {
    (void)m;
    return 4;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::EsfSTATUSSens_<ContainerAllocator> & m) {
    stream.next(m.sens_status1);
    stream.next(m.sens_status2);
    stream.next(m.freq);
    stream.next(m.faults);
  }
};

///
/// @brief Serializes the EsfSTATUS message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::EsfSTATUS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::EsfSTATUS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.fusion_mode);
    stream.next(m.reserved2);
    stream.next(m.num_sens);
    m.sens.resize(m.num_sens);
    for (std::size_t i = 0; i < m.sens.size(); ++i) {
      deserialize(stream, m.sens[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::EsfSTATUS_<ContainerAllocator> &m) {
    return 16 + 4 * m.num_sens;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::EsfSTATUS_<ContainerAllocator> &m) {
    if (m.sens.size() != m.num_sens) {
      // ROS_ERROR("Writing EsfSTATUS message: num_sens must equal size of sens");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.fusion_mode);
    stream.next(m.reserved2);
    stream.next(static_cast<typename ublox_msgs::msg::EsfSTATUS_<ContainerAllocator>::_num_sens_type>(m.sens.size()));
    for (std::size_t i = 0; i < m.sens.size(); ++i) {
      serialize(stream, m.sens[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::HnrPVT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::HnrPVT_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.nano);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.g_speed);
    stream.next(m.speed);
    stream.next(m.head_mot);
    stream.next(m.head_veh);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::HnrPVT_<ContainerAllocator> & m) {
    (void)m;
    return 72;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::HnrPVT_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.nano);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.g_speed);
    stream.next(m.speed);
    stream.next(m.head_mot);
    stream.next(m.head_veh);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
  }
};

///
/// @brief Serializes the Inf message which has a dynamic length string.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::Inf_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::Inf_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    m.str.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
      deserialize(stream, m.str[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::Inf_<ContainerAllocator> &m) {
    return m.str.size();
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::Inf_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    for (std::size_t i = 0; i < m.str.size(); ++i) {
      serialize(stream, m.str[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::MonHW_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::MonHW_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.pin_sel);
    stream.next(m.pin_bank);
    stream.next(m.pin_dir);
    stream.next(m.pin_val);
    stream.next(m.noise_per_ms);
    stream.next(m.agc_cnt);
    stream.next(m.a_status);
    stream.next(m.a_power);
    stream.next(m.flags);
    stream.next(m.reserved0);
    stream.next(m.used_mask);
    stream.next(m.vp[0]);
    stream.next(m.vp[1]);
    stream.next(m.vp[2]);
    stream.next(m.vp[3]);
    stream.next(m.vp[4]);
    stream.next(m.vp[5]);
    stream.next(m.vp[6]);
    stream.next(m.vp[7]);
    stream.next(m.vp[8]);
    stream.next(m.vp[9]);
    stream.next(m.vp[10]);
    stream.next(m.vp[11]);
    stream.next(m.vp[12]);
    stream.next(m.vp[13]);
    stream.next(m.vp[14]);
    stream.next(m.vp[15]);
    stream.next(m.vp[16]);
    stream.next(m.jam_ind);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.pin_irq);
    stream.next(m.pull_h);
    stream.next(m.pull_l);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::MonHW_<ContainerAllocator> & m) {
    (void)m;
    return 60;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::MonHW_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.pin_sel);
    stream.next(m.pin_bank);
    stream.next(m.pin_dir);
    stream.next(m.pin_val);
    stream.next(m.noise_per_ms);
    stream.next(m.agc_cnt);
    stream.next(m.a_status);
    stream.next(m.a_power);
    stream.next(m.flags);
    stream.next(m.reserved0);
    stream.next(m.used_mask);
    stream.next(m.vp[0]);
    stream.next(m.vp[1]);
    stream.next(m.vp[2]);
    stream.next(m.vp[3]);
    stream.next(m.vp[4]);
    stream.next(m.vp[5]);
    stream.next(m.vp[6]);
    stream.next(m.vp[7]);
    stream.next(m.vp[8]);
    stream.next(m.vp[9]);
    stream.next(m.vp[10]);
    stream.next(m.vp[11]);
    stream.next(m.vp[12]);
    stream.next(m.vp[13]);
    stream.next(m.vp[14]);
    stream.next(m.vp[15]);
    stream.next(m.vp[16]);
    stream.next(m.jam_ind);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.pin_irq);
    stream.next(m.pull_h);
    stream.next(m.pull_l);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::MonHW6_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::MonHW6_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.pin_sel);
    stream.next(m.pin_bank);
    stream.next(m.pin_dir);
    stream.next(m.pin_val);
    stream.next(m.noise_per_ms);
    stream.next(m.agc_cnt);
    stream.next(m.a_status);
    stream.next(m.a_power);
    stream.next(m.flags);
    stream.next(m.reserved0);
    stream.next(m.used_mask);
    stream.next(m.vp[0]);
    stream.next(m.vp[1]);
    stream.next(m.vp[2]);
    stream.next(m.vp[3]);
    stream.next(m.vp[4]);
    stream.next(m.vp[5]);
    stream.next(m.vp[6]);
    stream.next(m.vp[7]);
    stream.next(m.vp[8]);
    stream.next(m.vp[9]);
    stream.next(m.vp[10]);
    stream.next(m.vp[11]);
    stream.next(m.vp[12]);
    stream.next(m.vp[13]);
    stream.next(m.vp[14]);
    stream.next(m.vp[15]);
    stream.next(m.vp[16]);
    stream.next(m.vp[17]);
    stream.next(m.vp[18]);
    stream.next(m.vp[19]);
    stream.next(m.vp[20]);
    stream.next(m.vp[21]);
    stream.next(m.vp[22]);
    stream.next(m.vp[23]);
    stream.next(m.vp[24]);
    stream.next(m.jam_ind);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.pin_irq);
    stream.next(m.pull_h);
    stream.next(m.pull_l);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::MonHW6_<ContainerAllocator> & m) {
    (void)m;
    return 68;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::MonHW6_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.pin_sel);
    stream.next(m.pin_bank);
    stream.next(m.pin_dir);
    stream.next(m.pin_val);
    stream.next(m.noise_per_ms);
    stream.next(m.agc_cnt);
    stream.next(m.a_status);
    stream.next(m.a_power);
    stream.next(m.flags);
    stream.next(m.reserved0);
    stream.next(m.used_mask);
    stream.next(m.vp[0]);
    stream.next(m.vp[1]);
    stream.next(m.vp[2]);
    stream.next(m.vp[3]);
    stream.next(m.vp[4]);
    stream.next(m.vp[5]);
    stream.next(m.vp[6]);
    stream.next(m.vp[7]);
    stream.next(m.vp[8]);
    stream.next(m.vp[9]);
    stream.next(m.vp[10]);
    stream.next(m.vp[11]);
    stream.next(m.vp[12]);
    stream.next(m.vp[13]);
    stream.next(m.vp[14]);
    stream.next(m.vp[15]);
    stream.next(m.vp[16]);
    stream.next(m.vp[17]);
    stream.next(m.vp[18]);
    stream.next(m.vp[19]);
    stream.next(m.vp[20]);
    stream.next(m.vp[21]);
    stream.next(m.vp[22]);
    stream.next(m.vp[23]);
    stream.next(m.vp[24]);
    stream.next(m.jam_ind);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.pin_irq);
    stream.next(m.pull_h);
    stream.next(m.pull_l);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::MonVERExtension_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::MonVERExtension_<ContainerAllocator> & m) {
    stream.next(m.field);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::MonVERExtension_<ContainerAllocator> & m) {
    (void)m;
    return 30;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::MonVERExtension_<ContainerAllocator> & m) {
    stream.next(m.field);
  }
};

///
/// @brief Serializes the MonVER message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::MonVER_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::MonVER_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.sw_version);
    stream.next(m.hw_version);

    m.extension.clear();
    int N = (count - 40) / 30;
    m.extension.resize(N);
    typename ublox_msgs::msg::MonVER_<ContainerAllocator>::_extension_type::value_type ext;
    for (int i = 0; i < N; i++) {
      // Read each extension string
      deserialize(stream, m.extension[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::MonVER_<ContainerAllocator> &m) {
    return 40 + (30 * m.extension.size());
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::MonVER_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.sw_version);
    stream.next(m.hw_version);
    for (std::size_t i = 0; i < m.extension.size(); ++i) {
      serialize(stream, m.extension[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavATT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavATT_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.heading);
    stream.next(m.acc_roll);
    stream.next(m.acc_pitch);
    stream.next(m.acc_heading);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavATT_<ContainerAllocator> & m) {
    (void)m;
    return 32;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavATT_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.heading);
    stream.next(m.acc_roll);
    stream.next(m.acc_pitch);
    stream.next(m.acc_heading);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavCLOCK_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavCLOCK_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.clk_b);
    stream.next(m.clk_d);
    stream.next(m.t_acc);
    stream.next(m.f_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavCLOCK_<ContainerAllocator> & m) {
    (void)m;
    return 20;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavCLOCK_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.clk_b);
    stream.next(m.clk_d);
    stream.next(m.t_acc);
    stream.next(m.f_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavCOV_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavCOV_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.pos_cov_valid);
    stream.next(m.vel_cov_valid);
    stream.next(m.reserved_0[0]);
    stream.next(m.reserved_0[1]);
    stream.next(m.reserved_0[2]);
    stream.next(m.reserved_0[3]);
    stream.next(m.reserved_0[4]);
    stream.next(m.reserved_0[5]);
    stream.next(m.reserved_0[6]);
    stream.next(m.reserved_0[7]);
    stream.next(m.reserved_0[8]);
    stream.next(m.pos_cov_nn);
    stream.next(m.pos_cov_ne);
    stream.next(m.pos_cov_nd);
    stream.next(m.pos_cov_ee);
    stream.next(m.pos_cov_ed);
    stream.next(m.pos_cov_dd);
    stream.next(m.vel_cov_nn);
    stream.next(m.vel_cov_ne);
    stream.next(m.vel_cov_nd);
    stream.next(m.vel_cov_ee);
    stream.next(m.vel_cov_ed);
    stream.next(m.vel_cov_dd);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavCOV_<ContainerAllocator> & m) {
    (void)m;
    return 64;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavCOV_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.pos_cov_valid);
    stream.next(m.vel_cov_valid);
    stream.next(m.reserved_0[0]);
    stream.next(m.reserved_0[1]);
    stream.next(m.reserved_0[2]);
    stream.next(m.reserved_0[3]);
    stream.next(m.reserved_0[4]);
    stream.next(m.reserved_0[5]);
    stream.next(m.reserved_0[6]);
    stream.next(m.reserved_0[7]);
    stream.next(m.reserved_0[8]);
    stream.next(m.pos_cov_nn);
    stream.next(m.pos_cov_ne);
    stream.next(m.pos_cov_nd);
    stream.next(m.pos_cov_ee);
    stream.next(m.pos_cov_ed);
    stream.next(m.pos_cov_dd);
    stream.next(m.vel_cov_nn);
    stream.next(m.vel_cov_ne);
    stream.next(m.vel_cov_nd);
    stream.next(m.vel_cov_ee);
    stream.next(m.vel_cov_ed);
    stream.next(m.vel_cov_dd);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavDGPSSV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::NavDGPSSV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.age_c);
    stream.next(m.prc);
    stream.next(m.prrc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavDGPSSV_<ContainerAllocator> & m) {
    (void)m;
    return 12;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::NavDGPSSV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.age_c);
    stream.next(m.prc);
    stream.next(m.prrc);
  }
};

///
/// @brief Serializes the NavDGPS message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavDGPS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavDGPS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.age);
    stream.next(m.base_id);
    stream.next(m.base_health);
    stream.next(m.num_ch);
    stream.next(m.status);
    stream.next(m.reserved1);
    m.sv.resize(m.num_ch);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavDGPS_<ContainerAllocator> &m) {
    return 16 + 12 * m.num_ch;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavDGPS_<ContainerAllocator> &m) {
    if (m.sv.size() != static_cast<size_t>(m.num_ch)) {
      // ROS_ERROR("NavDGPS numCh must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.age);
    stream.next(m.base_id);
    stream.next(m.base_health);
    stream.next(static_cast<typename ublox_msgs::msg::NavDGPS_<ContainerAllocator>::_num_ch_type>(m.sv.size()));
    stream.next(m.status);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavPOSECEF_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavPOSECEF_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.ecef_x);
    stream.next(m.ecef_y);
    stream.next(m.ecef_z);
    stream.next(m.p_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavPOSECEF_<ContainerAllocator> & m) {
    (void)m;
    return 20;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavPOSECEF_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.ecef_x);
    stream.next(m.ecef_y);
    stream.next(m.ecef_z);
    stream.next(m.p_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavPOSLLH_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavPOSLLH_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavPOSLLH_<ContainerAllocator> & m) {
    (void)m;
    return 28;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavPOSLLH_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavPVT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavPVT_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.fix_type);
    stream.next(m.flags);
    stream.next(m.flags2);
    stream.next(m.num_sv);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
    stream.next(m.head_veh);
    stream.next(m.mag_dec);
    stream.next(m.mag_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavPVT_<ContainerAllocator> & m) {
    (void)m;
    return 92;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavPVT_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.fix_type);
    stream.next(m.flags);
    stream.next(m.flags2);
    stream.next(m.num_sv);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
    stream.next(m.head_veh);
    stream.next(m.mag_dec);
    stream.next(m.mag_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavPVT7_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavPVT7_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.fix_type);
    stream.next(m.flags);
    stream.next(m.flags2);
    stream.next(m.num_sv);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavPVT7_<ContainerAllocator> & m) {
    (void)m;
    return 84;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavPVT7_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.fix_type);
    stream.next(m.flags);
    stream.next(m.flags2);
    stream.next(m.num_sv);
    stream.next(m.lon);
    stream.next(m.lat);
    stream.next(m.height);
    stream.next(m.h_msl);
    stream.next(m.h_acc);
    stream.next(m.v_acc);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.head_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
    stream.next(m.reserved1[3]);
    stream.next(m.reserved1[4]);
    stream.next(m.reserved1[5]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavRELPOSNED_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavRELPOSNED_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.reserved0);
    stream.next(m.ref_station_id);
    stream.next(m.i_tow);
    stream.next(m.rel_pos_n);
    stream.next(m.rel_pos_e);
    stream.next(m.rel_pos_d);
    stream.next(m.rel_pos_hpn);
    stream.next(m.rel_pos_hpe);
    stream.next(m.rel_pos_hpd);
    stream.next(m.reserved1);
    stream.next(m.acc_n);
    stream.next(m.acc_e);
    stream.next(m.acc_d);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavRELPOSNED_<ContainerAllocator> & m) {
    (void)m;
    return 40;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavRELPOSNED_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.reserved0);
    stream.next(m.ref_station_id);
    stream.next(m.i_tow);
    stream.next(m.rel_pos_n);
    stream.next(m.rel_pos_e);
    stream.next(m.rel_pos_d);
    stream.next(m.rel_pos_hpn);
    stream.next(m.rel_pos_hpe);
    stream.next(m.rel_pos_hpd);
    stream.next(m.reserved1);
    stream.next(m.acc_n);
    stream.next(m.acc_e);
    stream.next(m.acc_d);
    stream.next(m.flags);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavRELPOSNED9_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavRELPOSNED9_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.ref_station_id);
    stream.next(m.i_tow);
    stream.next(m.rel_pos_n);
    stream.next(m.rel_pos_e);
    stream.next(m.rel_pos_d);
    stream.next(m.rel_pos_length);
    stream.next(m.rel_pos_heading);
    stream.next(m.reserved2[0]);
    stream.next(m.reserved2[1]);
    stream.next(m.reserved2[2]);
    stream.next(m.reserved2[3]);
    stream.next(m.rel_pos_hpn);
    stream.next(m.rel_pos_hpe);
    stream.next(m.rel_pos_hpd);
    stream.next(m.rel_pos_hp_length);
    stream.next(m.acc_n);
    stream.next(m.acc_e);
    stream.next(m.acc_d);
    stream.next(m.acc_length);
    stream.next(m.acc_heading);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.reserved3[2]);
    stream.next(m.reserved3[3]);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavRELPOSNED9_<ContainerAllocator> & m) {
    (void)m;
    return 64;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavRELPOSNED9_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.reserved1);
    stream.next(m.ref_station_id);
    stream.next(m.i_tow);
    stream.next(m.rel_pos_n);
    stream.next(m.rel_pos_e);
    stream.next(m.rel_pos_d);
    stream.next(m.rel_pos_length);
    stream.next(m.rel_pos_heading);
    stream.next(m.reserved2[0]);
    stream.next(m.reserved2[1]);
    stream.next(m.reserved2[2]);
    stream.next(m.reserved2[3]);
    stream.next(m.rel_pos_hpn);
    stream.next(m.rel_pos_hpe);
    stream.next(m.rel_pos_hpd);
    stream.next(m.rel_pos_hp_length);
    stream.next(m.acc_n);
    stream.next(m.acc_e);
    stream.next(m.acc_d);
    stream.next(m.acc_length);
    stream.next(m.acc_heading);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
    stream.next(m.reserved3[2]);
    stream.next(m.reserved3[3]);
    stream.next(m.flags);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSATSV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::NavSATSV_<ContainerAllocator> & m) {
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.cno);
    stream.next(m.elev);
    stream.next(m.azim);
    stream.next(m.pr_res);
    stream.next(m.flags);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSATSV_<ContainerAllocator> & m) {
    (void)m;
    return 12;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::NavSATSV_<ContainerAllocator> & m) {
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.cno);
    stream.next(m.elev);
    stream.next(m.azim);
    stream.next(m.pr_res);
    stream.next(m.flags);
  }
};

///
/// @brief Serializes the NavSAT message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSAT_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSAT_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(m.num_svs);
    stream.next(m.reserved0);
    m.sv.resize(m.num_svs);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSAT_<ContainerAllocator> &m) {
    return 8 + 12 * m.num_svs;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSAT_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_svs) {
      // ROS_ERROR("NavSAT num_svs must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.version);
    stream.next(static_cast<typename ublox_msgs::msg::NavSAT_<ContainerAllocator>::_num_svs_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSBASSV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::NavSBASSV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.udre);
    stream.next(m.sv_sys);
    stream.next(m.sv_service);
    stream.next(m.reserved1);
    stream.next(m.prc);
    stream.next(m.reserved2);
    stream.next(m.ic);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSBASSV_<ContainerAllocator> & m) {
    (void)m;
    return 12;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::NavSBASSV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.udre);
    stream.next(m.sv_sys);
    stream.next(m.sv_service);
    stream.next(m.reserved1);
    stream.next(m.prc);
    stream.next(m.reserved2);
    stream.next(m.ic);
  }
};

///
/// @brief Serializes the NavSBAS message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSBAS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSBAS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(m.cnt);
    stream.next(m.reserved0);
    m.sv.resize(m.cnt);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSBAS_<ContainerAllocator> &m) {
    return 12 + 12 * m.cnt;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSBAS_<ContainerAllocator> &m) {
    if (m.sv.size() != m.cnt) {
      // ROS_ERROR("NavSBAS cnt must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(static_cast<typename ublox_msgs::msg::NavSBAS_<ContainerAllocator>::_cnt_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSOL_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSOL_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.f_tow);
    stream.next(m.week);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.ecef_x);
    stream.next(m.ecef_y);
    stream.next(m.ecef_z);
    stream.next(m.p_acc);
    stream.next(m.ecef_vx);
    stream.next(m.ecef_vy);
    stream.next(m.ecef_vz);
    stream.next(m.s_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1);
    stream.next(m.num_sv);
    stream.next(m.reserved2);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSOL_<ContainerAllocator> & m) {
    (void)m;
    return 52;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSOL_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.f_tow);
    stream.next(m.week);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.ecef_x);
    stream.next(m.ecef_y);
    stream.next(m.ecef_z);
    stream.next(m.p_acc);
    stream.next(m.ecef_vx);
    stream.next(m.ecef_vy);
    stream.next(m.ecef_vz);
    stream.next(m.s_acc);
    stream.next(m.p_dop);
    stream.next(m.reserved1);
    stream.next(m.num_sv);
    stream.next(m.reserved2);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSTATUS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSTATUS_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.fix_stat);
    stream.next(m.flags2);
    stream.next(m.ttff);
    stream.next(m.msss);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSTATUS_<ContainerAllocator> & m) {
    (void)m;
    return 16;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSTATUS_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.gps_fix);
    stream.next(m.flags);
    stream.next(m.fix_stat);
    stream.next(m.flags2);
    stream.next(m.ttff);
    stream.next(m.msss);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSVIN_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSVIN_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.i_tow);
    stream.next(m.dur);
    stream.next(m.mean_x);
    stream.next(m.mean_y);
    stream.next(m.mean_z);
    stream.next(m.mean_xhp);
    stream.next(m.mean_yhp);
    stream.next(m.mean_zhp);
    stream.next(m.reserved1);
    stream.next(m.mean_acc);
    stream.next(m.obs);
    stream.next(m.valid);
    stream.next(m.active);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSVIN_<ContainerAllocator> & m) {
    (void)m;
    return 40;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSVIN_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.i_tow);
    stream.next(m.dur);
    stream.next(m.mean_x);
    stream.next(m.mean_y);
    stream.next(m.mean_z);
    stream.next(m.mean_xhp);
    stream.next(m.mean_yhp);
    stream.next(m.mean_zhp);
    stream.next(m.reserved1);
    stream.next(m.mean_acc);
    stream.next(m.obs);
    stream.next(m.valid);
    stream.next(m.active);
    stream.next(m.reserved3[0]);
    stream.next(m.reserved3[1]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSVINFOSV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::NavSVINFOSV_<ContainerAllocator> & m) {
    stream.next(m.chn);
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.quality);
    stream.next(m.cno);
    stream.next(m.elev);
    stream.next(m.azim);
    stream.next(m.pr_res);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSVINFOSV_<ContainerAllocator> & m) {
    (void)m;
    return 12;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::NavSVINFOSV_<ContainerAllocator> & m) {
    stream.next(m.chn);
    stream.next(m.svid);
    stream.next(m.flags);
    stream.next(m.quality);
    stream.next(m.cno);
    stream.next(m.elev);
    stream.next(m.azim);
    stream.next(m.pr_res);
  }
};

///
/// @brief Serializes the NavSVINFO message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavSVINFO_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavSVINFO_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.num_ch);
    stream.next(m.global_flags);
    stream.next(m.reserved2);
    m.sv.resize(m.num_ch);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavSVINFO_<ContainerAllocator> &m) {
    return 8 + 12 * m.num_ch;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavSVINFO_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_ch) {
      // ROS_ERROR("NavSVINFO num_ch must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(static_cast<typename ublox_msgs::msg::NavSVINFO_<ContainerAllocator>::_num_ch_type>(m.sv.size()));
    stream.next(m.global_flags);
    stream.next(m.reserved2);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavVELNED_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavVELNED_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.speed);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.c_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavVELNED_<ContainerAllocator> & m) {
    (void)m;
    return 36;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavVELNED_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.vel_n);
    stream.next(m.vel_e);
    stream.next(m.vel_d);
    stream.next(m.speed);
    stream.next(m.g_speed);
    stream.next(m.heading);
    stream.next(m.s_acc);
    stream.next(m.c_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavTIMEGPS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavTIMEGPS_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.f_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(m.valid);
    stream.next(m.t_acc);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavTIMEGPS_<ContainerAllocator> & m) {
    (void)m;
    return 16;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavTIMEGPS_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.f_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(m.valid);
    stream.next(m.t_acc);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::NavTIMEUTC_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::NavTIMEUTC_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::NavTIMEUTC_<ContainerAllocator> & m) {
    (void)m;
    return 20;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::NavTIMEUTC_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.t_acc);
    stream.next(m.nano);
    stream.next(m.year);
    stream.next(m.month);
    stream.next(m.day);
    stream.next(m.hour);
    stream.next(m.min);
    stream.next(m.sec);
    stream.next(m.valid);
  }
};
  
///
/// @brief Serializes the RxmALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmALM_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmALM_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename ublox_msgs::msg::RxmALM_<ContainerAllocator>::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmALM_<ContainerAllocator> &m) {
    return 8 + (4 * m.dwrd.size());
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmALM_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      serialize(stream, m.dwrd[i]);
    }
  }
};

///
/// @brief Serializes the RxmEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmEPH_<ContainerAllocator> >
{
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmEPH_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename ublox_msgs::msg::RxmEPH_<ContainerAllocator>::_sf1d_type::value_type temp1;
      typename ublox_msgs::msg::RxmEPH_<ContainerAllocator>::_sf2d_type::value_type temp2;
      typename ublox_msgs::msg::RxmEPH_<ContainerAllocator>::_sf3d_type::value_type temp3;

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

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmEPH_<ContainerAllocator> &m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmEPH_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for (std::size_t i = 0; i < m.sf1d.size(); ++i) {
      serialize(stream, m.sf1d[i]);
    }
    for (std::size_t i = 0; i < m.sf2d.size(); ++i) {
      serialize(stream, m.sf2d[i]);
    }
    for (std::size_t i = 0; i < m.sf3d.size(); ++i) {
      serialize(stream, m.sf3d[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmRAWSV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::RxmRAWSV_<ContainerAllocator> & m) {
    stream.next(m.cp_mes);
    stream.next(m.pr_mes);
    stream.next(m.do_mes);
    stream.next(m.sv);
    stream.next(m.mes_qi);
    stream.next(m.cno);
    stream.next(m.lli);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmRAWSV_<ContainerAllocator> & m) {
    (void)m;
    return 24;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::RxmRAWSV_<ContainerAllocator> & m) {
    stream.next(m.cp_mes);
    stream.next(m.pr_mes);
    stream.next(m.do_mes);
    stream.next(m.sv);
    stream.next(m.mes_qi);
    stream.next(m.cno);
    stream.next(m.lli);
  }
};

///
/// @brief Serializes the RxmRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmRAW_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmRAW_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.num_sv);
    stream.next(m.reserved1);
    m.sv.resize(m.num_sv);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmRAW_<ContainerAllocator> &m) {
    return 8 + 24 * m.num_sv;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmRAW_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_sv) {
      // ROS_ERROR("RxmRAW num_sv must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(static_cast<typename ublox_msgs::msg::RxmRAW_<ContainerAllocator>::_num_sv_type>(m.sv.size()));
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmRAWXMeas_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::RxmRAWXMeas_<ContainerAllocator> & m) {
    stream.next(m.pr_mes);
    stream.next(m.cp_mes);
    stream.next(m.do_mes);
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.reserved0);
    stream.next(m.freq_id);
    stream.next(m.locktime);
    stream.next(m.cno);
    stream.next(m.pr_stdev);
    stream.next(m.cp_stdev);
    stream.next(m.do_stdev);
    stream.next(m.trk_stat);
    stream.next(m.reserved1);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmRAWXMeas_<ContainerAllocator> & m) {
    (void)m;
    return 32;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::RxmRAWXMeas_<ContainerAllocator> & m) {
    stream.next(m.pr_mes);
    stream.next(m.cp_mes);
    stream.next(m.do_mes);
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.reserved0);
    stream.next(m.freq_id);
    stream.next(m.locktime);
    stream.next(m.cno);
    stream.next(m.pr_stdev);
    stream.next(m.cp_stdev);
    stream.next(m.do_stdev);
    stream.next(m.trk_stat);
    stream.next(m.reserved1);
  }
};

///
/// @brief Serializes the RxmRAWX message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmRAWX_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmRAWX_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(m.num_meas);
    stream.next(m.rec_stat);
    stream.next(m.version);
    stream.next(m.reserved1);
    m.meas.resize(m.num_meas);
    for (std::size_t i = 0; i < m.meas.size(); ++i) {
      deserialize(stream, m.meas[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmRAWX_<ContainerAllocator> &m) {
    return 16 + 32 * m.num_meas;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmRAWX_<ContainerAllocator> &m) {
    if (m.meas.size() != m.num_meas) {
      // ROS_ERROR("RxmRAWX num_meas must equal meas size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.rcv_tow);
    stream.next(m.week);
    stream.next(m.leap_s);
    stream.next(static_cast<typename ublox_msgs::msg::RxmRAWX_<ContainerAllocator>::_num_meas_type>(m.meas.size()));
    stream.next(m.rec_stat);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.meas.size(); ++i) {
      serialize(stream, m.meas[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmRTCM_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmRTCM_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.version);
    stream.next(m.flags);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.ref_station);
    stream.next(m.msg_type);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmRTCM_<ContainerAllocator> & m) {
    (void)m;
    return 8;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmRTCM_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.version);
    stream.next(m.flags);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.ref_station);
    stream.next(m.msg_type);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmSFRB_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmSFRB_<ContainerAllocator> & m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.chn);
    stream.next(m.svid);
    stream.next(m.dwrd[0]);
    stream.next(m.dwrd[1]);
    stream.next(m.dwrd[2]);
    stream.next(m.dwrd[3]);
    stream.next(m.dwrd[4]);
    stream.next(m.dwrd[5]);
    stream.next(m.dwrd[6]);
    stream.next(m.dwrd[7]);
    stream.next(m.dwrd[8]);
    stream.next(m.dwrd[9]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmSFRB_<ContainerAllocator> & m) {
    (void)m;
    return 42;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmSFRB_<ContainerAllocator> & m) {
    UbloxOStream stream(data, size);
    stream.next(m.chn);
    stream.next(m.svid);
    stream.next(m.dwrd[0]);
    stream.next(m.dwrd[1]);
    stream.next(m.dwrd[2]);
    stream.next(m.dwrd[3]);
    stream.next(m.dwrd[4]);
    stream.next(m.dwrd[5]);
    stream.next(m.dwrd[6]);
    stream.next(m.dwrd[7]);
    stream.next(m.dwrd[8]);
    stream.next(m.dwrd[9]);
  }
};

///
/// @brief Serializes the RxmSFRBX message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmSFRBX_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmSFRBX_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
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
      deserialize(stream, m.dwrd[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmSFRBX_<ContainerAllocator> &m) {
    return 8 + 4 * m.num_words;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmSFRBX_<ContainerAllocator> &m) {
    if (m.dwrd.size() != m.num_words) {
      // ROS_ERROR("RxmSFRBX num_words must equal dwrd size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.gnss_id);
    stream.next(m.sv_id);
    stream.next(m.reserved0);
    stream.next(m.freq_id);
    stream.next(static_cast<typename ublox_msgs::msg::RxmSFRBX_<ContainerAllocator>::_num_words_type>(m.dwrd.size()));
    stream.next(m.chn);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) {
      serialize(stream, m.dwrd[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmSVSISV_<ContainerAllocator> > {
  inline static void read(UbloxIStream& stream, ublox_msgs::msg::RxmSVSISV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.sv_flag);
    stream.next(m.azim);
    stream.next(m.elev);
    stream.next(m.age);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmSVSISV_<ContainerAllocator> & m) {
    (void)m;
    return 6;
  }

  inline static void write(UbloxOStream& stream, const ublox_msgs::msg::RxmSVSISV_<ContainerAllocator> & m) {
    stream.next(m.svid);
    stream.next(m.sv_flag);
    stream.next(m.azim);
    stream.next(m.elev);
    stream.next(m.age);
  }
};

///
/// @brief Serializes the RxmSVSI message which has a repeated block.
///
template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::RxmSVSI_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::RxmSVSI_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.i_tow);
    stream.next(m.week);
    stream.next(m.num_vis);
    stream.next(m.num_sv);
    m.sv.resize(m.num_sv);
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      deserialize(stream, m.sv[i]);
    }
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::RxmSVSI_<ContainerAllocator> &m) {
    return 8 + 6 * m.num_sv;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::RxmSVSI_<ContainerAllocator> &m) {
    if (m.sv.size() != m.num_sv) {
      // ROS_ERROR("RxmSVSI num_sv must equal sv size");
    }
    UbloxOStream stream(data, size);
    stream.next(m.i_tow);
    stream.next(m.week);
    stream.next(m.num_vis);
    stream.next(static_cast<typename ublox_msgs::msg::RxmSVSI_<ContainerAllocator>::_num_sv_type>(m.sv.size()));
    for (std::size_t i = 0; i < m.sv.size(); ++i) {
      serialize(stream, m.sv[i]);
    }
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::TimTM2_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::TimTM2_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.ch);
    stream.next(m.flags);
    stream.next(m.rising_edge_count);
    stream.next(m.wn_r);
    stream.next(m.wn_f);
    stream.next(m.tow_ms_r);
    stream.next(m.tow_sub_ms_r);
    stream.next(m.tow_ms_f);
    stream.next(m.tow_sub_ms_f);
    stream.next(m.acc_est);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::TimTM2_<ContainerAllocator> &m) {
    (void)m;
    return 28;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::TimTM2_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.ch);
    stream.next(m.flags);
    stream.next(m.rising_edge_count);
    stream.next(m.wn_r);
    stream.next(m.wn_f);
    stream.next(m.tow_ms_r);
    stream.next(m.tow_sub_ms_r);
    stream.next(m.tow_ms_f);
    stream.next(m.tow_sub_ms_f);
    stream.next(m.acc_est);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::UpdSOS_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::UpdSOS_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.cmd);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::UpdSOS_<ContainerAllocator> &m) {
    (void)m;
    return 4;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::UpdSOS_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.cmd);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
  }
};

template <typename ContainerAllocator>
struct UbloxSerializer<ublox_msgs::msg::UpdSOSAck_<ContainerAllocator> > {
  inline static void read(const uint8_t *data, uint32_t count,
                          ublox_msgs::msg::UpdSOSAck_<ContainerAllocator> &m) {
    UbloxIStream stream(const_cast<uint8_t *>(data), count);
    stream.next(m.cmd);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.response);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
  }

  inline static uint32_t serializedLength(const ublox_msgs::msg::UpdSOSAck_<ContainerAllocator> &m) {
    (void)m;
    return 8;
  }

  inline static void write(uint8_t *data, uint32_t size,
                           const ublox_msgs::msg::UpdSOSAck_<ContainerAllocator> &m) {
    UbloxOStream stream(data, size);
    stream.next(m.cmd);
    stream.next(m.reserved0[0]);
    stream.next(m.reserved0[1]);
    stream.next(m.reserved0[2]);
    stream.next(m.response);
    stream.next(m.reserved1[0]);
    stream.next(m.reserved1[1]);
    stream.next(m.reserved1[2]);
  }
};

}  // namespace ublox

#endif  // UBLOX_MSGS_SERIALIZATION_HPP
