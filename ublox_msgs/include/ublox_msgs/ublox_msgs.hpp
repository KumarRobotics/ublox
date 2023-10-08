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

#ifndef UBLOX_MSGS_UBLOX_MSGS_HPP
#define UBLOX_MSGS_UBLOX_MSGS_HPP

#include <ublox_msgs/msg/nav_att.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>
#include <ublox_msgs/msg/nav_cov.hpp>
#include <ublox_msgs/msg/nav_dgps.hpp>
#include <ublox_msgs/msg/nav_dop.hpp>
#include <ublox_msgs/msg/nav_posecef.hpp>
#include <ublox_msgs/msg/nav_posllh.hpp>
#include <ublox_msgs/msg/nav_relposned.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>
#include <ublox_msgs/msg/nav_sbas.hpp>
#include <ublox_msgs/msg/nav_sol.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_pvt7.hpp>
#include <ublox_msgs/msg/nav_status.hpp>
#include <ublox_msgs/msg/nav_sat.hpp>
#include <ublox_msgs/msg/nav_svin.hpp>
#include <ublox_msgs/msg/nav_svinfo.hpp>
#include <ublox_msgs/msg/nav_timegps.hpp>
#include <ublox_msgs/msg/nav_timeutc.hpp>
#include <ublox_msgs/msg/nav_velecef.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>

#include <ublox_msgs/msg/rxm_alm.hpp>
#include <ublox_msgs/msg/rxm_eph.hpp>
#include <ublox_msgs/msg/rxm_raw.hpp>
#include <ublox_msgs/msg/rxm_rawsv.hpp>
#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <ublox_msgs/msg/rxm_rawx_meas.hpp>
#include <ublox_msgs/msg/rxm_rtcm.hpp>
#include <ublox_msgs/msg/rxm_sfrb.hpp>
#include <ublox_msgs/msg/rxm_sfrbx.hpp>
#include <ublox_msgs/msg/rxm_svsi.hpp>

#include <ublox_msgs/msg/inf.hpp>
#include <ublox_msgs/msg/ack.hpp>

#include <ublox_msgs/msg/cfg_ant.hpp>
#include <ublox_msgs/msg/cfg_cfg.hpp>
#include <ublox_msgs/msg/cfg_dat.hpp>
#include <ublox_msgs/msg/cfg_dgnss.hpp>
#include <ublox_msgs/msg/cfg_gnss.hpp>
#include <ublox_msgs/msg/cfg_hnr.hpp>
#include <ublox_msgs/msg/cfg_inf.hpp>
#include <ublox_msgs/msg/cfg_inf_block.hpp>
#include <ublox_msgs/msg/cfg_msg.hpp>
#include <ublox_msgs/msg/cfg_nav5.hpp>
#include <ublox_msgs/msg/cfg_navx5.hpp>
#include <ublox_msgs/msg/cfg_nmea.hpp>
#include <ublox_msgs/msg/cfg_nmea6.hpp>
#include <ublox_msgs/msg/cfg_nmea7.hpp>
#include <ublox_msgs/msg/cfg_prt.hpp>
#include <ublox_msgs/msg/cfg_rate.hpp>
#include <ublox_msgs/msg/cfg_rst.hpp>
#include <ublox_msgs/msg/cfg_sbas.hpp>
#include <ublox_msgs/msg/cfg_tmode3.hpp>
#include <ublox_msgs/msg/cfg_usb.hpp>
#include <ublox_msgs/msg/cfg_valdel.hpp>
#include <ublox_msgs/msg/cfg_valget.hpp>
#include <ublox_msgs/msg/cfg_valset.hpp>
#include <ublox_msgs/msg/cfg_valset_cfgdata.hpp>

#include <ublox_msgs/msg/upd_sos.hpp>
#include <ublox_msgs/msg/upd_sos_ack.hpp>

#include <ublox_msgs/msg/mon_gnss.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/mon_hw6.hpp>
#include <ublox_msgs/msg/mon_ver.hpp>

#include <ublox_msgs/msg/aid_alm.hpp>
#include <ublox_msgs/msg/aid_eph.hpp>
#include <ublox_msgs/msg/aid_hui.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/esf_status_sens.hpp>

#include <ublox_msgs/msg/mga_gal.hpp>

#include <ublox_msgs/msg/hnr_pvt.hpp>

#include <ublox_msgs/msg/tim_tm2.hpp>

namespace ublox_msgs {

namespace Class {
  static const uint8_t NAV = 0x01; //!< Navigation Result Messages: Position,
                                   //!< Speed, Time, Acceleration, Heading,
                                   //!< DOP, SVs used
  static const uint8_t RXM = 0x02; //!< Receiver Manager Messages:
                                   //!< Satellite Status, RTC Status
  static const uint8_t INF = 0x04; //!< Information Messages:
                                   //!< Printf-Style Messages, with IDs such as
                                   //!< Error, Warning, Notice
  static const uint8_t ACK = 0x05; //!< Ack/Nack Messages: Acknowledge or Reject
                                   //!< messages to CFG input messages
  static const uint8_t CFG = 0x06; //!< Configuration Input Messages: Set
                                   //!< Dynamic Model, Set DOP Mask, Set Baud
                                   //!< Rate, etc.
  static const uint8_t UPD = 0x09; //!< Firmware Update Messages: i.e.
                                   //!< Memory/Flash erase/write, Reboot, Flash
                                   //!< identification, etc.
                                   //!< Used to update the firmware and identify
                                   //!< any attached flash device
  static const uint8_t MON = 0x0A; //!< Monitoring Messages: Communication
                                   //!< Status, CPU Load, Stack Usage,
                                   //!< Task Status
  static const uint8_t AID = 0x0B; //!< AssistNow Aiding Messages: Ephemeris,
                                   //!< Almanac, other A-GPS data input
  static const uint8_t TIM = 0x0D; //!< Timing Messages: Timepulse Output,
                                   //!< Timemark Results
  static const uint8_t ESF = 0x10; //!< External Sensor Fusion Messages:
                                   //!< External sensor measurements and status
                                   //!< information
  static const uint8_t MGA = 0x13; //!< Multiple GNSS Assistance Messages:
                                   //!< Assistance data for various GNSS
  static const uint8_t LOG = 0x21; //!< Logging Messages: Log creation,
                                   //!< deletion, info and retrieval
  static const uint8_t SEC = 0x27; //!< Security Feature Messages
  static const uint8_t HNR = 0x28; //!< High Rate Navigation Results Messages:
                                   //!< High rate time, position, speed, heading
  static const uint8_t RTCM = 0xF5; //!< RTCM Configuration Messages
}  // namespace Class

namespace Message {
  namespace NAV {
    static const uint8_t ATT = ublox_msgs::msg::NavATT::MESSAGE_ID;
    static const uint8_t CLOCK = ublox_msgs::msg::NavCLOCK::MESSAGE_ID;
    static const uint8_t COV = ublox_msgs::msg::NavCOV::MESSAGE_ID;
    static const uint8_t DGPS = ublox_msgs::msg::NavDGPS::MESSAGE_ID;
    static const uint8_t DOP = ublox_msgs::msg::NavDOP::MESSAGE_ID;
    static const uint8_t POSECEF = ublox_msgs::msg::NavPOSECEF::MESSAGE_ID;
    static const uint8_t POSLLH = ublox_msgs::msg::NavPOSLLH::MESSAGE_ID;
    static const uint8_t RELPOSNED = ublox_msgs::msg::NavRELPOSNED::MESSAGE_ID;
    static const uint8_t RELPOSNED9 = ublox_msgs::msg::NavRELPOSNED9::MESSAGE_ID;
    static const uint8_t SBAS = ublox_msgs::msg::NavSBAS::MESSAGE_ID;
    static const uint8_t SOL = ublox_msgs::msg::NavSOL::MESSAGE_ID;
    static const uint8_t PVT = ublox_msgs::msg::NavPVT::MESSAGE_ID;
    static const uint8_t SAT = ublox_msgs::msg::NavSAT::MESSAGE_ID;
    static const uint8_t STATUS = ublox_msgs::msg::NavSTATUS::MESSAGE_ID;
    static const uint8_t SVINFO = ublox_msgs::msg::NavSVINFO::MESSAGE_ID;
    static const uint8_t SVIN = ublox_msgs::msg::NavSVIN::MESSAGE_ID;
    static const uint8_t TIMEGPS = ublox_msgs::msg::NavTIMEGPS::MESSAGE_ID;
    static const uint8_t TIMEUTC = ublox_msgs::msg::NavTIMEUTC::MESSAGE_ID;
    static const uint8_t VELECEF = ublox_msgs::msg::NavVELECEF::MESSAGE_ID;
    static const uint8_t VELNED = ublox_msgs::msg::NavVELNED::MESSAGE_ID;
  }  // namespace NAV

  namespace RXM {
    static const uint8_t ALM = ublox_msgs::msg::RxmALM::MESSAGE_ID;
    static const uint8_t EPH = ublox_msgs::msg::RxmEPH::MESSAGE_ID;
    static const uint8_t RAW = ublox_msgs::msg::RxmRAW::MESSAGE_ID;
    static const uint8_t RAWX = ublox_msgs::msg::RxmRAWX::MESSAGE_ID;
    static const uint8_t RTCM = ublox_msgs::msg::RxmRTCM::MESSAGE_ID;
    static const uint8_t SFRB = ublox_msgs::msg::RxmSFRB::MESSAGE_ID;
    static const uint8_t SFRBX = ublox_msgs::msg::RxmSFRBX::MESSAGE_ID;
    static const uint8_t SVSI = ublox_msgs::msg::RxmSVSI::MESSAGE_ID;
  }  // namespace RXM

  namespace INF {
    static const uint8_t ERROR = 0x00;
    static const uint8_t WARNING = 0x01;
    static const uint8_t NOTICE = 0x02;
    static const uint8_t TEST = 0x03;
    static const uint8_t DEBUG = 0x04;
  }  // namespace INF

  namespace ACK {
    static const uint8_t NACK = 0x00;
    static const uint8_t ACK = 0x01;
  }  // namespace ACK

  namespace AID {
    static const uint8_t ALM = ublox_msgs::msg::AidALM::MESSAGE_ID;
    static const uint8_t EPH = ublox_msgs::msg::AidEPH::MESSAGE_ID;
    static const uint8_t HUI = ublox_msgs::msg::AidHUI::MESSAGE_ID;
  }  // namespace AID

  namespace CFG {
    static const uint8_t ANT = ublox_msgs::msg::CfgANT::MESSAGE_ID;
    static const uint8_t CFG = ublox_msgs::msg::CfgCFG::MESSAGE_ID;
    static const uint8_t DAT = ublox_msgs::msg::CfgDAT::MESSAGE_ID;
    static const uint8_t GNSS = ublox_msgs::msg::CfgGNSS::MESSAGE_ID;
    static const uint8_t HNR = ublox_msgs::msg::CfgHNR::MESSAGE_ID;
    static const uint8_t INF = ublox_msgs::msg::CfgINF::MESSAGE_ID;
    static const uint8_t DGNSS = ublox_msgs::msg::CfgDGNSS::MESSAGE_ID;
    static const uint8_t MSG = ublox_msgs::msg::CfgMSG::MESSAGE_ID;
    static const uint8_t NAV5 = ublox_msgs::msg::CfgNAV5::MESSAGE_ID;
    static const uint8_t NAVX5 = ublox_msgs::msg::CfgNAVX5::MESSAGE_ID;
    static const uint8_t NMEA = ublox_msgs::msg::CfgNMEA::MESSAGE_ID;
    static const uint8_t PRT = ublox_msgs::msg::CfgPRT::MESSAGE_ID;
    static const uint8_t RATE = ublox_msgs::msg::CfgRATE::MESSAGE_ID;
    static const uint8_t RST = ublox_msgs::msg::CfgRST::MESSAGE_ID;
    static const uint8_t SBAS = ublox_msgs::msg::CfgSBAS::MESSAGE_ID;
    static const uint8_t TMODE3 = ublox_msgs::msg::CfgTMODE3::MESSAGE_ID;
    static const uint8_t USB = ublox_msgs::msg::CfgUSB::MESSAGE_ID;
    static const uint8_t VALDEL = ublox_msgs::msg::CfgVALDEL::MESSAGE_ID;
    static const uint8_t VALGET = ublox_msgs::msg::CfgVALGET::MESSAGE_ID;
    static const uint8_t VALSET = ublox_msgs::msg::CfgVALSET::MESSAGE_ID;
  }  // namespace CFG

  namespace UPD {
    //! SOS and SOS_Ack have the same message ID, but different lengths
    static const uint8_t SOS = ublox_msgs::msg::UpdSOS::MESSAGE_ID;
  }  // namespace UPD

  namespace MON {
    static const uint8_t GNSS = ublox_msgs::msg::MonGNSS::MESSAGE_ID;
    static const uint8_t HW = ublox_msgs::msg::MonHW::MESSAGE_ID;
    static const uint8_t VER = ublox_msgs::msg::MonVER::MESSAGE_ID;
  }  // namespace MON

  namespace ESF {
    static const uint8_t INS = ublox_msgs::msg::EsfINS::MESSAGE_ID;
    static const uint8_t MEAS = ublox_msgs::msg::EsfMEAS::MESSAGE_ID;
    static const uint8_t RAW = ublox_msgs::msg::EsfRAW::MESSAGE_ID;
    static const uint8_t STATUS = ublox_msgs::msg::EsfSTATUS::MESSAGE_ID;
  }  // namespace ESF

  namespace MGA {
    static const uint8_t GAL = ublox_msgs::msg::MgaGAL::MESSAGE_ID;
  }  // namespace MGA

  namespace HNR {
    static const uint8_t PVT = ublox_msgs::msg::HnrPVT::MESSAGE_ID;
  }  // namespace HNR

  namespace TIM {
    static const uint8_t TM2 = ublox_msgs::msg::TimTM2::MESSAGE_ID;
  }  // namespace TIM
}  // namespace Message

}  // namespace ublox_msgs

#endif //!< UBLOX_MSGS_UBLOX_MSGS_HPP
