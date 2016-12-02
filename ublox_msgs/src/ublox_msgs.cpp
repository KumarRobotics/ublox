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

#include <ublox/serialization/ublox_msgs.h>

template <typename T>
std::vector<std::pair<uint8_t,uint8_t> > ublox::Message<T>::keys_;

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::CLOCK, ublox_msgs, NavCLOCK);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::DGPS, ublox_msgs, NavDGPS);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::DOP, ublox_msgs, NavDOP);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::POSECEF, ublox_msgs, NavPOSECEF);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::POSLLH, ublox_msgs, NavPOSLLH);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SBAS, ublox_msgs, NavSBAS);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SOL, ublox_msgs, NavSOL);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::STATUS, ublox_msgs, NavSTATUS);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SVINFO, ublox_msgs, NavSVINFO);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::TIMEGPS, ublox_msgs, NavTIMEGPS);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::TIMEUTC, ublox_msgs, NavTIMEUTC);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::VELECEF, ublox_msgs, NavVELECEF);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::VELNED, ublox_msgs, NavVELNED);

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::ALM, ublox_msgs, RxmALM);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::EPH, ublox_msgs, RxmEPH);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::RAW, ublox_msgs, RxmRAW);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::RAWX, ublox_msgs, RxmRAWX);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::SFRB, ublox_msgs, RxmSFRB);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::SVSI, ublox_msgs, RxmSVSI);

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, ublox_msgs, AidALM);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, ublox_msgs, AidEPH);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI, ublox_msgs, AidHUI);

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::ANT, ublox_msgs, CfgANT);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::CFG, ublox_msgs, CfgCFG);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::MSG, ublox_msgs, CfgMSG);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NAV5, ublox_msgs, CfgNAV5);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NAVX5, ublox_msgs, CfgNAVX5);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::PRT, ublox_msgs, CfgPRT);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::RATE, ublox_msgs, CfgRATE);
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::GNSS, ublox_msgs, CfgGNSS);

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MON, ublox_msgs::Message::MON::VER, ublox_msgs, MonVER);
