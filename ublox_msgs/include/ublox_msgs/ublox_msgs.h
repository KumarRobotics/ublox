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

#ifndef UBLOX_MSGS_H
#define UBLOX_MSGS_H

#include <ublox_msgs/NavCLOCK.h>
#include <ublox_msgs/NavDGPS.h>
#include <ublox_msgs/NavDOP.h>
#include <ublox_msgs/NavPOSECEF.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavSBAS.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavSVINFO.h>
#include <ublox_msgs/NavTIMEGPS.h>
#include <ublox_msgs/NavTIMEUTC.h>
#include <ublox_msgs/NavVELECEF.h>
#include <ublox_msgs/NavVELNED.h>

#include <ublox_msgs/RxmALM.h>
#include <ublox_msgs/RxmEPH.h>
#include <ublox_msgs/RxmRAW.h>
#include <ublox_msgs/RxmRAW_SV.h>
#include <ublox_msgs/RxmSFRB.h>
#include <ublox_msgs/RxmSVSI.h>

#include <ublox_msgs/AidALM.h>
#include <ublox_msgs/AidEPH.h>
#include <ublox_msgs/AidHUI.h>

#include <ublox_msgs/CfgANT.h>
#include <ublox_msgs/CfgCFG.h>
#include <ublox_msgs/CfgMSG.h>
#include <ublox_msgs/CfgNAV5.h>
#include <ublox_msgs/CfgNAVX5.h>
#include <ublox_msgs/CfgPRT.h>
#include <ublox_msgs/CfgRATE.h>
#include <ublox_msgs/CfgSBAS.h>
#include <ublox_msgs/CfgGNSS.h>

#include <ublox_msgs/MonVER.h>

namespace ublox_msgs {

namespace Class {
  static const uint8_t NAV = 0x01; // Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
  static const uint8_t RXM = 0x02; // Receiver Manager Messages: Satellite Status, RTC Status
  static const uint8_t INF = 0x04; // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
  static const uint8_t ACK = 0x05; // Ack/Nack Messages: as replies to CFG Input Messages
  static const uint8_t CFG = 0x06; // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
  static const uint8_t MON = 0x0A; // Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
  static const uint8_t AID = 0x0B; // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
  static const uint8_t TIM = 0x0D; // Timing Messages: Timepulse Output, Timemark Results
  static const uint8_t ESF = 0x10; // External Sensor Fusion Messages: External sensor measurements and status information
}

namespace Message {
  namespace NAV {
    static const uint8_t CLOCK = NavCLOCK::MESSAGE_ID;
    static const uint8_t DGPS = NavDGPS::MESSAGE_ID;
    static const uint8_t DOP = NavDOP::MESSAGE_ID;
    static const uint8_t POSECEF = NavPOSECEF::MESSAGE_ID;
    static const uint8_t POSLLH = NavPOSLLH::MESSAGE_ID;
    static const uint8_t SBAS = NavSBAS::MESSAGE_ID;
    static const uint8_t SOL = NavSOL::MESSAGE_ID;
    static const uint8_t STATUS = NavSTATUS::MESSAGE_ID;
    static const uint8_t SVINFO = NavSVINFO::MESSAGE_ID;
    static const uint8_t TIMEGPS = NavTIMEGPS::MESSAGE_ID;
    static const uint8_t TIMEUTC = NavTIMEUTC::MESSAGE_ID;
    static const uint8_t VELECEF = NavVELECEF::MESSAGE_ID;
    static const uint8_t VELNED = NavVELNED::MESSAGE_ID;
  }

  namespace RXM {
    static const uint8_t ALM = RxmALM::MESSAGE_ID;
    static const uint8_t EPH = RxmEPH::MESSAGE_ID;
    static const uint8_t RAW = RxmRAW::MESSAGE_ID;
    static const uint8_t SFRB = RxmSFRB::MESSAGE_ID;
    static const uint8_t SVSI = RxmSVSI::MESSAGE_ID;
  }

  namespace AID {
    static const uint8_t ALM = AidALM::MESSAGE_ID;
    static const uint8_t EPH = AidEPH::MESSAGE_ID;
    static const uint8_t HUI = AidHUI::MESSAGE_ID;
  }

  namespace CFG {
    static const uint8_t ANT   = CfgANT::MESSAGE_ID;
    static const uint8_t CFG   = CfgCFG::MESSAGE_ID;
    static const uint8_t MSG   = CfgMSG::MESSAGE_ID;
    static const uint8_t NAV5  = CfgNAV5::MESSAGE_ID;
    static const uint8_t NAVX5 = CfgNAVX5::MESSAGE_ID;
    static const uint8_t PRT   = CfgPRT::MESSAGE_ID;
    static const uint8_t RATE  = CfgRATE::MESSAGE_ID;
    static const uint8_t SBAS  = CfgSBAS::MESSAGE_ID;
    static const uint8_t GNSS  = CfgGNSS::MESSAGE_ID;
  }
  
  namespace MON {
    static const uint8_t VER   = MonVER::MESSAGE_ID;
  }
}

} // namespace ublox_msgs

#endif // UBLOX_MSGS_H
