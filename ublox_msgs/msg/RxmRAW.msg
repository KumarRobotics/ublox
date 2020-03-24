# RXM-RAW (0x02 0x10)
# Raw Measurement Data
#
# Supported up to ublox 7 firmware. See RxmRAWX for ublox 8
# This message contains all information needed to be able to generate a RINEX
# observation file.
# This message outputs pseudorange, doppler and carrier phase measurements for
# GPS satellites once signals have been synchronised. No other GNSS types are
# currently supported.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 16

int32 rcv_tow            # Measurement time of week in receiver local time [s]
int16 week              # Measurement week number in receiver local time [weeks]

uint8 num_sv             # # of satellites following
uint8 reserved1         # Reserved

RxmRAWSV[] sv          # numSV times
