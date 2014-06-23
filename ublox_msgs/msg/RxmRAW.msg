# RXM-RAW (0x02 0x10)
# Raw Measurement Data
#
# This message contains all information needed to be able to generate a RINEX file.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 16

int32 iTOW              # Measurement integer millisecond GPS time of week (Receiver Time) [ms]
int16 week              # Measurement GPS week number (Receiver Time) [weeks]

uint8 numSV             # # of satellites following
uint8 reserved1         # Reserved

RxmRAW_SV[] sv
