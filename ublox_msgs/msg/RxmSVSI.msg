# RXM-SVSI (0x02 0x20)
# SV Status Info
#
# Status of the receiver manager knowledge about GPS Orbit Validity
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 32

int32 iTOW              # Measurement integer millisecond GPS time of week [ms]
int16 week              # Measurement GPS week number [weeks]

uint8 numVis            # Number of visible satellites
uint8 numSV             # Number of per-SV data blocks following

RxmSVSI_SV[] sv
