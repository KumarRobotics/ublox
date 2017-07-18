# NAV-SAT (0x01 0x35)
# Satellite information
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 53

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 version           # Message version
uint8 numSV             # Number of satellites

uint8[2] reserved1      # Reserved

NavSAT_SV[] sv
