# NAV-HPPOSECEF (0x01 0x13)
# High Precision Position Solution in ECEF
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 19

uint8 version
uint8[3] reserved0

uint32 iTOW             # GPS Millisecond Time of Week [ms]

int32 ecefX             # ECEF X coordinate [cm]
int32 ecefY             # ECEF Y coordinate [cm]
int32 ecefZ             # ECEF Z coordinate [cm]

int8 ecefXHp            # ECEF X high precision component [0.1mm]
int8 ecefYHp            # ECEF Y high precision component [0.1mm]
int8 ecefZHp            # ECEF Z high precision component [0.1mm]
uint8 flags

uint32 pAcc             # Position Accuracy Estimate [0.1mm]
