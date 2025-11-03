# NAV-HPPOSECEF (0x01 0x13)
# High Precision Position Solution in ECEF
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 0x01
uint8 MESSAGE_ID = 0x13

uint8 version
uint8[3] reserved0

uint32 i_tow            # GPS Millisecond Time of Week [ms]

int32 ecef_x            # ECEF X coordinate [cm]
int32 ecef_y            # ECEF Y coordinate [cm]
int32 ecef_z            # ECEF Z coordinate [cm]

int8 ecef_x_hp          # ECEF X high precision component [0.1mm]
int8 ecef_y_hp          # ECEF Y high precision component [0.1mm]
int8 ecef_z_hp          # ECEF Z high precision component [0.1mm]
uint8 flags

uint32 p_acc            # Position Accuracy Estimate [0.1mm]
