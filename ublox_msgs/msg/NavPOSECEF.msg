# NAV-POSECEF (0x01 0x01)
# Position Solution in ECEF
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 1

uint32 i_tow             # GPS Millisecond Time of Week [ms]

int32 ecef_x             # ECEF X coordinate [cm]
int32 ecef_y             # ECEF Y coordinate [cm]
int32 ecef_z             # ECEF Z coordinate [cm]
uint32 p_acc             # Position Accuracy Estimate [cm]
