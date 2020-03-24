# NAV-VELECEF (0x01 0x11)
# Velocity Solution in ECEF
#
# See important comments concerning validity of velocity given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 17

uint32 i_tow             # GPS Millisecond time of week [ms]

int32 ecef_vx            # ECEF X velocity [cm/s]
int32 ecef_vy            # ECEF Y velocity [cm/s]
int32 ecef_vz            # ECEF Z velocity [cm/s]
uint32 s_acc             # Speed Accuracy Estimate [cm/s]
