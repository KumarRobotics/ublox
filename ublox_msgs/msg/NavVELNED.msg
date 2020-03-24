# NAV-VELNED (0x01 0x12)
# Velocity Solution in NED
#
# See important comments concerning validity of velocity given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 18

uint32 i_tow             # GPS Millisecond time of week [ms]

int32 vel_n              # NED north velocity [cm/s]
int32 vel_e              # NED east velocity [cm/s]
int32 vel_d              # NED down velocity [cm/s]
uint32 speed            # Speed (3-D) [cm/s]
uint32 g_speed           # Ground Speed (2-D) [cm/s]
int32 heading           # Heading of motion 2-D [deg / 1e-5]
uint32 s_acc             # Speed Accuracy Estimate [cm/s]
uint32 c_acc             # Course / Heading Accuracy Estimate [deg / 1e-5]
