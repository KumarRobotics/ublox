# NAV-SOL (0x01 0x06)
# Navigation Solution Information
#
# This message combines Position, velocity and time solution in ECEF, including
# accuracy figures
# This message has only been retained for backwards compatibility; users are
# recommended to use the UBX-NAV-PVT message in preference.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 6

uint32 i_tow             # GPS Millisecond time of week [ms]
int32 f_tow              # Fractional Nanoseconds remainder of rounded
                        # ms above, range -500000 .. 500000 [ns]
int16 week              # GPS week (GPS time)

uint8 gps_fix            # GPSfix Type, range 0..5
uint8 GPS_NO_FIX = 0
uint8 GPS_DEAD_RECKONING_ONLY = 1
uint8 GPS_2D_FIX = 2
uint8 GPS_3D_FIX = 3
uint8 GPS_GPS_DEAD_RECKONING_COMBINED = 4
uint8 GPS_TIME_ONLY_FIX = 5

uint8 flags             # Fix Status Flags
uint8 FLAGS_GPS_FIX_OK = 1      # Fix within limits i.e. within DOP & ACC Masks
uint8 FLAGS_DIFF_SOLN = 2       # DGPS used
uint8 FLAGS_WKNSET = 4          # Week Number valid
uint8 FLAGS_TOWSET = 8          # Time of Week valid

int32 ecef_x             # ECEF X coordinate [cm]
int32 ecef_y             # ECEF Y coordinate [cm]
int32 ecef_z             # ECEF Z coordinate [cm]
uint32 p_acc             # 3D Position Accuracy Estimate [cm]
int32 ecef_vx            # ECEF X velocity [cm/s]
int32 ecef_vy            # ECEF Y velocity [cm/s]
int32 ecef_vz            # ECEF Z velocity [cm/s]
uint32 s_acc             # Speed Accuracy Estimate [cm/s]
uint16 p_dop             # Position DOP [1 / 0.01]
uint8 reserved1         # Reserved
uint8 num_sv             # Number of SVs used in Nav Solution
uint32 reserved2        # Reserved
