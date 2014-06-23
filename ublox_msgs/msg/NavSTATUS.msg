# NAV-STATUS (0x01 0x03)
# Receiver Navigation Status
#
# See important comments concerning validity of position and velocity given in
# section Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 3

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 gpsFix            # GPSfix Type, range 0..5
uint8 GPS_NO_FIX = 0
uint8 GPS_DEAD_RECKONING_ONLY = 1
uint8 GPS_2D_FIX = 2
uint8 GPS_3D_FIX = 3
uint8 GPS_GPS_DEAD_RECKONING_COMBINED = 4
uint8 GPS_TIME_ONLY_FIX = 6

uint8 flags             # Navigation Status Flags
uint8 FLAGS_GPSFIXOK = 1      # i.e. within DOP & ACC Masks
uint8 FLAGS_DIFFSOLN = 2      # DGPS used
uint8 FLAGS_WKNSET = 4        # Week Number valid
uint8 FLAGS_TOWSET = 8        # Time of Week valid

uint8 fixStat           # Fix Status Information
# DGPS Input Status:
uint8 DGPSISTAT_MASK = 1
uint8 DGPSISTAT_NONE = 0        # none
uint8 DGPSISTAT_PR_PRR = 1      # PR+PRR Correction
# map matching status:
uint8 MAPMATCHING_MASK = 192
uint8 MAPMATCHING_NONE = 0      # none
uint8 MAPMATCHING_VALID = 64    # valid, i.e. map matching data was received, but was too old
uint8 MAPMATCHING_USED = 128    # used, map matching data was applied
uint8 MAPMATCHING_DR = 192      # DR, map matching was the reason to enable the dead reckoning gpsFix type instead of publishing no fix

uint8 flags2            # further information about navigation output
# power safe mode state (Only for FW version >= 7.01; undefined otherwise)
uint8 PSMSTATE_ACQUISITION = 0                # ACQUISITION [or when psm disabled]
uint8 PSMSTATE_TRACKING = 1                   # TRACKING
uint8 PSMSTATE_POWER_OPTIMIZED_TRACKING = 2   # POWER OPTIMIZED TRACKING
uint8 PSMSTATE_INACTIVE = 3                   # INACTIVE

uint32 ttff             # Time to first fix (millisecond time tag)
uint32 msss             # Milliseconds since Startup / Reset
