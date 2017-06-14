# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# This message combines Position, velocity and time solution in LLH, 
# including accuracy figures

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 iTOW             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid             # Validity flags
uint8 VALID_DATE = 1            # Valid UTC Date
uint8 VALID_TIME = 2            # Valid 
uint8 VALID_FULLY_RESOLVED = 3  # UTC time of day has been fully resolved 
                                # (no seconds uncertainty)

uint32 tAcc             # time accuracy estimate [ns] (UTC)
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)

uint8 gpsFix            # GPS fix Type, range 0..5
uint8 GPS_NO_FIX = 0
uint8 GPS_DEAD_RECKONING_ONLY = 1
uint8 GPS_2D_FIX = 2                      # Signal from only 3 SVs, 
                                          # constant altitude assumed
uint8 GPS_3D_FIX = 3
uint8 GPS_GPS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning
uint8 GPS_TIME_ONLY_FIX = 5               # Time only fix (High precision 
                                          # devices)

uint8 flags             # Fix Status Flags
# Defines bit positions
uint8 FLAGS_GPS_FIX_OK = 1            # i.e. within DOP & accuracy masks
uint8 FLAGS_DIFF_SOLN = 2            # DGPS used
uint8 FLAGS_PSM_0 = 4               # Power Save Mode state flags
uint8 FLAGS_PSM_1 = 8
uint8 FLAGS_PSM_2 = 16
uint8 FLAGS_HEAD_VEH_VALID = 32     # heading of vehicle is valid
uint8 FLAGS_CARRIER_PHASE_FLOAT = 64    # carrier phase float solution (no fixed 
                                        # integer measurements have been used to calculate the solution)
uint8 FLAGS_CARRIER_PHASE_FIXED = 128   # carrier phase fixed solution
# Power save mode flags, flags bits 2-4
uint8 PSM_OFF = 0                       # PSM is off
uint8 PSM_ENABLED = 1                   # Enabled (an intermediate state before Acquisition state
uint8 PSM_ACQUIRED = 2                  # Acquisition
uint8 PSM_TRACKING = 3                  # Tracking
uint8 PSM_POWER_OPTIMIZED_TRACKING = 4  # Power Optimized Tracking
uint8 PSM_INACTIVE = 5                  # Inactive
# Carrier Phase Range solution flags
uint8 CARRIER_PHASE_NO_SOLUTION = 0 # no carrier phase range solution
uint8 CARRIER_PHASE_FLOAT = 1 # float solution 
uint8 CARRIER_PHASE_FIXED = 2 # fixed solution (>=1 fixed integer carrier phase range measurements have been used to calculate the
                              # solution)

uint8 flags2            # Additional Flags
uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of 
                                        # Day validity confirmation is available
uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed
uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed

uint8 numSV             # Number of SVs used in Nav Solution
int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 hMSL              # Height above mean sea level [mm]
uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]

int32 velN              # NED north velocity [mm/s]
int32 velE              # NED east velocity [mm/s]
int32 velD              # NED down velocity [mm/s]
uint32 gSpeed           # Ground Speed (2-D) [mm/s]
int32 heading           # Heading of motion 2-D [deg / 1e-5]
uint32 sAcc             # Speed Accuracy Estimate [mm/s]
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) 
                        # [deg / 1e-5]

uint16 pDOP             # Position DOP, scaled by 0.01
uint8[6] reserved1      # Reserved

int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]
int16 magDec            # Magnetic declination [deg / 1e-2]
uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]