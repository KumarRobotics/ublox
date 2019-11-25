# NAV-DGPS (0x01 0x31)
# DGPS Data Used for NAV
#
# This message outputs the Correction data as it has been applied to the current
# NAV Solution. See also the notes on the RTCM protocol.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 49

uint32 i_tow             # GPS Millisecond time of week [ms]

int32 age               # Age of newest correction data [ms]
int16 base_id            # DGPS Base Station ID
int16 base_health        # DGPS Base Station Health Status
int8 num_ch              # Number of channels for which correction data is
                        # following

uint8 status            # DGPS Correction Type Status
uint8 DGPS_CORRECTION_NONE = 0
uint8 DGPS_CORRECTION_PR_PRR = 1

uint16 reserved1        # Reserved

NavDGPSSV[] sv
