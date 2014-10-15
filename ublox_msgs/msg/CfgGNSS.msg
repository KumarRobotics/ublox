# CFG-GNSS (0x06 0x3E)
# GNSS Configuration
# Gets or sets the GNSS system channel sharing configuration.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 62

uint8 msgVer            # Message version (= 0 for this version)
uint8 numTrkChHw        # Number of tracking channels in hardware (read only)
uint8 numTrkChUse       # Number of tracking channels to use (<= numTrkChHw)
uint8 numConfigBlocks   # Number of configuration blocks following (set to 1)

uint8 gnssId            # GNSS identifier (select from following list)

uint8 GNSS_ID_GPS = 0
uint8 GNSS_ID_SBAS = 1
uint8 GNSS_ID_GALILEO = 2
uint8 GNSS_ID_BEIDOU = 3
uint8 GNSS_ID_QZSS = 5
uint8 GNSS_ID_GLONASS = 6

uint8 resTrkCh          # Number of reserved (minimum) tracking channels for this GNSS system
uint8 maxTrkCh          # Maximum number of tracking channels (>=resTrkChn)
uint8 reserved1         # Reserved
uint32 flags            # Bitfield of flags (bit 0 = enable/disable)