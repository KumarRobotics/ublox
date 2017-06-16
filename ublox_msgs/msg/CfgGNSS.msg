# CFG-GNSS (0x06 0x3E)
# GNSS Configuration
# Gets or sets the GNSS system channel sharing configuration.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 62

uint8 msgVer            # Message version (= 0 for this version)
uint8 numTrkChHw        # Number of tracking channels in hardware (read only)
uint8 numTrkChUse       # Number of tracking channels to use (<= numTrkChHw)
                        # If 0xFF, then number of tracking channels to use will 
                        # be set to numTrkChHw
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

uint32 flags            # Bitfield of flags
uint32 FLAGS_ENABLE = 1 
uint32 FLAGS_SIG_CFG_MASK = 16711680   # Signal configuration mask
                                       # When gnssId is 0 (GPS)
                                       # * 0x01 = GPS L1C/A
                                       # When gnssId is 1 (SBAS)
                                       # * 0x01 = SBAS L1C/A
                                       # When gnssId is 2 (Galileo)
                                       # * 0x01 = Galileo E1OS (not supported in 
                                       #   protocol versions less than 18)
                                       # When gnssId is 3 (BeiDou)
                                       # * 0x01 = BeiDou B1I
                                       # When gnssId is 4 (IMES)
                                       # * 0x01 = IMES L1
                                       # When gnssId is 5 (QZSS)
                                       # * 0x01 = QZSS L1C/A
                                       # * 0x04 = QZSS L1SAIF
                                       # When gnssId is 6 (GLONASS)
                                       # * 0x01 = GLONASS L1OF