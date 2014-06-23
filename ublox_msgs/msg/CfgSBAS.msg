# CFG-SBAS (0x06 0x24)
# SBAS Configuration
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 22

uint8 mode              # SBAS Mode
uint8 MODE_ENABLED = 1    # SBAS Enabled (1) / Disabled (0)
uint8 MODE_TEST = 2       # SBAS Testbed: Use data anyhow (1) / Ignore data when in Test Mode (SBAS Msg 0)

uint8 usage             # SBAS Usage
uint8 USAGE_RANGE = 1     # Use SBAS GEOs as a ranging source (for navigation)
uint8 USAGE_DIFFCORR = 2  # Use SBAS Differential Corrections
uint8 USAGE_INTEGRITY = 4 # Use SBAS Integrity Information

uint8 maxSBAS           # Maximum Number of SBAS prioritized tracking
                        # channels (valid range: 0 - 3) to use


uint8 scanmode2         # Continuation of scanmode bitmask below
uint32 scanmode1        # Which SBAS PRN numbers to search for (Bitmask)
                        # If all Bits are set to zero, auto-scan (i.e. all valid
                        # PRNs) are searched. Every bit corresponds to a PRN number.
