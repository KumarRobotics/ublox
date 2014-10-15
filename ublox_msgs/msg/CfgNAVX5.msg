# CFG-NAVX5 (0x06 0x24)
# Navigation Engine Expert Settings
# Warning: Refer to u-blox protocol spec before changing these settings.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 35

uint16 version        # Message version (set to 0)

uint16 mask1          # First parameters bitmask (possible values below)
uint16 MASK_MIN_MAX        = 4
uint16 MASK_MIN_CNO        = 8
uint16 MASK_INITIAL_FIX_3D = 64
uint16 MASK_WKN_ROLL       = 512
uint16 MASK_PPP            = 8192
uint16 MASK_AOP            = 16384

uint32 reserved0      # Always set to zero
uint8 reserved1       # Always set to zero
uint8 reserved2       # Always set to zero

uint8 minSVs          # Minimum number of satellites for navigation
uint8 maxSVs          # Maximum number of satellites for navigation
uint8 minCNO          # Minimum satellite signal level for navigation

uint8 reserved5       # Always set to zero

uint8 iniFix3D        # If set to 1, initial fix must be 3D

uint8 reserved6       # Always set to zero
uint8 reserved7       # Always set to zero

uint8 ackAiding       # If set to 1, issue acknowledgements for assistance
uint16 wknRollover    # GPS rollover number

uint32 reserved8      # Always set to zero
uint8 reserved9       # Always set to zero
uint8 reserved10      # Always set to zero

uint8 usePPP          # Enable/disable PPP (on supported units)
uint8 aopCfg          # AssistNow Autonomous configuration

uint8 reserved11      # Always set to zero
uint8 reserved12      # Always set to zero

uint16 aopOrbMaxErr   # Maximum acceptable (modelled) autonomous orbit error

uint8 reserved13      # Always set to zero
uint8 reserved14      # Always set to zero
uint16 reserved3      # Always set to zero
uint32 reserved4      # Always set to zero
