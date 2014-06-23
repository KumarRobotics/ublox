# CFG-CFG (0x06 0x09)
# Antenna Control Settings
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 9

uint32 clearMask          # Mask with configuration sub-sections to Clear
                          # (=Load Default Configurations to Permanent
                          # Configurations in non-volatile memory)
uint32 saveMask           # Mask with configuration sub-section to Save
                          # (=Save Current Configuration to Non-volatile
                          # Memory)
uint32 loadMask           # Mask with configuration sub-sections to Load
                          # (=Load Permanent Configurations from
                          # Non-volatile Memory to Current Configurations)

uint32 MASK_IO_PORT = 1       # Port Settings
uint32 MASK_MSG_CONF = 2      # Message Configuration
uint32 MASK_INF_MSG = 4       # INF Message Configuration
uint32 MASK_NAV_CONF = 8      # Navigation Configuration
uint32 MASK_RXM_CONF = 16     # Receiver Manager Configuration
uint32 MASK_RINV_CONF = 512   # Remote Inventory Configuration
uint32 MASK_ANT_CONF = 1024   # Antenna Configuration

uint8 deviceMask          # Mask which selects the devices for this command
uint8 DEV_BBR = 1             # device battery backed RAM
uint8 DEV_FLASH = 2           # device Flash
uint8 DEV_EEPROM = 4          # device EEPROM
uint8 DEV_SPI_FLASH = 16      # device SPI Flash
