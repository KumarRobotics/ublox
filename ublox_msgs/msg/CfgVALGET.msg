# CFG-VALGET (0x06 0x8b)
# Get configuration items
#
# This message is used to get configuration values by providing a list of configuration key IDs,
# which identify the configuration items to retrieve. Maximum of 64 Key IDs.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 139

uint8 version               # Message Version (0x00 for this version)

uint8 layers                # The layer which the configuration items should be retrieved
uint8 LAYER_RAM = 0         # RAM layer
uint8 LAYER_BBR = 1         # BBR layer
uint8 LAYER_FLASH = 2       # Flash Layer
uint8 LAYER_DEFAULT = 7     # Default Layer

uint16 position             # Skip this many key values before constructing output message

# Start of repeated block
uint32[] keys               # Configuration key IDs of configuration items to be retrieved
# End of repeated block
