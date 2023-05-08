# CFG-VALDEL (0x06 0x8c)
# Delete configuration item values
#
# This message is used to delete saved configurations to effectively revert the items values to
# defaults. Maximum of 64 Key IDs.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 140

uint8 version               # Message Version (0x00 for this version)

uint8 layers                # Bit mask: the layers where the configuration should be applied
uint8 LAYER_BBR = 1         # BBR layer
uint8 LAYER_FLASH = 2       # Flash Layer

uint8[2] reserved0

# Start of repeated block
uint32[] keys               # Configuration key IDs of configuration items to be retrieved
# End of repeated block
