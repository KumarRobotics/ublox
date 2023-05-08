# CFG-VALSET (0x06 0x8a)
# Set configuration item values
#
# This message is used to set a configuration by providing configuration data ( a list of
# key and value pairs), which identify the configuration items to change, and their new values.
# Maximum of 64 Key IDs.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 138

uint8 version               # Message Version (0x00 for this version)

uint8 layers                # Bit mask: the layers where the configuration should be applied
uint8 LAYER_RAM = 1         # RAM layer
uint8 LAYER_BBR = 2         # BBR layer
uint8 LAYER_FLASH = 4       # Flash Layer

uint8[2] reserved0

# Start of repeated block
CfgVALSETCfgdata[] cfgdata    # Configuration data (key and value pairs)
# End of repeated block
