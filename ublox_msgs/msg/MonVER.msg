# MON-VER (0x0A 0x04)
#
# Receiver/Software Version
# Returned when the version is polled.

uint8 CLASS_ID = 10
uint8 MESSAGE_ID = 4

char[30] sw_version    # Zero-terminated software version string.
char[10] hw_version    # Zero-terminated hardware version string.

# Start of repeated block (N times)
MonVERExtension[] extension
# End of repeated block
