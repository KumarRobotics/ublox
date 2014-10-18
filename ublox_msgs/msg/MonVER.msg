# MON-VER (0xB5 0x62)
# Receiver/Software Version
# Returned when the version is polled.

uint8 CLASS_ID = 10
uint8 MESSAGE_ID = 4

char[30] swVersion    # Zero-terminated software version string.
char[10] hwVersion    # Zero-terminated hardware version string.

# TODO: Implement support for dynamic char[] array in serializer.