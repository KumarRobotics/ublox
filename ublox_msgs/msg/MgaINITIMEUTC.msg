# MGA-INI-TIME_UTC(0x13 0x40)
# Initial Time Assistance
#
# This message allows the delivery of UTC time assistance to a receiver. This message is
# equivalent to the UBX-MGA-INI-TIME_GNSS message, except for the time base. See the
# description of AssistNow Online for details.

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 64

uint8 type             # Message type (0x11 for this type)
uint8 version          # Message version (0x10 for this version)
uint8 ref              #Reference to be used to set time (see graphic below)
int8 leapSecs          # Number of leap seconds since 1980 (or 0x80 =-128 if unknown)
uint16 year            # Year
uint8 month            # Month, starting at 1  
uint8 day              # Day, starting at 1
uint8 hour             # Hour, from 0 to 23
uint8 minute           # Minute, from 0 to 59
uint8 second           # Seconds, from 0 to 59
uint8 reserved1        # reserved1 
uint32 ns              # Nanoseconds, from 0 to 999,999,999
uint16 tAccS           # Seconds part of time accuracy
uint8[2] reserved2     # reserved2
uint32 tAccNs          # Nanoseconds part of time accuracy, from 0 to 999,999,999
