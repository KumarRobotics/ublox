# MGA-INI-TIME_GNSS (0x13 0x40)
# Initial Time Assistance
#
# This message allows the delivery of time assistance to a receiver in a chosen GNSS
# timebase. This message is equivalent to the UBX-MGA-INI-TIME_UTC message, except
# for the time base. See the description of AssistNow Online for details.

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 64

uint8 type              # Message type (0x11 for this type)
uint8 version           # Message version (0x00 for this version)
uint8 ref               # Reference to be used to set time
uint8 gnssId            # Source of time information. Currently supported: 0: GPS time 2: Galileo time 3: BeiDou time 6: GLONASS time: week = 834 + ((N4-1)*1461 + Nt)/7, tow = (((N4-1)*1461 + Nt) % 7) * 86400 + tod
uint8[2] reserved1      # reserved1  
uint16 week             # GNSS week number
uint32 tow              # GNSS time of week
uint32 ns               # GNSS time of week, nanosecond part from 0 to 999,999,999
uint16 tAccS            # Seconds part of time accuracy
uint8[2] reserved2      # reserved2  
uint32 tAccNs           # Nanoseconds part of time accuracy, from 0 to 999,999,999