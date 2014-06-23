# NAV-TIMEUTC (0x01 0x21)
# UTC Time Solution
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 33

uint32 iTOW             # GPS Millisecond time of week [ms]

uint32 tAcc             # Time Accuracy Estimate [ns]
int32 nano              # Nanoseconds of second, range -1e9 .. 1e9 (UTC) [ns]
uint16 year             # Year, range 1999..2099 (UTC) [y]
uint8 month             # Month, range 1..12 (UTC) [month]
uint8 day               # Day of Month, range 1..31 (UTC) [d]
uint8 hour              # Hour of Day, range 0..23 (UTC) [h]
uint8 min               # Minute of Hour, range 0..59 (UTC) [min]
uint8 sec               # Seconds of Minute, range 0..59 (UTC) [s]

uint8 valid             # Validity Flags
uint8 VALID_TOW = 1         # Valid Time of Week
uint8 VALID_WKN = 2         # Valid Week Number
uint8 VALID_UTC = 4         # Valid Leap Seconds, i.e. Leap Seconds already known
