# CFG-RATE (0x06 0x08)
# Navigation/Measurement Rate Settings
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 8

uint16 measRate           # Measurement Rate, GPS measurements are
                          # taken every measRate milliseconds [ms]
uint16 navRate            # Navigation Rate, in number of measurement
                          # cycles. On u-blox 5 and u-blox 6, this parameter
                          # cannot be changed, and is always equals 1.
uint16 timeRef            # Alignment to reference time: 0 = UTC time, 1 = GPS time
uint16 TIME_REF_UTC = 0
uint16 TIME_REF_GPS = 1
