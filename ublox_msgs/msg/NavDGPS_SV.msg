# see message NavDGPS

uint8 svid              # Satellite ID

uint8 flags             # Bitmask / Channel Number
uint8 CHANNEL1 = 1
uint8 CHANNEL2 = 2
uint8 CHANNEL3 = 3
uint8 CHANNEL4 = 4
uint8 CHANNEL5 = 5
uint8 CHANNEL6 = 6
uint8 CHANNEL7 = 7
uint8 CHANNEL8 = 8
uint8 DGPS = 16

uint16 ageC             # Age of latest correction data [ms]
float32 prc             # Pseudo Range Correction [m]
float32 prrc            # Pseudo Range Rate Correction [m/s]

