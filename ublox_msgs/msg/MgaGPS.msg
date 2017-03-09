# MGA-GPS (0x13 0x00)
# GPS Ephemeris Assistance
#
# This message allows the delivery of GPS ephemeris assistance to a receiver. See the
# description of AssistNow Online for details.

uint8 type # Message type (0x01 for this type)
uint8 version # Message version (0x00 for this version)
uint8 svId # GPS Satellite identifier (see Satellite Numbering)
uint8 reserved1 # Reserved
uint8 fitInterval # Fit interval flag
uint8 uraIndex # URA index
uint8 svHealth # SV health
int8 tgd # Group delay differential # scaling:2^-31
uint16 iodc # IODC
uint16 toc # Clock data reference time # scaling 2^4
uint8 reserved2
int8 af2 # Time polynomial coefficient 2 # scaling 2^-55
int16 af1 # Time polynomial coefficient 1 # scaling 2^-43
int32 af0 # Time polynomial coefficient 0 # scaling 2^-31
int16 crs # Crs # scaling 2^-5
int16 deltaN # Mean motion difference from computed value # scaling 2^-43
int32 m0 # Mean anomaly at reference time # sacling 2^-31
int16 cuc # Amplitude of cosine harmonic correction term to argument of latitude # scaling 2^-29
int16 cus # Amplitude of sine harmonic correction term to argument of latitude # scaling 2^-29
uint32 e # Eccentricity # scaling 2^-33
uint32 sqrtA # Square root of the semi-major axis # scaling 2^-19
uint16 toe # Reference time of ephemeris # scaling 2^4
int16 cic # Amplitude of cos harmonic correction term to angle of inclination # scaling 2^-29
int32 omega0 # Longitude of ascending node of orbit plane at weekly epoch # scaling 2^-31
int16 cis # Amplitude of sine harmonic correction term to angle of inclination # scaling 2^-29
int16 crc # Amplitude of cosine harmonic correction term to orbit radius # scaling 2^-5
int32 i0 # Inclination angle at reference time # scaling 2^-31
int32 omega # Argument of perigee # scaling 2^-31
int32 omegaDot # Rate of right ascension # scaling 2^-43
int16 idot # Rate of inclination angle # scaling 2^-43
uint8[2] reserved3
