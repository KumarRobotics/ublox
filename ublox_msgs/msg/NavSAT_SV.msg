# see NAV-SAT message

uint8 gnssId      # GNSS identifier
uint8 svId        # Satellite identifier
uint8 cno         # Carrier to noise ratio (signal strength) ]dBHz
int8 elev         # Elevation (range: +/-90), unknown if out of range [deg]
int16 azim        # Azimuth (range 0-360), unknown if elevation is out of range 
                  # [deg]
int16 prRes       # Pseudo range residual [0.1 m]
uint32 flags      # Bitmask (see graphic below)
